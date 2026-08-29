//! Support for connecting through an ESP32 debug/programming bridge over
//! WiFi, instead of a local serial port.
//!
//! This is the *only* part of this fork of `espflash` that isn't upstream -
//! everything else (every CLI command, every flag, all of
//! `bin/espflash.rs`'s dispatch logic) is untouched. This module is called
//! from a single site in [`super::connect`] when `--port` names a bridge
//! (`bridge://<host>`) instead of a real serial device; every other
//! command's logic elsewhere in this crate then runs completely unmodified
//! against the [`Connection`] this hands back. See
//! `/home/kit/esp32/esp32_debug_bridge/DESIGN.md` for the bridge project
//! this talks to.
//!
//! The bridge doesn't expose anything `espflash`'s normal port-discovery
//! (`serialport::available_ports()`, which only finds real, OS-visible
//! serial hardware) could ever find, so there was no way to make this work
//! by only changing *how* a port gets picked - a target-side connection had
//! to be built by hand: a local PTY fronting the bridge's raw flashing TCP
//! socket (see "Client transport: PTY shim" in DESIGN.md), with
//! reset/bootloader-entry driven over the bridge's separate HTTP control
//! endpoints rather than DTR/RTS (which don't exist on a PTY).

use std::fs::File;
use std::io;
use std::net::TcpStream;
use std::os::fd::OwnedFd;
use std::thread;

use log::info;
use miette::{IntoDiagnostic, Result, WrapErr};
use nix::fcntl::OFlag;
use nix::pty::{grantpt, posix_openpt, ptsname_r, unlockpt};
use serialport::{TTYPort, UsbPortInfo};

use crate::connection::Connection;
use crate::connection::reset::{ResetAfterOperation, ResetBeforeOperation};

/// `--port` prefix that selects a bridge target instead of a real serial
/// device, e.g. `--port bridge://192.168.1.50`.
const PORT_PREFIX: &str = "bridge://";

/// The host of whatever bridge connection is currently active, if any. Set
/// by [`connect`] and read by call sites that need to know "should DTR/RTS
/// be replaced with a bridge HTTP call right now" but don't have a
/// [`Connection`] on hand to check `bridge_host` on directly - namely
/// `cli::monitor`'s own `reset_after_flash` calls, which operate on a raw
/// `Port` (see its doc comment). `Connection::reset`/`reset_after` check
/// their own `bridge_host` field instead and don't need this; it exists
/// only for the places that can't.
static ACTIVE_BRIDGE_HOST: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// Returns the host of the currently-active bridge connection, if any -
/// see [`ACTIVE_BRIDGE_HOST`].
pub(crate) fn active_host() -> Option<String> {
    ACTIVE_BRIDGE_HOST.lock().unwrap().clone()
}

/// Baud rate for both the PTY and the ROM bootloader handshake. Must match
/// the bridge firmware's fixed target-UART baud (see `bridge_fw`'s
/// README) - the bridge doesn't renegotiate mid-session, so callers should
/// pass `--baud` no higher than this (or leave it unset).
pub const BAUD: u32 = 115_200;

/// Returns `Some(host)` if `port` names a bridge target.
pub fn target_host(port: &str) -> Option<&str> {
    port.strip_prefix(PORT_PREFIX)
}

/// The bridge's HTTP control port, overridable via `ESPBRIDGE_HTTP_PORT`
/// for a bridge not using `bridge_fw`'s default.
fn http_port() -> u16 {
    env_port("ESPBRIDGE_HTTP_PORT", 80)
}

/// The bridge's raw flashing TCP port, overridable via
/// `ESPBRIDGE_FLASH_PORT`.
fn flash_port() -> u16 {
    env_port("ESPBRIDGE_FLASH_PORT", 3333)
}

fn env_port(var: &str, default: u16) -> u16 {
    std::env::var(var)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

/// Establishes a `Connection` to a target through the bridge at `host`.
///
/// `before`/`after` (from `ConnectArgs`, i.e. whatever the user passed to
/// `--before`/`--after`) are honored by translating them into the
/// equivalent bridge HTTP calls rather than DTR/RTS, since the PTY this
/// hands back has no real control lines to drive: entering bootloader mode
/// happens here, before the socket connects, for any `before` other than
/// `NoResetNoSync` (matching upstream's own "skip the reset, but still
/// sync" vs. "skip both" distinction). `after` isn't acted on here - it's
/// stored on the returned `Connection` (as `after_operation`, same as a
/// normal connection) purely so `Connection::reset_after` can see it later
/// and know whether to call [`hard_reset`] once the caller is actually
/// done with the connection; see that struct's `bridge_host` field.
pub fn connect(
    host: &str,
    before: ResetBeforeOperation,
    after: ResetAfterOperation,
) -> Result<Connection> {
    if before != ResetBeforeOperation::NoResetNoSync {
        http_post(host, "/enter-bootloader")
            .wrap_err("failed to put target into bootloader mode via bridge")?;
    }

    let master =
        posix_openpt(OFlag::O_RDWR | OFlag::O_NOCTTY).into_diagnostic().wrap_err("posix_openpt failed")?;
    grantpt(&master).into_diagnostic().wrap_err("grantpt failed")?;
    unlockpt(&master).into_diagnostic().wrap_err("unlockpt failed")?;
    let slave_path = ptsname_r(&master).into_diagnostic().wrap_err("ptsname_r failed")?;

    let port = flash_port();
    info!("Bridge: PTY shim at {slave_path}, connecting to {host}:{port}");

    let tcp_writer = TcpStream::connect((host, port))
        .into_diagnostic()
        .wrap_err("failed to connect to bridge flashing socket")?;
    let tcp_reader = tcp_writer
        .try_clone()
        .into_diagnostic()
        .wrap_err("failed to clone bridge flashing socket")?;

    let pty_reader: File = OwnedFd::from(master).into();
    let pty_writer = pty_reader
        .try_clone()
        .into_diagnostic()
        .wrap_err("failed to clone PTY fd")?;

    // Fire-and-forget copy threads: there's nothing to flush on shutdown,
    // and the whole process exits once whatever command invoked us is
    // done, closing the PTY and TCP socket - the bridge notices the
    // disconnect and hands the target UART back to its log-relay task on
    // its own (see bridge_fw/src/target_link.rs).
    spawn_copy(pty_reader, tcp_writer, "target->client");
    spawn_copy(tcp_reader, pty_writer, "client->target");

    let tty = TTYPort::open(&serialport::new(&slave_path, BAUD))
        .into_diagnostic()
        .wrap_err_with(|| format!("failed to open PTY slave {slave_path}"))?;

    // No real USB device backs this connection, so there's no vendor/
    // product info to report.
    let usb_info = UsbPortInfo {
        vid: 0,
        pid: 0,
        serial_number: None,
        manufacturer: None,
        product: None,
    };

    // `before_operation` is forced to `NoReset` regardless of what the
    // caller asked for: bootloader-entry already happened over HTTP above
    // (or was explicitly skipped, for `NoResetNoSync`), and `Connection`
    // must not *also* try to drive DTR/RTS over the PTY, which has no real
    // control lines. `after_operation` keeps the caller's real value,
    // though - `reset_after` needs to see it to know whether to call
    // `hard_reset` below once the caller is done with the connection.
    let mut connection = Connection::new(
        tty,
        usb_info,
        after,
        ResetBeforeOperation::NoReset,
        BAUD,
    );
    // `bridge_host` is `pub(crate)`, not exposed through `Connection::new`
    // (an upstream-facing constructor) - set directly since this module is
    // in the same crate. This is what makes `reset_after` (in
    // `connection/mod.rs`) redirect a `HardReset` to `hard_reset` below
    // instead of trying to drive DTR/RTS on the PTY.
    connection.bridge_host = Some(host.to_string());
    *ACTIVE_BRIDGE_HOST.lock().unwrap() = Some(host.to_string());
    Ok(connection)
}

/// Reboots the target by hitting the bridge's HTTP `/reset` endpoint.
/// Called from `Connection::reset_after` (see `bridge_host`'s doc comment
/// on the `Connection` struct) in place of the normal DTR/RTS-based
/// `HardReset` handling, which can't do anything useful on a PTY with no
/// real control lines.
pub(crate) fn hard_reset(host: &str) -> std::result::Result<(), crate::error::Error> {
    http_post(host, "/reset").map_err(|e| crate::error::Error::Connection(Box::new(BridgeError(e))))
}

fn http_post(host: &str, path: &str) -> Result<()> {
    let url = format!("http://{host}:{}{path}", http_port());
    ureq::post(&url)
        .send_empty()
        .into_diagnostic()
        .wrap_err_with(|| format!("POST {url} failed"))?;
    Ok(())
}

/// Wraps a `miette::Report` so it can be boxed into `Error::Connection`'s
/// `Box<dyn core::error::Error + Send + Sync>` payload - `miette::Report`
/// itself doesn't implement `std::error::Error`.
#[derive(Debug)]
struct BridgeError(miette::Report);

impl std::fmt::Display for BridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for BridgeError {}

fn spawn_copy(
    mut from: impl io::Read + Send + 'static,
    mut to: impl io::Write + Send + 'static,
    label: &'static str,
) {
    thread::spawn(move || {
        if let Err(e) = io::copy(&mut from, &mut to) {
            info!("Bridge: {label} copy loop ended: {e}");
        }
    });
}
