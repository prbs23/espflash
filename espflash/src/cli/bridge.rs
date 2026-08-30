//! Support for connecting through an ESP32 debug/programming bridge over
//! WiFi, instead of a local serial port.
//!
//! This is the *only* part of this fork of `espflash` that isn't upstream -
//! everything else (every CLI command, every flag, all of
//! `bin/espflash.rs`'s dispatch logic) is untouched, save for `Port`
//! becoming a `Box<dyn PortLike>` in `connection/mod.rs` so [`TcpSerialPort`]
//! below can stand in for a real serial port (see that type's doc comment
//! for why). This module is called from a single site in [`super::connect`]
//! when `--port` names a bridge (`bridge://<host>`) instead of a real
//! serial device; every other command's logic elsewhere in this crate then
//! runs completely unmodified against the [`Connection`] this hands back.
//! See `/home/kit/esp32/esp32_debug_bridge/DESIGN.md` for the bridge
//! project this talks to.
//!
//! The bridge doesn't expose anything `espflash`'s normal port-discovery
//! (`serialport::available_ports()`, which only finds real, OS-visible
//! serial hardware) could ever find, so there was no way to make this work
//! by only changing *how* a port gets picked - a target-side connection has
//! to be built by hand, wrapping a raw `TcpStream` to the bridge's flashing
//! socket in [`TcpSerialPort`] so it satisfies `Connection`'s `Port` bound,
//! with reset/bootloader-entry driven over the bridge's separate HTTP
//! control endpoints rather than DTR/RTS (which have no physical meaning on
//! a plain TCP socket).

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::os::fd::{AsRawFd, RawFd};
use std::time::Duration;

use log::info;
use miette::{IntoDiagnostic, Result, WrapErr};
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits, UsbPortInfo};

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

/// Baud rate reported to `espflash` for the ROM bootloader handshake - the
/// bridge firmware's fixed target-UART baud (see `bridge_fw`'s README), not
/// anything this module's [`TcpSerialPort`] actually configures (there's no
/// real UART on this end to configure). The bridge doesn't renegotiate
/// mid-session, so callers should pass `--baud` no higher than this (or
/// leave it unset).
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
/// equivalent bridge HTTP calls rather than DTR/RTS, since [`TcpSerialPort`]
/// has no real control lines: entering bootloader mode happens here, before
/// the socket connects, for any `before` other than `NoResetNoSync`
/// (matching upstream's own "skip the reset, but still sync" vs. "skip
/// both" distinction).
///
/// `NoResetNoSync` specifically is also preserved into the returned
/// `Connection`'s own `before_operation` (every other value collapses to
/// `NoReset` - see the comment below) rather than always forcing `NoReset`:
/// `Flasher::try_connect` only skips its ROM sync/chip-detect handshake
/// when `before_operation() == NoResetNoSync`, not merely `NoReset`. Found
/// the hard way 2026-08-29: with the HTTP `/enter-bootloader` call above
/// skipped (correctly) but `before_operation` always reported as `NoReset`,
/// `try_connect` still attempted to sync - against the target's *running
/// application firmware*, since it was never actually put in the
/// bootloader - and failed with a generic "Failed to connect to the
/// device" after exhausting its retries. Callers using `NoResetNoSync`
/// against a bridge must still pass `--chip` explicitly, same as upstream
/// already requires for a real serial port with this flag (see
/// `Error::ChipNotProvided`).
///
/// `after` isn't acted on here - it's stored on the returned `Connection`
/// (as `after_operation`, same as a normal connection) purely so
/// `Connection::reset_after` can see it later and know whether to call
/// [`hard_reset`] once the caller is actually done with the connection;
/// see that struct's `bridge_host` field.
pub fn connect(
    host: &str,
    before: ResetBeforeOperation,
    after: ResetAfterOperation,
) -> Result<Connection> {
    if before != ResetBeforeOperation::NoResetNoSync {
        http_post(host, "/enter-bootloader")
            .wrap_err("failed to put target into bootloader mode via bridge")?;
    }

    let port = flash_port();
    info!("Bridge: connecting to {host}:{port}");
    let stream = TcpStream::connect((host, port))
        .into_diagnostic()
        .wrap_err("failed to connect to bridge flashing socket")?;

    // No real USB device backs this connection, so there's no vendor/
    // product info to report.
    let usb_info = UsbPortInfo {
        vid: 0,
        pid: 0,
        serial_number: None,
        manufacturer: None,
        product: None,
    };

    // `before_operation` collapses every value except `NoResetNoSync` to
    // `NoReset`: bootloader-entry already happened over HTTP above, and
    // `Connection` must not *also* try to drive DTR/RTS over
    // `TcpSerialPort`, which has no real control lines. `NoResetNoSync`
    // itself is passed straight through instead of also collapsing to
    // `NoReset` - see this fn's doc comment for why that distinction
    // matters. `after_operation` keeps the caller's real value regardless -
    // `reset_after` needs to see it to know whether to call `hard_reset`
    // below once the caller is done with the connection.
    let connection_before = match before {
        ResetBeforeOperation::NoResetNoSync => ResetBeforeOperation::NoResetNoSync,
        _ => ResetBeforeOperation::NoReset,
    };
    let mut connection = Connection::new(
        Box::new(TcpSerialPort::new(stream, host.to_string())),
        usb_info,
        after,
        connection_before,
        BAUD,
    );
    // `bridge_host` is `pub(crate)`, not exposed through `Connection::new`
    // (an upstream-facing constructor) - set directly since this module is
    // in the same crate. This is what makes `reset_after` (in
    // `connection/mod.rs`) redirect a `HardReset` to `hard_reset` below
    // instead of trying to drive DTR/RTS on `TcpSerialPort`.
    connection.bridge_host = Some(host.to_string());
    *ACTIVE_BRIDGE_HOST.lock().unwrap() = Some(host.to_string());
    Ok(connection)
}

/// A minimal `serialport::SerialPort` implementation backed directly by a
/// `TcpStream`, standing in for a real serial device when connected through
/// an ESP32 debug bridge (see this module's doc comment).
///
/// `espflash`'s `Connection` doesn't accept an arbitrary `Read + Write` - a
/// `Port` has to implement the *whole* `SerialPort` trait (line settings,
/// control-line queries, buffer control, ...), even though essentially none
/// of that means anything on a plain TCP connection: the target UART's
/// actual line settings are fixed in `bridge_fw` hardware, not renegotiated
/// per-connection, and control-line queries (DTR/RTS/CTS/...) have no
/// physical referent here at all - reset/bootloader-entry go over the
/// bridge's separate HTTP endpoints instead (see [`hard_reset`] and this
/// module's [`connect`]). So every line-setting/control-line method below
/// is a harmless stub; only `Read`/`Write` (the actual byte relay),
/// `set_timeout`/`timeout`, and `clear`/`bytes_to_read` (used on every
/// command - see their doc comments) do real work.
///
/// This replaces an earlier PTY-based shim (a real `posix_openpt`-allocated
/// pty device, with two background threads copying bytes between the pty
/// and this same TCP socket) that existed only because `Connection`'s
/// `serial` field used to be hardcoded to the concrete `TTYPort` type. Now
/// that `Port` accepts anything satisfying `connection::PortLike`, there's
/// no reason to synthesize an OS-level tty at all - this wraps the socket
/// directly, with no separate copy threads (`Connection` only ever reads or
/// writes `serial` from one thread at a time, never both at once, so a
/// single owned `TcpStream` is enough).
struct TcpSerialPort {
    stream: TcpStream,
    /// The bridge host this is connected to, purely for `SerialPort::name`
    /// (used in a couple of diagnostic/log call sites) - not read back for
    /// anything functional.
    host: String,
    /// Mirrors whatever `set_timeout` last set, so `timeout()` (`&self`)
    /// can report it back without an extra syscall. `Connection::with_timeout`
    /// reads this before every command to save/restore the previous value.
    timeout: Duration,
}

impl TcpSerialPort {
    fn new(stream: TcpStream, host: String) -> Self {
        Self {
            stream,
            host,
            timeout: Duration::ZERO,
        }
    }
}

impl Read for TcpSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.stream.read(buf) {
            // A real tty's VTIME-based read timeout (what `TTYPort` uses,
            // and what every retry loop in `connection/mod.rs` -
            // `sync`'s `MAX_CONNECT_ATTEMPTS`, `SlipDecoder`'s callers,
            // etc. - is written assuming) returns `Ok(0)`, never an error:
            // "nothing arrived in time" looks exactly like "nothing
            // arrived yet". A `TcpStream`'s `SO_RCVTIMEO` expiry (or a
            // nonblocking read finding nothing ready) instead surfaces as
            // `WouldBlock`/`TimedOut` - translate both back to the tty
            // convention so callers don't need a bridge-specific case.
            // Found the hard way 2026-08-29: without this, `detect_sdm`'s
            // deliberately-expected-to-sometimes-fail probes (and any
            // other genuine no-response timeout) surfaced as a raw
            // "Resource temporarily unavailable (os error 11)" instead of
            // being handled as a normal, silent timeout.
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {
                Ok(0)
            }
            other => other,
        }
    }
}

impl Write for TcpSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.stream.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.stream.flush()
    }
}

impl AsRawFd for TcpSerialPort {
    fn as_raw_fd(&self) -> RawFd {
        self.stream.as_raw_fd()
    }
}

impl SerialPort for TcpSerialPort {
    fn name(&self) -> Option<String> {
        Some(format!("bridge://{}", self.host))
    }

    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(BAUD)
    }

    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }

    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }

    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }

    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> serialport::Result<()> {
        // The bridge's target UART runs at a single fixed baud (see
        // `BAUD`'s doc comment) that isn't renegotiated per-connection.
        // Accepting silently (rather than erroring) matches what callers
        // expect from a normal serial-port baud change that happens to be
        // a no-op.
        Ok(())
    }

    fn set_data_bits(&mut self, _data_bits: DataBits) -> serialport::Result<()> {
        Ok(())
    }

    fn set_flow_control(&mut self, _flow_control: FlowControl) -> serialport::Result<()> {
        Ok(())
    }

    fn set_parity(&mut self, _parity: Parity) -> serialport::Result<()> {
        Ok(())
    }

    fn set_stop_bits(&mut self, _stop_bits: StopBits) -> serialport::Result<()> {
        Ok(())
    }

    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        // `TcpStream::set_read_timeout`/`set_write_timeout` panic on
        // exactly `Duration::ZERO` ("the timeout is not allowed to be
        // zero") - unlike a real serial port, where a zero timeout is the
        // normal way to ask for a non-blocking read (this crate's own
        // `serialport::new()` builder defaults to it). Map that case to
        // real socket non-blocking mode instead, which is the actual
        // behavioral equivalent; any nonzero timeout is a normal blocking
        // read/write with a deadline.
        if timeout.is_zero() {
            self.stream.set_nonblocking(true).map_err(serialport::Error::from)?;
        } else {
            self.stream.set_nonblocking(false).map_err(serialport::Error::from)?;
            self.stream.set_read_timeout(Some(timeout)).map_err(serialport::Error::from)?;
            self.stream.set_write_timeout(Some(timeout)).map_err(serialport::Error::from)?;
        }
        self.timeout = timeout;
        Ok(())
    }

    fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
        Ok(())
    }

    fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
        Ok(())
    }

    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }

    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }

    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }

    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> {
        // `FIONREAD` reports the kernel receive buffer's current depth for
        // any stream-like fd, sockets included - same ioctl `TTYPort` uses
        // on a tty fd. Used by `clear` below (and, on a real serial
        // connection, by `connect_attempt`'s boot-log sniffing - dead code
        // for a bridge connection specifically, since `before_operation` is
        // always forced to `NoReset` in `connect` above, but implemented
        // properly anyway rather than assuming that never changes).
        let mut n: libc::c_int = 0;
        let ret = unsafe { libc::ioctl(self.stream.as_raw_fd(), libc::FIONREAD, &mut n) };
        if ret != 0 {
            return Err(serialport::Error::from(io::Error::last_os_error()));
        }
        Ok(n as u32)
    }

    fn bytes_to_write(&self) -> serialport::Result<u32> {
        // Not called anywhere in this crate today (checked: only
        // `bytes_to_read` is, for `clear`'s input-drain below).
        Ok(0)
    }

    fn clear(&self, buffer_to_clear: ClearBuffer) -> serialport::Result<()> {
        // `Connection::write_command`/`write_raw` call this unconditionally
        // before *every* command, real serial port or bridge alike, to
        // discard any stale bytes left over from a previous exchange - so
        // unlike most of this impl, this one is actually exercised on the
        // hot path. There's no `tcflush` equivalent for a plain socket, so
        // this mirrors `TTYPort::clear`'s effect by reading and discarding
        // exactly however many bytes `FIONREAD` currently reports: that
        // many bytes are already sitting in the kernel receive buffer, so
        // the read below completes immediately without blocking.
        if matches!(buffer_to_clear, ClearBuffer::Input | ClearBuffer::All) {
            let n = self.bytes_to_read()?;
            if n > 0 {
                let mut discard = vec![0u8; n as usize];
                (&self.stream).read_exact(&mut discard).map_err(serialport::Error::from)?;
            }
        }
        // `ClearBuffer::Output` has nothing to do: every `write()` on
        // `TcpSerialPort` already goes straight to the socket, with no
        // buffering layer of our own to discard.
        Ok(())
    }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(TcpSerialPort::new(
            self.stream.try_clone().map_err(serialport::Error::from)?,
            self.host.clone(),
        )))
    }

    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }

    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

/// Reboots the target by hitting the bridge's HTTP `/reset` endpoint.
/// Called from `Connection::reset_after` (see `bridge_host`'s doc comment
/// on the `Connection` struct) in place of the normal DTR/RTS-based
/// `HardReset` handling, which can't do anything useful on `TcpSerialPort`
/// with no real control lines.
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
