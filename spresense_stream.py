"""Best-effort GPS CSV output; the worker owns all Spresense USB I/O.

Automatic discovery deliberately requires a Spresense descriptor or an exact
configured USB serial number. The common CP210x VID/PID is not a unique device
identity. An explicit /dev/serial/by-id path is also supported. Once connected,
USB VID/PID/serial metadata pins the device across reconnects when available.
"""

import csv
import io
import logging
import os
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


def same_serial_port(a, b):
    """Compare device paths, including stable symlink aliases."""
    return bool(a and b) and os.path.realpath(a) == os.path.realpath(b)


def matches_spresense(port_info, serial_number=None):
    """Match a configured serial number, or an explicit Spresense descriptor."""
    if serial_number:
        return getattr(port_info, "serial_number", None) == serial_number
    description = " ".join(
        str(getattr(port_info, name, None) or "")
        for name in ("description", "manufacturer", "product")
    ).lower()
    return "spresense" in description


def reserved_spresense_ports(port="auto", serial_number=None, ports=None):
    """Return every potential destination to exclude from RC/GPS discovery."""
    if ports is None:
        ports = list_ports.comports() if list_ports is not None else ()
    reserved = [port] if port and port != "auto" else []
    for info in ports:
        if matches_spresense(info, serial_number):
            if not any(same_serial_port(info.device, item) for item in reserved):
                reserved.append(info.device)
    return tuple(reserved)


def format_gps_csv(sample):
    """Encode one sample, preserving its original timestamp, with one LF."""
    output = io.StringIO(newline="")
    csv.writer(output, lineterminator="\n").writerow((
        sample["timestamp"],
        sample["fix_quality"],
        "" if sample["latitude"] is None else f"{sample['latitude']:.8f}",
        "" if sample["longitude"] is None else f"{sample['longitude']:.8f}",
        sample["num_sats"],
        "" if sample["hdop"] is None else sample["hdop"],
    ))
    return output.getvalue().encode("ascii")


class SpresenseGPSStreamer:
    """Send the latest sample every five seconds without queuing old samples.

    sample_provider must return a copy of a coherent GPS sample, or None until
    the first sample arrives. Its timestamp is never regenerated here. The
    provider should hold its data lock only while making that copy.
    """

    INTERVAL_SEC = 5.0
    RETRY_SEC = 1.0
    WRITE_TIMEOUT_SEC = 0.5
    WARNING_INTERVAL_SEC = 30.0

    def __init__(self, sample_provider, port="auto", serial_number=None,
                 excluded_ports=(), logger=None):
        self.sample_provider = sample_provider
        self.port = port or "auto"
        self.serial_number = serial_number
        self.excluded_ports = tuple(excluded_ports)
        self.logger = logger
        self.thread = None
        self._stop_event = threading.Event()
        self._serial = None
        self._identity = None
        self._warning = None
        self._warning_time = 0.0

    def _log(self, message, level=logging.INFO):
        # Diagnostics must not terminate this optional worker, even when an
        # application's logger itself is being shut down.
        try:
            if self.logger is None:
                logging.getLogger(__name__).log(level, message)
            else:
                self.logger(message, level)
        except Exception:
            pass

    def _warn(self, message):
        now = time.monotonic()
        if message != self._warning or now - self._warning_time >= self.WARNING_INTERVAL_SEC:
            self._log(f"[SPRESENSE] {message}", logging.WARNING)
            self._warning, self._warning_time = message, now

    @staticmethod
    def _usb_identity(info):
        number = getattr(info, "serial_number", None)
        if not number:
            return None
        return (getattr(info, "vid", None), getattr(info, "pid", None), number)

    def _is_excluded(self, path):
        return any(same_serial_port(path, used) for used in self.excluded_ports)

    def _discover(self):
        """Return (path, metadata); never guess between multiple matches."""
        ports = list(list_ports.comports()) if list_ports is not None else []
        if self._identity is not None:
            candidates = [p for p in ports if self._usb_identity(p) == self._identity]
        elif self.port != "auto":
            if self._is_excluded(self.port):
                self._warn("Configured output port overlaps the Arduino/GPS port; output disabled until corrected.")
                return None
            candidates = [p for p in ports if same_serial_port(p.device, self.port)]
            if self.serial_number:
                candidates = [p for p in candidates if matches_spresense(p, self.serial_number)]
                if not candidates:
                    self._warn("Configured output path does not match the requested USB serial number; waiting.")
                    return None
            elif not candidates:
                # A stable path still works on platforms without USB metadata.
                # Opening it will fail and retry if the device is absent.
                return self.port, None
        else:
            candidates = [p for p in ports if matches_spresense(p, self.serial_number)]

        unique = []
        for info in candidates:
            if not self._is_excluded(info.device) and not any(
                    same_serial_port(info.device, other.device) for other in unique):
                unique.append(info)
        if not unique:
            self._warn("No matching free USB port; waiting for Spresense. Set --spresense-port or --spresense-serial-number for a generic USB bridge.")
            return None
        if len(unique) != 1:
            self._warn("Multiple matching USB ports; set --spresense-port or --spresense-serial-number to select one.")
            return None
        return unique[0].device, unique[0]

    def _connect(self):
        selected = self._discover()
        if selected is None:
            return False
        path, info = selected
        self._serial = serial.Serial(
            port=path, baudrate=115200,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0,
            write_timeout=self.WRITE_TIMEOUT_SEC,
            xonxoff=False, rtscts=False, dsrdtr=False,
            exclusive=True,
        )
        if info is not None and self._identity is None:
            self._identity = self._usb_identity(info)
        self._warning = None
        self._log(f"[SPRESENSE] Connected to {path}: 115200 baud, 8N1; GPS CSV every 5 seconds.")
        return True

    def _close(self):
        connection, self._serial = self._serial, None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def _run(self):
        if serial is None:
            self._warn("pyserial is unavailable; GPS USB output is disabled.")
            return
        next_send = time.monotonic()
        try:
            while not self._stop_event.is_set():
                try:
                    if self._serial is None and not self._connect():
                        self._stop_event.wait(self.RETRY_SEC)
                        continue
                    delay = next_send - time.monotonic()
                    if delay > 0:
                        self._stop_event.wait(delay)
                        continue
                    send_started = time.monotonic()
                    sample = self.sample_provider()
                    if sample is not None:
                        payload = format_gps_csv(sample)
                        written = self._serial.write(payload)
                        if written != len(payload):
                            raise OSError(f"Incomplete CSV write ({written}/{len(payload)} bytes)")
                    next_send = send_started + self.INTERVAL_SEC
                    # Missed intervals are dropped; never burst stale samples.
                    if next_send <= time.monotonic():
                        next_send = time.monotonic() + self.INTERVAL_SEC
                except Exception as exc:
                    self._warn(f"USB output unavailable: {exc}; retrying.")
                    self._close()
                    next_send = time.monotonic() + self.INTERVAL_SEC
                    self._stop_event.wait(self.RETRY_SEC)
        finally:
            self._close()

    def start(self):
        """Start without enumerating, opening, or writing any USB port here."""
        if self.thread is not None and self.thread.is_alive():
            return
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="spresense-gps", daemon=True)
        self.thread.start()

    def stop(self):
        """Wake the worker and wait a bounded time; the worker closes its port."""
        self._stop_event.set()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=self.WRITE_TIMEOUT_SEC + 1.0)
