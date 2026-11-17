# ArbaLabs GPS USB stream

The boat controller sends its latest ZED-F9P GGA observation to the Spresense at
921600 baud, 8 data bits, no parity, 1 stop bit, and no flow control. Discovery,
connection, writes, retries, and closing run in a dedicated background thread.
The 20 Hz control loop does not write to this USB port. The 921600 baud rate
matches the reported ArbaLabs receiver setting; both ends must use this rate.
The Arduino and ZED-F9P connections retain their separate baud settings.

## CSV protocol

Each message is one ASCII CSV row ending in LF (`\n`), without a header:

```text
timestamp,fix_quality,latitude,longitude,num_sats,hdop
```

Example transmitted row:

```text
043015.20,4,35.12345678,129.12345678,18,0.7
```

- `timestamp` preserves the original GGA UTC time exactly, including leading
  zeros and fractional precision: `hhmmss.ss` as supplied by the receiver. It is
  time of day, without a date, and rolls over at UTC midnight. It is neither a
  Unix timestamp nor the USB transmission time.
- Latitude and longitude are signed decimal degrees with eight decimal places.
- Fix quality, satellite count, and HDOP come from the same GGA sentence.
  A no-fix GGA sends quality `0` and blank latitude/longitude, rather than the
  previous navigation position. Missing HDOP is blank.
- The first row may be sent immediately after connection; subsequent rows are
  sent every five seconds. Nothing is sent before the first GPS observation.
- When GPS stops updating, the latest row repeats with its original timestamp
  and quality. ArbaLabs should use timestamp progression to detect stale data.
  Historical samples are never queued for later replay.

## Identify the USB device once

With the Spresense plugged into the Jetson, list ports using the controller's
existing pySerial dependency:

```sh
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
```

Compare the list with the Spresense unplugged to identify its port and `SER`
value. Prefer its unique USB serial number:

```sh
python3 multipath_boat.py --spresense-serial-number YOUR_DEVICE_SERIAL
```

Alternatively, use its actual stable device path:

```sh
python3 multipath_boat.py --spresense-port /dev/serial/by-id/YOUR_SPRESENSE_DEVICE
```

Replace the placeholders with the values from your Jetson. These commands start
the normal boat controller, including its normal PWM initialization. For normal
launches without extra arguments, add `spresense_serial_number` or
`spresense_port` to the existing `hardware` object in `calibration.json`.

The stream is enabled by default. Without a configured selector it connects only
when exactly one free port explicitly identifies itself as Spresense in its USB
description, manufacturer, or product. A configured serial number matches exactly.
Ambiguous matches wait for a selector instead of choosing an arbitrary port.

The stock main-board USB connection uses a Silicon Labs CP210x bridge, which can
have a generic description shared by unrelated devices. Such a device needs the
one-time selector above. ArbaLabs firmware must read this serial interface at
921600 baud, 8N1, no flow control;
connecting the board alone does not install a CSV receiver.
See [Sony SDK setup](https://developer.spresense.sony-semicon.com/development-guides/?lang=en&page=sdk_set_up).

The selected Spresense is excluded from Arduino/GPS discovery, including symlink
aliases. Generic Silicon Labs ports are also excluded from Arduino fallback while
streaming is enabled. If the RC board itself uses a generic Silicon Labs bridge,
select it explicitly with `--arduino-port` and select the Spresense separately.
The USB output worker always excludes the active Arduino and GPS ports.

## Reconnection and shutdown

An absent or disconnected Spresense does not stop navigation. The worker retries
discovery/connection every second, with rate-limited diagnostics in the terminal
and session event log. When available, USB VID/PID/serial metadata pins the device
after connection so reconnects can follow a changed `/dev/ttyUSB*` number.
Without metadata, use a stable `/dev/serial/by-id/` path.

Writes have a 0.5-second timeout. Failed or partial writes close the connection
and trigger reconnect; they do not replay old samples. A physical disconnect can
truncate an in-flight row, so the receiving firmware should discard incomplete or
malformed rows when reconnecting. The worker does not call an unbounded serial
flush. See the [pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html).

Ctrl+C stops the stream during normal controller shutdown. To run with the
previous USB behavior and no Spresense output:

```sh
python3 multipath_boat.py --no-spresense
```

## Verification

The terminal and event log distinguish three sender states:

- `Connected to ...: 921600 baud`: the host opened the selected serial port.
- `USB port is open but no GPS GGA sample has arrived`: no CSV is being sent.
  This warning repeats every 30 seconds while waiting. A no-fix GGA is enough
  to start sending; a position fix is not required.
- `Host write OK: ... bytes`: the host serial driver accepted a complete CSV
  row. This appears on the first successful write after connection and every
  30 seconds while sending. It does not acknowledge receipt by the receiver.

To test raw reception indoors without relying on GPS data, stop the boat
controller first, then run this on the Jetson. Replace `/dev/ttyUSB0` with the
confirmed Spresense port if it differs. This sends three synthetic **no-fix**
rows (no coordinates) five seconds apart and does not initialize boat controls:

```sh
python3 - <<'PY'
import time
import serial

with serial.Serial("/dev/ttyUSB0", 921600, bytesize=8, parity="N", stopbits=1,
                   timeout=0, write_timeout=0.5, xonxoff=False, rtscts=False,
                   dsrdtr=False, exclusive=True) as port:
    for _ in range(3):
        row = b"000000.00,0,,,0,\n"
        written = port.write(row)
        print(f"Host accepted {written}/{len(row)} bytes", flush=True)
        if written != len(row):
            raise RuntimeError("Incomplete diagnostic write")
        time.sleep(5)
PY
```

Watch the receiver's raw byte count during this test. Complete host writes with
zero received bytes rule out missing GPS samples as the cause of the test's
silence; verify the selected device, the physical USB connection, and which
serial interface the receiver firmware reads at 921600 baud, 8N1, no flow control.
If test bytes arrive but normal streaming stays silent, use the sender state
messages above to check GPS input and any write/reconnection errors.

Run the offline tests without connecting motors, GPS, or a network:

```sh
python3 -m unittest discover -s tests -v
```

On the Jetson, verify the actual receiver gets exactly six fields per line,
approximately five-second spacing, the source UTC timestamp, and a new row after
unplugging/replugging USB. Confirm ArbaLabs reads the same timestamp format and
treats a repeated timestamp as a repeated observation.
