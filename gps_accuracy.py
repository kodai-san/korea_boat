"""Optional NMEA horizontal-accuracy observations; no receiver configuration or I/O.

The caller verifies and strips the NMEA checksum before splitting a sentence.
PUBX,00 supplies the receiver's horizontal accuracy estimate in metres. GST
supplies latitude/longitude standard deviations; their hypotenuse is a
conservative combined horizontal standard-deviation estimate, not a guaranteed
position-error bound. Neither message is assumed to be enabled on the receiver.
"""

import math
import re


_UTC_PATTERN = re.compile(r"\d{6}(?:\.\d+)?\Z", re.ASCII)


def nmea_utc_seconds(value):
    """Parse a UTC hhmmss[.fraction] field without accepting malformed times."""
    if not isinstance(value, str) or not _UTC_PATTERN.fullmatch(value):
        return None
    hours, minutes, seconds = int(value[:2]), int(value[2:4]), float(value[4:])
    if hours >= 24 or minutes >= 60 or not 0 <= seconds < 60:
        return None
    return hours * 3600.0 + minutes * 60.0 + seconds


def utc_skew_seconds(observation_utc, gga_utc):
    """Signed nearest-day difference between source times, including midnight."""
    observation = nmea_utc_seconds(observation_utc)
    gga = nmea_utc_seconds(gga_utc)
    if observation is None or gga is None:
        return None
    return (observation - gga + 43200.0) % 86400.0 - 43200.0


def _nonnegative_number(value):
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) and number >= 0 else None


def parse_accuracy_message(fields):
    """Return ``{utc, hacc_m, source}`` for valid PUBX,00/GST, otherwise None.

    ``fields`` is a sequence such as ``['$PUBX', '00', ...]`` or
    ``['$GNGST', ...]``. No malformed sentence can create a zero uncertainty.
    """
    if not isinstance(fields, (list, tuple)) or not fields:
        return None
    message = fields[0]
    if message == "$PUBX":
        if len(fields) < 10 or fields[1] != "00":
            return None
        if not fields[8] or fields[8] in ("NF", "DR", "TT"):
            return None
        utc, accuracy, source = fields[2], _nonnegative_number(fields[9]), "pubx"
    elif isinstance(message, str) and re.fullmatch(r"\$[A-Z]{2}GST", message):
        if len(fields) < 8:
            return None
        latitude = _nonnegative_number(fields[6])
        longitude = _nonnegative_number(fields[7])
        if latitude is None or longitude is None:
            return None
        utc, accuracy, source = fields[1], math.hypot(latitude, longitude), "gst"
    else:
        return None
    if nmea_utc_seconds(utc) is None or accuracy is None or not math.isfinite(accuracy):
        return None
    return {"utc": utc, "hacc_m": accuracy, "source": source}


def usable_accuracy(accuracy, gga_utc, now, max_age, max_skew):
    """Return a fresh, epoch-matched accuracy estimate, or None.

    The receiver adds a monotonic ``received_at`` value to the parser result.
    A replayed, stale, or unmatched source epoch must not imply fresh accuracy.
    """
    if not isinstance(accuracy, dict):
        return None
    value = _nonnegative_number(accuracy.get("hacc_m"))
    limits = [_nonnegative_number(item) for item in (now, accuracy.get("received_at"), max_age, max_skew)]
    if value is None or any(item is None for item in limits):
        return None
    now, received_at, max_age, max_skew = limits
    age = now - received_at
    skew = utc_skew_seconds(accuracy.get("utc"), gga_utc)
    if not 0 <= age <= max_age or skew is None or abs(skew) > max_skew:
        return None
    return value
