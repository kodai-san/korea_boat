"""Pure geometry for continuous targets along a waypoint polyline."""

import math


def _finite_number(value, label):
    try:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError
        result = float(value)
        if not math.isfinite(result):
            raise ValueError
        return result
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"{label} must be a finite number") from None


def interpolate_lookahead(points, segment_index, boat_x, boat_y, distance_m):
    """Return ``(endpoint_index, (x, y))`` a distance ahead along the path.

    First project the boat onto the supplied segment, with projection clamped
    to that segment. Then walk ``distance_m`` forward along the polyline and
    interpolate within the segment containing the target. ``endpoint_index``
    identifies that segment's ending waypoint; it need not be the target itself.

    Repeated waypoints consume no distance. A degenerate starting segment
    projects to its coincident point, and an exhausted path returns its final
    waypoint. Distance zero returns the projection. Segment indices outside the
    path are clamped; empty paths, nonfinite coordinates and negative distances
    are rejected explicitly.
    """
    if not points:
        raise ValueError("points must contain at least one waypoint")
    if isinstance(segment_index, bool) or not isinstance(segment_index, int):
        raise ValueError("segment_index must be an integer")
    boat_x = _finite_number(boat_x, "boat_x")
    boat_y = _finite_number(boat_y, "boat_y")
    remaining = _finite_number(distance_m, "distance_m")
    if remaining < 0:
        raise ValueError("distance_m must be nonnegative")
    path = []
    for point in points:
        if len(point) != 2:
            raise ValueError("each waypoint must contain x and y")
        path.append((_finite_number(point[0], "waypoint x"),
                     _finite_number(point[1], "waypoint y")))
    if len(path) == 1:
        return 0, path[0]

    index = max(0, min(segment_index, len(path) - 2))
    ax, ay = path[index]
    bx, by = path[index + 1]
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length > 0:
        tx, ty = dx / length, dy / length
        along = max(0.0, min(length, (boat_x - ax) * tx + (boat_y - ay) * ty))
        start_x, start_y = ax + along * tx, ay + along * ty
    else:
        start_x, start_y = ax, ay

    if remaining == 0:
        return index + 1, (start_x, start_y)
    for end_index in range(index + 1, len(path)):
        end_x, end_y = path[end_index]
        dx, dy = end_x - start_x, end_y - start_y
        length = math.hypot(dx, dy)
        if length > 0:
            if remaining <= length:
                fraction = remaining / length
                return end_index, (start_x + fraction * dx, start_y + fraction * dy)
            remaining -= length
        start_x, start_y = end_x, end_y
    return len(path) - 1, path[-1]


class StraightLineGuidance:
    """A straight race line parallel to the course, drawn from the AUTO entry point.

    Coordinates are local ENU meters. The course frame measures ``s`` along the
    course from its start point and ``d`` to the right of the course centerline.
    The banks are straight and parallel to the centerline, so the safe water is
    the band ``-bank_left_m <= d <= bank_right_m``. The line is drawn at the entry
    offset (or where the first turn ends) but never beyond ``-line_left_m`` or
    ``line_right_m``; outside that the boat merges onto it while moving forward.
    """

    def __init__(self, start_xy, end_xy, bank_left_m, bank_right_m, line_left_m, line_right_m):
        sx = _finite_number(start_xy[0], "start x")
        sy = _finite_number(start_xy[1], "start y")
        ex = _finite_number(end_xy[0], "end x")
        ey = _finite_number(end_xy[1], "end y")
        self.length = math.hypot(ex - sx, ey - sy)
        if self.length <= 0:
            raise ValueError("course start and end must differ")
        self.bank_left_m = _finite_number(bank_left_m, "bank_left_m")
        self.bank_right_m = _finite_number(bank_right_m, "bank_right_m")
        self.line_left_m = _finite_number(line_left_m, "line_left_m")
        self.line_right_m = _finite_number(line_right_m, "line_right_m")
        for line, bank, side in ((self.line_left_m, self.bank_left_m, "left"),
                                 (self.line_right_m, self.bank_right_m, "right")):
            if not 0 <= line < bank:
                raise ValueError(f"line_{side}_m must be nonnegative and inside bank_{side}_m")
        self.start_x, self.start_y = sx, sy
        self.ux, self.uy = (ex - sx) / self.length, (ey - sy) / self.length
        self.bearing_deg = (math.degrees(math.atan2(self.ux, self.uy)) + 360.0) % 360.0
        self.line_offset_m = 0.0
        self.entry_offset_m = 0.0

    def frame(self, x, y):
        """Return ``(s, d)``: distance along the course and offset right of the centerline."""
        px, py = x - self.start_x, y - self.start_y
        return px * self.ux + py * self.uy, px * self.uy - py * self.ux

    def point(self, s, d):
        """Inverse of ``frame``."""
        return (self.start_x + s * self.ux + d * self.uy,
                self.start_y + s * self.uy - d * self.ux)

    def set_line(self, d):
        """Put the line at offset ``d``, clamped to the line limits; return the offset used."""
        self.line_offset_m = max(-self.line_left_m, min(self.line_right_m, d))
        return self.line_offset_m

    def draw_line(self, x, y):
        """Start a new line at the boat, clamped toward the centerline; return its offset."""
        _, d = self.frame(x, y)
        self.entry_offset_m = d
        return self.set_line(d)

    def bank_limit(self, d):
        """The bank limit on the side of ``d`` (distance from the centerline, positive)."""
        return self.bank_right_m if d >= 0 else self.bank_left_m

    def desired_course(self, d, lookahead_m, max_intercept_deg):
        """Line-of-sight course toward the line, intercepting at no more than ``max_intercept_deg``."""
        intercept = math.degrees(math.atan2(d - self.line_offset_m, max(lookahead_m, 1e-3)))
        intercept = max(-max_intercept_deg, min(max_intercept_deg, intercept))
        return (self.bearing_deg - intercept) % 360.0

    def lateral_rate(self, speed_mps, course_deg):
        """Rate of change of ``d`` (m/s, + = moving right) for a velocity over ground."""
        return speed_mps * math.sin(math.radians(course_deg - self.bearing_deg))


def turn_arc(d0, phi0_deg, radius_m, direction, lead_m=0.0, step_deg=2.0):
    """Sideways track of a full-lock turn until the boat points down the course again.

    ``phi0_deg`` is the heading relative to the course (+ = pointing right, toward +d). ``direction``
    is +1 for a right turn (heading increases) and -1 for a left turn. The boat first travels
    ``lead_m`` straight (steering and GPS delay), then follows a circle of ``radius_m``. On a circle,
    d changes by R * (cos(phi0) - cos(phi)) per unit of turn, signed by the turn direction.
    Returns a dict: direction, sweep_deg, arc_m, min_d, max_d, end_d.
    """
    phi0 = (phi0_deg + 180.0) % 360.0 - 180.0
    d_start = d0 + lead_m * math.sin(math.radians(phi0))
    sweep = (-phi0) % 360.0 if direction > 0 else phi0 % 360.0
    steps = max(1, int(math.ceil(sweep / step_deg)))
    cos0 = math.cos(math.radians(phi0))
    track = [d_start + direction * radius_m * (cos0 - math.cos(math.radians(phi0 + direction * sweep * k / steps)))
             for k in range(steps + 1)]
    track.append(d0)
    return {"direction": direction, "sweep_deg": sweep, "arc_m": lead_m + radius_m * math.radians(sweep),
            "min_d": min(track), "max_d": max(track), "end_d": track[-2]}


def plan_turn(d0, phi0_deg, lead_m, radius_left_m, radius_right_m, low_d, high_d, tolerance_m=0.0):
    """Choose the turn direction that keeps the boat inside ``low_d <= d <= high_d`` in the shortest arc.

    A turn that leaves the band by at most ``tolerance_m`` counts as fitting. Returns ``(arc, fits)``. If
    neither direction fits, ``arc`` is the one that leaves the band least, with ``arc["violation_side"]``
    +1 (beyond high_d) or -1 (beyond low_d).
    """
    arcs = [turn_arc(d0, phi0_deg, radius_right_m, +1, lead_m), turn_arc(d0, phi0_deg, radius_left_m, -1, lead_m)]
    for arc in arcs:
        below, above = low_d - arc["min_d"], arc["max_d"] - high_d
        arc["violation_m"] = max(0.0, below, above)
        arc["violation_side"] = (1 if above >= below else -1) if arc["violation_m"] > 0 else 0
    fitting = [arc for arc in arcs if arc["violation_m"] <= tolerance_m]
    if fitting:
        return min(fitting, key=lambda arc: arc["arc_m"]), True
    return min(arcs, key=lambda arc: arc["violation_m"]), False


def square_corners(s_start, limits, leg_length_m):
    """Corners ``(s, d, label)`` of one rectangle per ``(name, left_m, right_m)`` in ``limits``.

    Coordinates are the course frame of StraightLineGuidance. Each rectangle runs forward along the left
    line (-left_m), across to the right line (+right_m), back along it and across to the start, so every
    corner is a right turn. Labels name the limits and the leg that ends at the corner.
    """
    corners = []
    for name, left_m, right_m in limits:
        tag = f"{name} -{left_m:.2f}/+{right_m:.2f} m"
        corners += [
            (s_start, -left_m, f"{tag} to start"),
            (s_start + leg_length_m, -left_m, f"{tag} left line"),
            (s_start + leg_length_m, right_m, f"{tag} far side"),
            (s_start, right_m, f"{tag} right line"),
            (s_start, -left_m, f"{tag} near side"),
        ]
    return corners
