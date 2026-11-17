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
    the band ``|d| <= band_half_width_m``. The line keeps the entry offset unless
    that is beyond ``line_max_offset_m``, in which case it is drawn that far from
    the centerline and the boat merges onto it while moving forward.
    """

    def __init__(self, start_xy, end_xy, band_half_width_m, line_max_offset_m):
        sx = _finite_number(start_xy[0], "start x")
        sy = _finite_number(start_xy[1], "start y")
        ex = _finite_number(end_xy[0], "end x")
        ey = _finite_number(end_xy[1], "end y")
        self.length = math.hypot(ex - sx, ey - sy)
        if self.length <= 0:
            raise ValueError("course start and end must differ")
        self.band_half_width_m = _finite_number(band_half_width_m, "band_half_width_m")
        self.line_max_offset_m = _finite_number(line_max_offset_m, "line_max_offset_m")
        if not 0 <= self.line_max_offset_m < self.band_half_width_m:
            raise ValueError("line_max_offset_m must be nonnegative and inside the band")
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

    def draw_line(self, x, y):
        """Start a new line at the boat, clamped toward the centerline; return its offset."""
        _, d = self.frame(x, y)
        self.entry_offset_m = d
        self.line_offset_m = max(-self.line_max_offset_m, min(self.line_max_offset_m, d))
        return self.line_offset_m

    def desired_course(self, d, lookahead_m, max_intercept_deg):
        """Line-of-sight course toward the line, intercepting at no more than ``max_intercept_deg``."""
        intercept = math.degrees(math.atan2(d - self.line_offset_m, max(lookahead_m, 1e-3)))
        intercept = max(-max_intercept_deg, min(max_intercept_deg, intercept))
        return (self.bearing_deg - intercept) % 360.0

    def lateral_rate(self, speed_mps, course_deg):
        """Rate of change of ``d`` (m/s, + = moving right) for a velocity over ground."""
        return speed_mps * math.sin(math.radians(course_deg - self.bearing_deg))


def square_corners(s_start, half_widths, leg_length_m):
    """Corners ``(s, d, label)`` of one rectangle per half width, in the course frame of StraightLineGuidance.

    Each rectangle runs forward along the left line (-h), across to the right line (+h), back along it and
    across to the start, so every corner is a right turn (the hull's tighter direction). Labels name the leg
    that ends at the corner.
    """
    corners = []
    for h in half_widths:
        tag = f"+-{h:.2f} m"
        corners += [
            (s_start, -h, f"{tag} to start"),
            (s_start + leg_length_m, -h, f"{tag} left line"),
            (s_start + leg_length_m, h, f"{tag} far side"),
            (s_start, h, f"{tag} right line"),
            (s_start, -h, f"{tag} near side"),
        ]
    return corners
