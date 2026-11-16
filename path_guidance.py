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
