"""Position integrity and absolute-power governor for mode 06, without hardware I/O.

The uncertainty allowance is a conservative policy, not a measured accuracy bound.
Only accepted observations reach navigation. The existing modes do not use this
controller, and motor/RC emergency overrides remain in the main loop.
"""

import math


EARTH_RADIUS_M = 6378137.0


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _angle(value):
    return (value + 180.0) % 360.0 - 180.0


def _utc_seconds(value):
    try:
        if not isinstance(value, str) or len(value.split(".")[0]) != 6:
            return None
        hours, minutes, seconds = int(value[:2]), int(value[2:4]), float(value[4:])
        if not (0 <= hours < 24 and 0 <= minutes < 60 and 0 <= seconds < 60):
            return None
        return hours * 3600.0 + minutes * 60.0 + seconds
    except (ValueError, TypeError, OverflowError):
        return None


def _source_delta(new, old):
    return (new - old + 43200.0) % 86400.0 - 43200.0


class GpsAwareController:
    """Gate raw GPS observations before applying straight-line guidance.

    ``enter_auto`` always starts neutral. Qualification counts distinct source
    epochs, never control-loop iterations. After a fault the route is preserved;
    stable observations can automatically restore control, but failed blind
    acquisition cannot produce an endless sequence of forward pulses.
    """

    def __init__(self, navigator):
        self.navigator = navigator
        self.options = navigator.strategy.options
        self.reset()

    @staticmethod
    def _missing_heading(reason):
        return dict(heading_deg=None, source="none", age_sec=None, confident=False,
                    state=reason, speed_mps=None, observed_at_mono=None)

    def reset(self):
        self.state = "HOLD"
        self.heading_info = self._missing_heading("inactive")
        self._active = False
        self._route_started = False
        self._anchor = None
        self._history = []
        self._raw = None
        self._candidate = None
        self._qualified = False
        self._good_since = None
        self._good_fixes = 0
        self._fixed_since = None
        self._fixed_fixes = 0
        self._manual_course = None
        self._manual_forward_since = None
        self._last_forward_course = None
        self._forward_thrust_since = None
        self._launch_start = None
        self._launch_used = False
        self._tracking_since = None
        self._resume_blocked = False
        self._power = 0.0
        self._last_update = None
        self._uncertainty = None
        self._clearance = None
        self._innovation = None
        self._position_age = None
        self._retained_hacc = None
        self.navigator.strategy.reset()

    def _course(self, snap, now):
        age, skew = snap.get("course_age_sec"), snap.get("course_gga_skew_sec")
        heading, speed = snap.get("course_deg"), snap.get("course_speed_mps")
        observed = snap.get("course_time_mono")
        limit = self.options["course_max_age_sec"]
        if (not snap.get("course_valid") or not all(_finite(v) for v in (age, skew, heading, speed, observed))
                or not 0 <= age <= limit or abs(skew) > limit
                or not -0.05 <= now - observed <= limit
                or not 0 <= heading < 360
                or not self.options["min_course_speed_mps"] <= speed <= self.options["max_speed_mps"]):
            return None
        return dict(heading_deg=heading, source="gps_course", age_sec=max(age, now - observed, 0.0),
                    confident=snap.get("fix_quality") == 4, state="tracking", speed_mps=speed,
                    observed_at_mono=observed)

    def observe_manual(self, snap, throttle_norm, now):
        if throttle_norm < 0:
            self._manual_course = None
            self._manual_forward_since = None
            return
        course = self._course(snap, now)
        if self._basic_reason(snap) is not None:
            self._manual_forward_since = None
            self._manual_course = None
        elif course is None:
            self._manual_forward_since = None
            # The AUTO gesture itself holds neutral for two seconds. A fresh
            # low-speed coast must not erase the last validated orientation.
            # Preserve its old timestamp so stationary reports cannot renew it.
            if throttle_norm != 0 or not self._known_low_speed(snap, now):
                self._manual_course = None
        elif throttle_norm > 0:
            if self._manual_forward_since is None:
                self._manual_forward_since = now
            ready = self._manual_forward_since + self.options["forward_acquire_sec"]
            if now >= ready and course["observed_at_mono"] >= ready:
                self._manual_course = (now, course["heading_deg"])
        else:
            self._manual_forward_since = None
            if self._manual_course is not None:
                stamp, previous_heading = self._manual_course
                if abs(_angle(course["heading_deg"] - previous_heading)) >= 90.0:
                    self._manual_course = None
                else:
                    self._manual_course = (stamp, course["heading_deg"])

    def _known_low_speed(self, snap, now):
        speed, age = snap.get("course_speed_mps"), snap.get("course_age_sec")
        observed, skew = snap.get("course_time_mono"), snap.get("course_gga_skew_sec")
        return (all(_finite(v) for v in (speed, age, observed, skew))
                and 0 <= speed < self.options["min_course_speed_mps"]
                and 0 <= age <= self.options["course_max_age_sec"]
                and -0.05 <= now - observed <= self.options["course_max_age_sec"]
                and abs(skew) <= self.options["course_max_age_sec"])

    def _steering_course(self, course, now):
        """A velocity vector only becomes a steering reference after forward thrust."""
        if course is None:
            return None
        if self._forward_thrust_since is not None:
            ready = self._forward_thrust_since + self.options["forward_acquire_sec"]
            if now >= ready and course["observed_at_mono"] >= ready:
                return course
        if self._launch_start is not None:
            return None
        evidence = self._last_forward_course
        if (evidence is not None and 0 <= now - evidence[0] <= self.options["forward_memory_sec"]
                and abs(_angle(course["heading_deg"] - evidence[1])) < 90.0):
            return course
        return None

    def _record_power(self, course, now):
        if self._power <= 0:
            self._forward_thrust_since = None
            return
        if self._forward_thrust_since is None:
            self._forward_thrust_since = now
        ready = self._forward_thrust_since + self.options["forward_acquire_sec"]
        if (course is not None and now >= ready and course["observed_at_mono"] >= ready
                and abs(_angle(course["heading_deg"] - self.navigator.line_guidance.bearing_deg))
                <= self.options["race_max_heading_error_deg"]):
            self._last_forward_course = (now, course["heading_deg"])

    def enter_auto(self, snap, now):
        manual_course = self._manual_course
        self.reset()
        self._manual_course = manual_course
        self._last_forward_course = manual_course
        self._active = True
        return self.update(snap, now)

    def suspend(self):
        self._qualified = False
        self._good_since = self._fixed_since = None
        self._good_fixes = self._fixed_fixes = 0
        self._launch_start = None
        self._forward_thrust_since = None
        self._power = 0.0
        self.heading_info = self._missing_heading("suspended")
        self.navigator.strategy.reset()
        self.state = "HOLD"

    def _xy(self, lat, lon):
        return (EARTH_RADIUS_M * math.radians(lon - self.navigator.ref_lon)
                * math.cos(math.radians(self.navigator.ref_lat)),
                EARTH_RADIUS_M * math.radians(lat - self.navigator.ref_lat))

    def _basic_reason(self, snap):
        if not snap.get("valid") or snap.get("fix_quality") not in (1, 2, 4, 5):
            return "no_position_fix"
        lat, lon, age = snap.get("lat"), snap.get("lon"), snap.get("age_sec")
        if not (_finite(lat) and _finite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            return "invalid_coordinates"
        if not _finite(age) or not 0 <= age <= self.options["position_max_age_sec"]:
            return "position_stale"
        hdop, sats = snap.get("hdop"), snap.get("num_sats")
        if not _finite(hdop) or not 0 < hdop <= self.options["max_hdop"]:
            return "poor_hdop"
        if not _finite(sats) or sats < self.options["min_satellites"]:
            return "too_few_satellites"
        correction_age = snap.get("diff_age_sec")
        if snap.get("fix_quality") in (2, 4, 5) and correction_age is not None and (not _finite(correction_age) or not
                0 <= correction_age <= self.options["max_correction_age_sec"]):
            return "corrections_stale"
        if snap.get("fix_quality") == 1 and self.options["gps_throttle"] <= 0:
            return "standalone_gps"
        return None

    def _motion_residual(self, previous, current):
        dt = _source_delta(current["source"], previous["source"])
        if dt <= 0:
            return None
        dx, dy = current["x"] - previous["x"], current["y"] - previous["y"]
        distance = math.hypot(dx, dy)
        velocities = [obs["velocity"] for obs in (previous, current) if obs["velocity"] is not None]
        if velocities:
            vx = sum(v[0] for v in velocities) / len(velocities)
            vy = sum(v[1] for v in velocities) / len(velocities)
            residual = math.hypot(dx - vx * dt, dy - vy * dt)
        else:
            residual = distance
        allowed = self.options["jump_tolerance_m"] + self.options["innovation_growth_mps"] * dt
        plausible = distance <= self.options["jump_tolerance_m"] + self.options["max_speed_mps"] * dt
        return dt, residual, plausible and residual <= allowed

    def _window_residual(self, history, observation):
        """Check accumulated displacement too; per-fix tolerances must not add up."""
        horizon = min(self.options["prediction_horizon_sec"], self.options["max_source_gap_sec"])
        recent = list(history)
        while len(recent) > 1 and _source_delta(observation["source"], recent[1]["source"]) >= horizon:
            recent.pop(0)
        if not recent:
            return True, 0.0
        points = recent + [observation]
        expected_x = expected_y = 0.0
        for first, second in zip(points, points[1:]):
            dt = _source_delta(second["source"], first["source"])
            velocities = [point["velocity"] for point in (first, second) if point["velocity"] is not None]
            if dt <= 0 or not velocities:
                # No independent velocity vector is available over the window.
                return True, 0.0
            expected_x += sum(v[0] for v in velocities) / len(velocities) * dt
            expected_y += sum(v[1] for v in velocities) / len(velocities) * dt
        dt = _source_delta(observation["source"], recent[0]["source"])
        residual = math.hypot(observation["x"] - recent[0]["x"] - expected_x,
                              observation["y"] - recent[0]["y"] - expected_y)
        return residual <= self.options["jump_tolerance_m"] + self.options["innovation_growth_mps"] * dt, residual

    def _remember_observation(self, observation, history):
        history.append(observation)
        while len(history) > 1 and _source_delta(observation["source"], history[0]["source"]) > self.options["max_source_gap_sec"]:
            history.pop(0)

    def _consider_rebase(self, observation, quality, now):
        """Quarantine a coherent position cluster; never rebase on one outlier.

        Non-FIX observations retain their larger uncertainty allowance and
        lower power cap after requalification. GPS may rebase only if enabled.
        """
        enabled_quality = quality in (2, 4, 5) or (quality == 1 and self.options["gps_throttle"] > 0)
        if not enabled_quality or self._anchor is None:
            self._candidate = None
            return False
        gap = min(self.options["max_source_gap_sec"], max(0.0, now - self._anchor["mono"]))
        vx, vy = self._anchor["velocity"] or (0.0, 0.0)
        shift = math.hypot(observation["x"] - self._anchor["x"] - vx * gap,
                           observation["y"] - self._anchor["y"] - vy * gap)
        if shift > self.options["max_reacquire_shift_m"]:
            self._candidate = None
            return False
        candidate = self._candidate
        residual = self._motion_residual(candidate["last"], observation) if candidate else None
        window_ok = self._window_residual(candidate["history"], observation)[0] if candidate else True
        if (candidate is None or residual is None or not residual[2] or not window_ok
                or residual[0] > self.options["max_source_gap_sec"]):
            self._candidate = dict(last=observation, start=now, fixes=1, history=[observation])
            return False
        candidate["last"] = observation
        candidate["fixes"] += 1
        self._remember_observation(observation, candidate["history"])
        if (now - candidate["start"] >= self.options["qualification_sec"]
                and candidate["fixes"] >= self.options["qualification_fixes"]):
            self._anchor = observation
            self._history = list(candidate["history"])
            self._candidate = None
            return True
        return False

    def _observe(self, snap, course, now):
        """Return (new accepted epoch, fault reason). Repeated epochs cannot refresh age."""
        source = _utc_seconds(snap.get("gps_utc"))
        if source is None:
            return False, "invalid_position_time"
        x, y = self._xy(snap["lat"], snap["lon"])
        observed = now - snap["age_sec"]
        velocity = None
        if course is not None:
            radians = math.radians(course["heading_deg"])
            velocity = (course["speed_mps"] * math.sin(radians), course["speed_mps"] * math.cos(radians))
        observation = dict(source=source, mono=observed, x=x, y=y, lat=snap["lat"], lon=snap["lon"], velocity=velocity)
        self._position_age = snap["age_sec"]
        if self._raw is not None:
            delta = _source_delta(source, self._raw["source"])
            if delta == 0:
                self._position_age = max(self._position_age, now - self._raw["mono"])
                if math.hypot(x - self._raw["x"], y - self._raw["y"]) > 1e-4:
                    return False, "duplicate_position_changed"
                if self._position_age > self.options["position_max_age_sec"]:
                    return False, "position_source_stale"
                if self._anchor is None or source != self._anchor["source"]:
                    return False, "position_quarantined"
                return False, None
            if delta < 0:
                return False, "position_out_of_order"
            arrival_delta = observed - self._raw["mono"]
            clock_bad = abs(delta - arrival_delta) > self.options["source_clock_tolerance_sec"]
            if clock_bad:
                return False, "position_clock_mismatch"
        self._raw = observation
        if self._anchor is None:
            self._anchor = observation
            self._history = [observation]
            return True, None
        result = self._motion_residual(self._anchor, observation)
        if result is None:
            return False, "position_out_of_order"
        delta, self._innovation, plausible = result
        window_ok, window_innovation = self._window_residual(self._history, observation)
        self._innovation = max(self._innovation, window_innovation)
        plausible = plausible and window_ok
        if delta > self.options["max_source_gap_sec"] or not plausible:
            if self._consider_rebase(observation, snap["fix_quality"], now):
                return True, None
            return False, "position_gap" if delta > self.options["max_source_gap_sec"] else "position_jump"
        self._anchor = observation
        self._remember_observation(observation, self._history)
        self._candidate = None
        return True, None

    def _uncertainty_for(self, snap):
        prefix = {4: "fixed", 5: "float", 2: "dgps", 1: "gps"}[snap["fix_quality"]]
        uncertainty = self.options[prefix + "_uncertainty_m"]
        hacc = snap.get("hacc_m")
        if hacc is not None:
            age, skew = snap.get("hacc_age_sec"), snap.get("hacc_gga_skew_sec")
            if not all(_finite(v) for v in (hacc, age, skew)) or hacc < 0 or age < 0:
                return None
            if age <= self.options["accuracy_max_age_sec"] and abs(skew) <= self.options["accuracy_max_skew_sec"]:
                allowance = self.options["accuracy_multiplier"] * hacc
                self._retained_hacc = allowance
                uncertainty = max(uncertainty, allowance)
        if self.options.get("retain_stale_accuracy", True) and self._retained_hacc is not None:
            uncertainty = max(uncertainty, self._retained_hacc)
        return uncertainty + self.options["uncertainty_growth_mps"] * self._position_age

    def _diagnostics(self, reason, cap):
        return dict(gps_guard_state=self.state, gps_guard_reason=reason,
                    gps_guard_uncertainty_m=self._uncertainty, gps_guard_clearance_m=self._clearance,
                    gps_guard_innovation_m=self._innovation, gps_guard_position_age_sec=self._position_age,
                    gps_guard_throttle_cap=cap,
                    gps_guard_accepted_latitude=self._anchor["lat"] if self._anchor else None,
                    gps_guard_accepted_longitude=self._anchor["lon"] if self._anchor else None)

    def _hold(self, reason, invalidate=True):
        if invalidate:
            if self._qualified and not self.options["auto_resume"]:
                self._resume_blocked = True
            self._qualified = False
            self._good_since = self._fixed_since = None
            self._good_fixes = self._fixed_fixes = 0
            self._tracking_since = None
            self._launch_start = None
            self._forward_thrust_since = None
        if self.state != "HOLD":
            self.navigator.strategy.reset()
        self.state = "HOLD"
        self._power = 0.0
        self._forward_thrust_since = None
        self.heading_info = self._missing_heading(reason)
        info = self._neutral_info()
        info.update(self._diagnostics(reason, 0.0))
        return info

    @staticmethod
    def _neutral_info():
        return dict(finished=False, active_lane="line", shifted_to=None, wp_number=0,
                    cte_right_m=None, target_bearing_deg=None, boundary_guard=False,
                    throttle_blend=0.0, steer_norm=0.0, throttle_norm=0.0)

    def _power_for(self, cap, now, previous_update):
        dt = 0.0 if previous_update is None else max(0.0, min(0.1, now - previous_update))
        self._power = min(cap, self._power + self.options["throttle_rise_per_sec"] * dt)
        return self._power

    def update(self, snap, now):
        previous_update, self._last_update = self._last_update, now
        self._uncertainty = self._clearance = self._innovation = self._position_age = None
        if not self._active:
            return self._hold("inactive")
        reason = self._basic_reason(snap)
        if reason is not None:
            self._candidate = None
            return self._hold(reason)
        self._position_age = snap["age_sec"]
        if self._uncertainty_for(snap) is None:
            return self._hold("invalid_accuracy")
        course = self._course(snap, now)
        new, reason = self._observe(snap, course, now)
        if reason is not None:
            return self._hold(reason)
        self._uncertainty = self._uncertainty_for(snap)
        if self._uncertainty is None:
            return self._hold("invalid_accuracy")
        line = self.navigator.line_guidance
        _, d = line.frame(self._anchor["x"], self._anchor["y"])
        future = d
        if course is not None:
            future += line.lateral_rate(course["speed_mps"], course["heading_deg"]) * self.options["prediction_horizon_sec"]
        self._clearance = (self.options["band_half_width_m"] - max(abs(d), abs(future))
                           - self._uncertainty - self.options["hull_margin_m"])
        if self._clearance < self.options["min_clearance_m"]:
            return self._hold("insufficient_clearance")
        steering_course = self._steering_course(course, now)
        if steering_course is None and self.state in ("RACE", "CAUTION"):
            # Loss of course while driving always stops first. Any later bounded
            # reacquisition must earn a new position-qualification interval.
            return self._hold("heading_unavailable")
        if self._resume_blocked:
            return self._hold("manual_rearm_required", invalidate=False)
        if snap["fix_quality"] != 4:
            self._fixed_since, self._fixed_fixes = None, 0
        if new:
            if self._good_since is None:
                self._good_since = now
            self._good_fixes += 1
            if snap["fix_quality"] == 4:
                if self._fixed_since is None:
                    self._fixed_since = now
                self._fixed_fixes += 1
        if not self._qualified:
            if (self._good_since is None or now - self._good_since < self.options["qualification_sec"]
                    or self._good_fixes < self.options["qualification_fixes"]):
                return self._hold("qualifying_position", invalidate=False)
            self._qualified = True
        if not self._route_started:
            self.navigator.enter_auto_mode(self._anchor["lat"], self._anchor["lon"], now_mono=now)
            self._route_started = True
        if steering_course is None:
            evidence = self._last_forward_course
            direction_ok = (self._launch_start is not None or
                            (evidence is not None and 0 <= now - evidence[0] <= self.options["launch_heading_memory_sec"]
                             and abs(_angle(evidence[1] - line.bearing_deg)) <= self.options["race_max_heading_error_deg"]))
            low_speed_known = self._known_low_speed(snap, now)
            if self._launch_start is not None and now - self._launch_start >= self.options["launch_timeout_sec"]:
                return self._hold("acquisition_timeout")
            minimum_clearance = self.options["launch_min_clearance_m" if snap["fix_quality"] == 4
                                             else "degraded_launch_min_clearance_m"]
            quality_cap = self.options[{4: "race_throttle", 5: "float_throttle", 2: "dgps_throttle", 1: "gps_throttle"}[snap["fix_quality"]]]
            if (quality_cap <= 0 or not direction_ok or not (low_speed_known or course is not None)
                    or self._clearance < minimum_clearance):
                return self._hold("heading_unavailable")
            if self._launch_start is None:
                if self._launch_used:
                    return self._hold("acquisition_budget_spent")
                self._launch_used = True
                self._launch_start = now
            info = self._neutral_info()
            self.state = "ACQUIRE"
            self.heading_info = self._missing_heading("acquiring_course")
            cap = min(self.options["launch_throttle"], quality_cap)
            info["throttle_norm"] = self._power_for(cap, now, previous_update)
            self._record_power(course, now)
            info.update(self._diagnostics("acquiring_course", cap))
            return info
        self.heading_info = steering_course
        self._launch_start = None
        if self._tracking_since is None:
            self._tracking_since = now
        elif now - self._tracking_since >= self.options["qualification_sec"]:
            self._launch_used = False
        stable_fixed = (self._fixed_since is not None and now - self._fixed_since >= self.options["qualification_sec"]
                        and self._fixed_fixes >= self.options["qualification_fixes"])
        self.heading_info["confident"] = snap["fix_quality"] == 4 and stable_fixed
        info = self.navigator.update(self._anchor["lat"], self._anchor["lon"], course["heading_deg"],
                                     now_mono=now, heading_confident=self.heading_info["confident"], motion=course)
        if info["finished"]:
            self.state = "HOLD"
            self._power = 0.0
            info.update(steer_norm=0.0, throttle_norm=0.0, throttle_blend=0.0)
            info.update(self._diagnostics("finished", 0.0))
            return info
        heading_error = abs(_angle(info["target_bearing_deg"] - course["heading_deg"]))
        race = (snap["fix_quality"] == 4 and stable_fixed and not info["boundary_guard"]
                and self._clearance >= self.options["race_clearance_m"]
                and abs(info["cte_right_m"]) <= self.options["race_max_cte_m"]
                and heading_error <= self.options["race_max_heading_error_deg"])
        self.state = "RACE" if race else "CAUTION"
        cap = (self.options["race_throttle"] if race else
               self.options[{4: "boundary_throttle", 5: "float_throttle", 2: "dgps_throttle", 1: "gps_throttle"}[snap["fix_quality"]]])
        if info["boundary_guard"]:
            cap = min(cap, self.options["boundary_throttle"])
        info["throttle_norm"] = self._power_for(cap, now, previous_update)
        self._record_power(course, now)
        info.update(self._diagnostics("qualified_fix" if race else "conservative_tracking", cap))
        return info
