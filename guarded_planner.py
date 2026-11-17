"""Mode 08: 06's GPS guard in front of 07's turn planner, without hardware I/O.

All of GpsAwareController's position checks are reused unchanged: freshness, source-time order, jump and
window residuals, qualification, uncertainty, quality caps, bounded launch and automatic resumption. Only the
decisions that would stop 07's planner from working change:

- Clearance is measured to each side's bank limit (07's per-side limits) instead of a symmetric band.
- Near a bank limit on qualified RTK FIX the planner, bank guard and hard limit may steer away (power capped
  at boundary_throttle) instead of holding neutral; on any weaker fix the guard still holds neutral.
- A planned full-lock turn runs at race power on qualified FIX (the measured radii are for full throttle) and
  at the quality cap otherwise, instead of the 35% heading-error cap.
- The planner may reverse only on qualified FIX; otherwise its reverse becomes neutral, as in 06 where an
  uncertain GPS cannot authorize reverse propulsion.
- During a planned turn or reverse the planner continues from its remembered bow heading when no forward
  course is available, instead of the guard holding neutral; after a reverse the guard requires fresh forward
  thrust before a measured course steers again.
"""

from gps_guard import GpsAwareController, _angle


class GuardedPlannerController(GpsAwareController):

    def _hold(self, reason, invalidate=True):
        if invalidate:
            # A fault ends any planned manoeuvre; plan again from fresh observations after resuming.
            nav = self.navigator
            if getattr(nav, "line_guidance", None) is not None and hasattr(nav, "turn_dir"):
                nav.turn_dir = 0
                nav.rev_phase = None
                nav.plan_pending = True
        return super()._hold(reason, invalidate)

    def _maneuvering(self):
        nav = self.navigator
        return getattr(nav, "rev_phase", None) is not None or getattr(nav, "turn_dir", 0) != 0

    def _stable_fixed(self, snap, now):
        return (snap["fix_quality"] == 4 and self._fixed_since is not None
                and now - self._fixed_since >= self.options["qualification_sec"]
                and self._fixed_fixes >= self.options["qualification_fixes"])

    def _clearance_to_banks(self, d, future):
        line = self.navigator.line_guidance
        room = min(line.bank_limit(d) - abs(d), line.bank_limit(future) - abs(future))
        return room - self._uncertainty - self.options["hull_margin_m"]

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
        self._clearance = self._clearance_to_banks(d, future)

        # Qualification counting as in 06 (moved before the clearance decision, which needs it).
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
        stable_fixed = self._stable_fixed(snap, now)
        if (not self._qualified and self._good_since is not None
                and now - self._good_since >= self.options["qualification_sec"]
                and self._good_fixes >= self.options["qualification_fixes"]):
            self._qualified = True
        if self._clearance < self.options["min_clearance_m"] and not (self._qualified and stable_fixed):
            # 06 holds and restarts qualification here. On RTK FIX keep counting instead, so a start near the
            # bank can qualify and then let the planner steer or reverse away; weaker fixes behave as in 06.
            return self._hold("insufficient_clearance", invalidate=snap["fix_quality"] != 4)

        maneuvering = self._maneuvering()
        steering_course = self._steering_course(course, now)
        if steering_course is None and self.state in ("RACE", "CAUTION", "TURN") and not maneuvering:
            return self._hold("heading_unavailable")
        if self._resume_blocked:
            return self._hold("manual_rearm_required", invalidate=False)
        if not self._qualified:
            return self._hold("qualifying_position", invalidate=False)
        if not self._route_started:
            self.navigator.enter_auto_mode(self._anchor["lat"], self._anchor["lon"], now_mono=now)
            self._route_started = True
        if steering_course is None and not maneuvering:
            return self._launch(snap, course, now, previous_update, line)

        if steering_course is not None:
            self.heading_info = steering_course
            self._launch_start = None
            if self._tracking_since is None:
                self._tracking_since = now
            elif now - self._tracking_since >= self.options["qualification_sec"]:
                self._launch_used = False
            self.heading_info["confident"] = snap["fix_quality"] == 4 and stable_fixed
            heading, motion = steering_course["heading_deg"], steering_course
        else:
            self.heading_info = self._missing_heading("planned_maneuver")
            heading, motion = None, None
        info = self.navigator.update(self._anchor["lat"], self._anchor["lon"], heading, now_mono=now,
                                     heading_confident=self.heading_info["confident"], motion=motion)
        if info["finished"]:
            self.state = "HOLD"
            self._power = 0.0
            info.update(steer_norm=0.0, throttle_norm=0.0, throttle_blend=0.0)
            info.update(self._diagnostics("finished", 0.0))
            return info
        if info.get("reacquire_heading"):
            # The course measured while reversing points backward; steer again only after forward thrust.
            self._forward_thrust_since = None
            self._last_forward_course = None

        status = str(info.get("prediction_status") or "")
        quality_cap = self.options[{4: "race_throttle", 5: "float_throttle", 2: "dgps_throttle",
                                    1: "gps_throttle"}[snap["fix_quality"]]]
        if status.startswith("plan_"):
            # Planner neutral / reverse. Reverse only on qualified FIX.
            requested = info.get("throttle_norm") or 0.0
            allowed = requested if (requested >= 0 or stable_fixed) else 0.0
            self.state = "MANEUVER"
            self._power = 0.0
            self._forward_thrust_since = None
            info["throttle_norm"] = allowed
            info.update(self._diagnostics("planner_reverse" if allowed < 0 else
                                          "planner_neutral" if requested >= 0 else "reverse_needs_fix", 0.0))
            return info
        if status.startswith("turn_"):
            # Full-lock turn: the measured radii are for full throttle, so no heading-error cap.
            self.state = "TURN"
            cap = quality_cap if stable_fixed or snap["fix_quality"] != 4 else self.options["boundary_throttle"]
            info["throttle_norm"] = self._power_for(cap, now, previous_update)
            self._record_power(steering_course, now)
            info.update(self._diagnostics("planned_turn", cap))
            return info

        heading_error = (abs(_angle(info["target_bearing_deg"] - heading))
                         if heading is not None and info.get("target_bearing_deg") is not None else 180.0)
        race = (snap["fix_quality"] == 4 and stable_fixed and not info["boundary_guard"]
                and self._clearance >= self.options["race_clearance_m"]
                and abs(info["cte_right_m"]) <= self.options["race_max_cte_m"]
                and heading_error <= self.options["race_max_heading_error_deg"])
        self.state = "RACE" if race else "CAUTION"
        cap = (self.options["race_throttle"] if race else
               self.options[{4: "boundary_throttle", 5: "float_throttle", 2: "dgps_throttle",
                             1: "gps_throttle"}[snap["fix_quality"]]])
        if info["boundary_guard"] or self._clearance < self.options["min_clearance_m"]:
            cap = min(cap, self.options["boundary_throttle"])
        info["throttle_norm"] = self._power_for(cap, now, previous_update)
        self._record_power(steering_course, now)
        info.update(self._diagnostics("qualified_fix" if race else "conservative_tracking", cap))
        return info

    def _launch(self, snap, course, now, previous_update, line):
        """06's bounded launch without a steering course, unchanged."""
        evidence = self._last_forward_course
        direction_ok = (self._launch_start is not None or
                        (evidence is not None and 0 <= now - evidence[0] <= self.options["launch_heading_memory_sec"]
                         and abs(_angle(evidence[1] - line.bearing_deg)) <= self.options["race_max_heading_error_deg"]))
        low_speed_known = self._known_low_speed(snap, now)
        if self._launch_start is not None and now - self._launch_start >= self.options["launch_timeout_sec"]:
            return self._hold("acquisition_timeout")
        minimum_clearance = self.options["launch_min_clearance_m" if snap["fix_quality"] == 4
                                         else "degraded_launch_min_clearance_m"]
        quality_cap = self.options[{4: "race_throttle", 5: "float_throttle", 2: "dgps_throttle",
                                    1: "gps_throttle"}[snap["fix_quality"]]]
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
