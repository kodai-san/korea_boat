"""Apply the fitted-plant GPS fault and race regressions to combined strategy 08."""

import test_gps_guard_simulation as simulation


class GpsTurnPlannerSimulationTests(simulation.GpsGuardSimulationTests):
    strategy_name = "08_gps_aware_turn_planner"
