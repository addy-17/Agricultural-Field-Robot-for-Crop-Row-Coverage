"""
lane_navigator.py - Pose-based lane coverage (replaces sensor-triggered FSM)
Drives lanes y = 0, +2.2, -2.2 (between the 4 crop rows) back and forth, any number of laps.
Uses GPS (x,y) + IMU yaw (closed loop) -> no timed/open-loop manoeuvres, so it cannot drift or loop.
Pure Python: testable without Webots.
"""
import math

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

class LaneNavigator:
    LANES_Y = [0.0, 2.2, -2.2]      # lane centrelines (world y), rows are at y=+-1.1, +-3.3
    X_EXIT = 9.0                    # lane finished when robot passes this |x| (rows end at 8)
    X_TURN = 10.2                   # headland turning line
    X_REENTRY = 9.3                 # point from which the next lane starts

    V_CRUISE = 0.50
    V_TURN = 0.38
    W_MAX = 0.70
    LOOKAHEAD = 0.80
    DETOUR_OFFSET = 0.45            # sideways shift around obstacle (lane half-width is 0.9)
    OBS_TRIGGER = 1.40              # front distance that starts a detour

    def __init__(self, laps=2):
        self.total_lanes = 3 * laps
        self.lane_i = 0
        self.direction = +1         # +1 heading +x, -1 heading -x
        self.state = "TRACK"
        self.offset = 0.0
        self.obs_x = None
        self.wps = []
        self.lanes_done = 0

    # ---- helpers
    def lane_y(self):
        return self.LANES_Y[self.lane_i % 3]

    def _goto(self, x, y, yaw, wp, v):
        bearing = math.atan2(wp[1] - y, wp[0] - x)
        err = wrap(bearing - yaw)
        if abs(err) > 0.35:                                   # pivot in place (skid steer)
            w = math.copysign(max(0.30, min(self.W_MAX, 1.6 * abs(err))), err)
            return 0.0, w
        return v, max(-self.W_MAX, min(self.W_MAX, 1.8 * err))

    # ---- main step: returns (v, omega, state_name)
    def update(self, x, y, yaw, front_dist):
        if self.state == "DONE":
            return 0.0, 0.0, "MISSION_COMPLETE"

        if self.state == "TRACK":
            d = self.direction
            href = 0.0 if d > 0 else math.pi
            # finished this lane?
            if d * x >= self.X_EXIT:
                self.lanes_done += 1
                if self.lanes_done >= self.total_lanes:
                    self.state = "DONE"
                    return 0.0, 0.0, "MISSION_COMPLETE"
                self.lane_i += 1
                y2 = self.lane_y()
                self.wps = [(d * self.X_TURN, y), (d * self.X_TURN, y2), (d * self.X_REENTRY, y2)]
                self.state = "HEADLAND_TURN"
                self.offset = 0.0
                self.obs_x = None
                return 0.0, 0.0, "HEADLAND_TRANSITION"

            aligned = abs(wrap(yaw - href)) < 0.35
            in_rows = abs(x) < 7.5
            # obstacle detection (only when aligned, centred and inside the rows)
            if (self.obs_x is None and in_rows and aligned and front_dist < self.OBS_TRIGGER
                    and abs(y - self.lane_y()) < 0.30):
                self.obs_x = x + d * front_dist
                self.offset = d * self.DETOUR_OFFSET      # shift to the robot's left
            if self.obs_x is not None and d * (x - self.obs_x) > 0.9:
                self.offset = 0.0                         # passed it -> back to centre
                if d * (x - self.obs_x) > 2.0:
                    self.obs_x = None                     # re-arm detector

            target_y = self.lane_y() + self.offset
            e_left = d * (y - target_y)
            psi_des = href - max(-0.6, min(0.6, math.atan2(e_left, self.LOOKAHEAD)))
            err = wrap(psi_des - yaw)
            if abs(err) > 0.9:                            # badly misaligned: pivot first
                return 0.0, math.copysign(0.6, err), "ROW_ENTRY"
            w = max(-self.W_MAX, min(self.W_MAX, 1.8 * err))
            v = self.V_CRUISE * max(0.35, 1.0 - abs(err) / 0.8)
            name = "OBSTACLE_AVOIDANCE" if self.obs_x is not None else "IN_ROW_TRACKING"
            return v, w, name

        if self.state == "HEADLAND_TURN":
            wp = self.wps[0]
            tol = 0.12 if len(self.wps) == 1 else 0.20
            if math.hypot(wp[0] - x, wp[1] - y) < tol:
                self.wps.pop(0)
                if not self.wps:
                    self.direction *= -1
                    self.state = "TRACK"
                    return 0.0, 0.0, "ROW_ENTRY"
                wp = self.wps[0]
            v, w = self._goto(x, y, yaw, wp, self.V_TURN)
            return v, w, "HEADLAND_TURN"
        return 0.0, 0.0, "MISSION_COMPLETE"
