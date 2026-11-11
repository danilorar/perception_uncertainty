import math
from .frames import check_period, markov_draw, markov_parameters

SETTINGS = {
    "base_miss_rate": 0.1,  # base probability of missing a detection, before range and angle effects
    "range_start_m": 20.0,  # range at which the miss probability starts to increase
    "angle_start_deg": 5.0,  # angle at which the miss probability starts to increase
    "recovery_range_decay_per_m": 0.0,
    "recovery_angle_decay_per_deg": 0.0,
    "range_slope_per_m": 0.0025,  # rate of increase in miss probability with range
    "angle_slope_per_deg": 0.005,  # rate of increase in miss probability with angle
    "miss_mean_s": 0.5,  # mean duration of a miss
    "sensor_period_s": 0.05,  # sensor period
}

RANGES = {
    "base_miss_rate": (0.0, 0.5),
    "range_start_m": (0.0, math.inf),
    "angle_start_deg": (0.0, 180.0),
    "recovery_range_decay_per_m": (0.0, math.inf),
    "recovery_angle_decay_per_deg": (0.0, math.inf),
    "range_slope_per_m": (0.0, math.inf),
    "angle_slope_per_deg": (0.0, math.inf),
    "miss_mean_s": (0.01, 20.0),
    "sensor_period_s": (0.01, 1.0),
}

LOG = ("frame", "sample_range_m", "sample_bearing_deg", "local_miss_rate", "previous_detected", "p_detect", "missed")


def check(settings, run):
    check_period(settings["sensor_period_s"], settings["miss_mean_s"], run.dt_s)


class Error:
    def __init__(self, settings, run):
        self.s = settings
        self.seed = run.seed
        self.steps_per_frame = round(settings["sensor_period_s"] / run.dt_s)
        self.cache = {}

    def perceive(self, step, t, ego, targets):
        """every control step, update the detection state of each target.
        The state is cached per frame, so that all steps in a frame have the same state."""
        frame = step // self.steps_per_frame
        s = self.s

        for target in targets:
            key = target["id"]
            record = self.cache.get(key)
            if record is None or record[0] != frame:
                true = target["true"]
                r = target["true_range"]
                phi = math.atan2(true["y"] - ego["y"], true["x"] - ego["x"]) - ego["heading"]
                angle = math.degrees(math.atan2(math.sin(phi), math.cos(phi)))
                # ------ p miss ------
                range_excess = max(0.0, r - s["range_start_m"])
                angle_excess = max(0.0, abs(angle) - s["angle_start_deg"])
                p = min(0.5,
                        s["base_miss_rate"]
                        + range_excess * s["range_slope_per_m"]
                        + angle_excess * s["angle_slope_per_deg"])
                enter, recover = markov_parameters(p, s["miss_mean_s"], s["sensor_period_s"])
                if p > 0.0:
                    penalty = (range_excess * s["recovery_range_decay_per_m"]
                               + angle_excess * s["recovery_angle_decay_per_deg"])
                    recovery_factor = math.exp(-penalty)
                    enter *= recovery_factor
                    recover *= recovery_factor
                initial = record is None or record[0] != frame - 1
                u = markov_draw(self.seed, frame, key, initial)
                if initial:
                    previous_detected, p_detect, missed = "", 1.0 - p, u < p
                elif record[-1]:
                    previous_detected, p_detect, missed = 0, recover, not (u < recover)
                else:
                    previous_detected, p_detect, missed = 1, 1.0 - enter, u < enter

                record = (frame, r, angle, p, previous_detected, p_detect, int(missed))
                self.cache[key] = record

            target["log"].update(zip(LOG, record))
            if record[-1]:
                target["seen"] = False

    def describe(self):
        return {
            "model": "Range/bearing-conditioned Markov dropout", **self.s
        }
