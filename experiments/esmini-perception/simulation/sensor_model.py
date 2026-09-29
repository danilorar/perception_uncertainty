"""Reproducible detection dropout applied after esmini's ideal object sensor.

Author: Zhuo Ma
The model removes complete detections, not individual position/category fields.
It does not model sensor physics, fusion, occlusion, tracking or false positives.
"""
import hashlib
import math


class DetectionDropout:
    """Remove each detected object with probability p at each simulation tick.

    At dt=0.01 s, p=0.1 is a 10% chance per object per 10 ms update. It is not
    a guaranteed 10% missing fraction in one run or a fixed 0.1 s blackout.
    """

    def __init__(self, probability=0.1, seed=42, dt=0.01):
        """Validate the probability and simulation timestep (seconds)."""
        if not math.isfinite(probability) or not 0 <= probability <= 1:
            raise ValueError("probability must be within [0, 1]")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        self.probability = probability
        self.seed = int(seed)
        self.dt = dt

    def apply(self, raw_ids, time_s, sensor_id):
        """Return (kept_ids, dropped_ids), without modifying the input list.

        The hash substitutes for a stateful random-number generator: the same
        seed/sensor/tick/object always gets the same draw, even if AEB changes
        visibility on other ticks. Matched runs can still have different total
        detection counts because their vehicle trajectories diverge.
        """
        tick = int(round(time_s / self.dt))
        kept, dropped = [], []
        for object_id in sorted(set(raw_ids)):
            # The same sensor/object/tick gets the same draw in OFF and ON runs.
            key = f"{self.seed}:{sensor_id}:{tick}:{object_id}".encode("ascii")
            digest = hashlib.blake2b(key, digest_size=8).digest()
            # Convert 53 bits to a deterministic uniform value in [0, 1).
            uniform = (int.from_bytes(digest, "big") >> 11) / (1 << 53)
            if uniform < self.probability:
                dropped.append(object_id)
            else:
                kept.append(object_id)
        return kept, dropped
