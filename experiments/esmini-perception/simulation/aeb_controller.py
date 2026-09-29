"""AEB decision and ego motion along the original scenario path.

Author: Zhuo Ma
This module is independent of ctypes and the viewer. Its longitudinal TTC
approximation is intended for these near-parallel rear-end cases, not general
crossing traffic. The target is replayed; only the ego motion responds to AEB.
"""
from bisect import bisect_left, bisect_right
import math
import xml.etree.ElementTree as ET


def integrate(speed, acceleration, dt):
    """Integrate constant acceleration exactly, including a stop within the step.

    Return speed [m/s] and travelled distance [m]; never move backwards after
    braking reaches zero speed. This is kinematics, not a tyre/brake model.
    """
    moving_dt = min(dt, speed / -acceleration) if acceleration < 0 else dt
    return (max(0.0, speed + acceleration * moving_dt),
            speed * moving_dt + 0.5 * acceleration * moving_dt * moving_dt)


def risk_metrics(ego, target):
    """Return bumper gap [m], closing speed [m/s], and lateral-path overlap.

    These are ground-truth evaluation values. The runner masks them before
    passing observations to the controller when the target is not detected.
    """
    # Project bounding-box centres and extents onto the ego axes.
    # TTC remains a longitudinal approximation for this near-parallel case.
    def center(state):
        c, s = math.cos(state.h), math.sin(state.h)
        return (state.x + state.centerOffsetX*c - state.centerOffsetY*s,
                state.y + state.centerOffsetX*s + state.centerOffsetY*c)

    ex, ey = center(ego)
    tx, ty = center(target)
    dx, dy = tx-ex, ty-ey
    longitudinal = dx*math.cos(ego.h) + dy*math.sin(ego.h)
    lateral = -dx*math.sin(ego.h) + dy*math.cos(ego.h)
    angle = target.h - ego.h
    c, s = abs(math.cos(angle)), abs(math.sin(angle))

    target_half_length = (target.length*c + target.width*s)/2
    target_half_width = (target.width*c + target.length*s)/2

    gap = longitudinal - ego.length/2 - target_half_length
    closing = ego.speed - target.speed*math.cos(angle)

    in_path = longitudinal > 0 and abs(lateral) <= ego.width/2 + target_half_width

    return gap, closing, in_path


def aeb_command(detected, in_path, gap, closing, speed, latched, *,
                enabled, ttc_threshold, deceleration):
    """Return acceleration [m/s^2], brake latch and TTC [s].

    After triggering, braking stays latched even if later detections disappear.
    This is a TTC teaching controller with no actuator delay or release logic.
    """
    ttc = math.inf

    if in_path and closing > 1e-3:
        ttc = max(gap, 0.0)/closing

    if enabled and detected and in_path:
        if gap <= 0.0 or ttc <= ttc_threshold:
            latched = True

    acceleration = -deceleration if latched and speed > 0 else 0.0

    return acceleration, latched, ttc


class HostReference:
    """Original ego path: query by time before braking, by distance afterwards."""
    def __init__(self, path):
        """Read the original ego polyline and accumulate travelled distance [m]."""
        root = ET.parse(path).getroot()
        groups = [g for g in root.findall(".//ManeuverGroup")
                  if g.find('./Actors/EntityRef[@entityRef="object_1"]') is not None]

        if len(groups) != 1:
            raise ValueError("Expected one original object_1 maneuver group")

        vertices = groups[0].findall(
            ".//FollowTrajectoryAction/Trajectory/Shape/Polyline/Vertex")
        self.times = [float(v.get("time")) for v in vertices]
        self.poses = [tuple(float(v.find("./Position/WorldPosition").get(k, "0"))
                            for k in ("x", "y", "z", "h")) for v in vertices]

        if len(self.times) < 2:
            raise ValueError("Original host trajectory needs at least two vertices")

        self.distances = [0.0]

        for i in range(1, len(self.times)):
            if self.times[i] <= self.times[i-1]:
                raise ValueError("Trajectory timestamps must increase")

            a, b = self.poses[i-1], self.poses[i]
            length = math.hypot(b[0]-a[0], b[1]-a[1])

            if length <= 0:
                raise ValueError("This reference sampler requires a moving host path")

            self.distances.append(self.distances[-1] + length)

    def interpolate(self, i, weight):
        """Interpolate xyz and take the shortest angular path for heading."""
        a, b = self.poses[i], self.poses[i+1]
        xyz = tuple(a[j] + weight*(b[j]-a[j]) for j in range(3))
        dh = math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))

        return (*xyz, a[3] + weight*dh)

    def at_time(self, t):
        """Return pose, distance [m], speed [m/s] on the unbraked time trace."""
        t = max(round(t, 9), self.times[0])

        if t > self.times[-1]:
            return self.poses[-1], self.distances[-1], 0.0

        # Use the interval just completed at an exact timestamp.
        i = max(0, bisect_left(self.times, t)-1)
        duration = self.times[i+1]-self.times[i]
        weight = (t-self.times[i])/duration
        length = self.distances[i+1]-self.distances[i]

        return (self.interpolate(i, weight),
                self.distances[i] + weight*length, length/duration)

    def at_distance(self, distance):
        """Follow the original spatial path at the distance reached under braking."""
        if distance > self.distances[-1] + 1e-8:
            raise RuntimeError("Host exceeded the original path; extend the path first")

        distance = min(max(distance, 0.0), self.distances[-1])
        i = min(max(bisect_right(self.distances, distance)-1, 0), len(self.poses)-2)
        weight = ((distance-self.distances[i]) /
                  (self.distances[i+1]-self.distances[i]))

        return self.interpolate(i, weight)
