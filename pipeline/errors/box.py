"""Box: a wrong reported length or width of the target, and which point of its box stays right.

    length = length_scale * true length,   width = width_scale * true width

The controller gets a box, so a wrong length moves its ends. keep says which point is still measured right:
    centre     the centre stays: both ends move by half the length error, so the gap to the near end is
               wrong by that half
    near_end   the end facing the ego stays: the centre moves along the target's heading by half the length
               error, so the gap to the near end is unchanged

With near_end the range changes by the part of that shift that lies on the line of sight, which is all of it
when the target is straight ahead or behind. This is the link from a size error to the distance that needs no
sensor type; the scale law in size.py is the camera-type link. The scales are assumed, not measured.
"""
import math

SETTINGS = {'length_scale': 1.0, 'width_scale': 1.0, 'keep': 'centre'}
RANGES = {'length_scale': (0.1, 10), 'width_scale': (0.1, 10)}
CHOICES = {'keep': ('centre', 'near_end')}
LOG = ('length_m', 'range_shift_m')


class Error:
    def __init__(self, settings, run):
        self.settings = settings
        self.length, self.width = settings['length_scale'], settings['width_scale']
        self.near_end = settings['keep'] == 'near_end'

    def perceive(self, step, t, ego, targets):
        for target in targets:
            true = target['true']
            shift = 0.0
            if self.length != 1.0:
                target['length'] = self.length * true['length']
                if self.near_end:
                    # Line of sight ego -> target, and the target's heading, both in the ground plane.
                    sight_x, sight_y = true['x'] - ego['x'], true['y'] - ego['y']
                    distance = math.hypot(sight_x, sight_y)
                    if distance > 1e-12:
                        along = (math.cos(true['heading']) * sight_x + math.sin(true['heading']) * sight_y) / distance
                        # A longer box pushes the centre away from the ego, whichever end faces it.
                        shift = (target['length'] - true['length']) / 2.0 * abs(along)
                        target['range'] = target['range'] + shift
            if self.width != 1.0:
                target['width'] = self.width * true['width']
            target['log'].update(length_m=target['length'], range_shift_m=shift)

    def describe(self):
        return {'box': 'length and width scaled; keep = the point of the box that stays right', **self.settings}
