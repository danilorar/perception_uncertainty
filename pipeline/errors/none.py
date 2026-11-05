"""No error: the controller sees the true positions and no hook is active."""
INJECTS = False


def validate(c):
    if c['range_factors'] is not None:
        raise ValueError('range_factors can only be set with uncertainty "range_factors".')


def describe(c):
    return None
