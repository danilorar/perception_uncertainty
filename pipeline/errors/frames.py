"""Frame sequences for the detection and identification plugins (a helper, not a plugin).

Port of Zhuo Ma's DetectionDropout and MarkovDropout (branch feature/esmini-perception,
commit 484675b, experiments/esmini-perception/simulation/acc_cpp/src/perception.cpp):
same SplitMix64 draws, so a seed and target ID give the same lost frames as that
experiment. A frame's state holds for every integration step inside the frame.
"""
MASK = (1 << 64) - 1
TO_UNIT = 1.0 / 9007199254740992.0
ID_OFFSET = 0x100000000
INITIAL_DOMAIN = 0x243f6a8885a308d3
TRANSITION_DOMAIN = 0x13198a2e03707344
# Identification uses the same chain on a stream that Detection never reads.
IDENTIFICATION_ID_OFFSET = 1000


def mix(x):
    x = (x + 0x9e3779b97f4a7c15) & MASK
    x = ((x ^ (x >> 30)) * 0xbf58476d1ce4e5b9) & MASK
    x = ((x ^ (x >> 27)) * 0x94d049bb133111eb) & MASK
    return x ^ (x >> 31)


def markov_draw(seed, sequence, ident, initial):
    domain = INITIAL_DOMAIN if initial else TRANSITION_DOMAIN
    key = mix(seed) ^ mix(sequence) ^ mix(ident + ID_OFFSET) ^ mix(domain)
    return (mix(key) >> 11) * TO_UNIT


def markov_parameters(probability, mean_s, period_s):
    """Two-state chain with stationary share `probability` and mean run `mean_s`."""
    if not 0 <= probability <= 1 or period_s <= 0 or mean_s < period_s:
        raise ValueError('Markov requires p in [0,1] and mean duration >= sensor period > 0')
    if probability == 0:
        return 0.0, 1.0
    if probability == 1:
        return 1.0, 0.0
    recover = period_s / mean_s
    enter = probability * recover / (1 - probability)
    if enter > 1:
        raise ValueError('Requested rate and duration give an entry probability above 1')
    return enter, recover


def markov_frames(probability, mean_s, period_s, seed, ident, frames):
    enter, recover = markov_parameters(probability, mean_s, period_s)
    state = probability == 1 or markov_draw(seed, 0, ident, True) < probability
    states = [state]
    for sequence in range(1, frames):
        draw = markov_draw(seed, sequence, ident, False)
        state = not draw < recover if state else draw < enter
        states.append(state)
    return states


def iid_frames(probability, seed, ident, frames):
    states = []
    for sequence in range(frames):
        key = mix(seed) ^ mix(sequence) ^ mix(ident + ID_OFFSET)
        states.append((mix(key) >> 11) * TO_UNIT < probability)
    return states


def hold_per_step(states, period_s, dt_s, steps):
    """One flag per integration step; the frame state holds until the next frame."""
    per_frame = round(period_s / dt_s)
    return [states[step // per_frame] for step in range(steps)]


def missed_steps(factors, seed, ident, dt_s, steps):
    period = factors['sensor_period_s']
    frames = steps // round(period / dt_s) + 1
    if factors['miss_model'] == 'iid':
        states = iid_frames(factors['miss_rate'], seed, ident, frames)
    else:
        states = markov_frames(factors['miss_rate'], factors['miss_mean_s'], period, seed, ident, frames)
    return hold_per_step(states, period, dt_s, steps)


def wrong_class_steps(factors, seed, ident, dt_s, steps):
    period = factors['sensor_period_s']
    frames = steps // round(period / dt_s) + 1
    states = markov_frames(factors['wrong_rate'], factors['wrong_mean_s'], period, seed,
                           ident + IDENTIFICATION_ID_OFFSET, frames)
    return hold_per_step(states, period, dt_s, steps)


def check_period(period_s, mean_s, dt_s):
    if abs(period_s / dt_s - round(period_s / dt_s)) > 1e-9:
        raise ValueError('sensor_period_s must be a whole number of time steps')
    if mean_s < period_s:
        raise ValueError('a mean duration cannot be shorter than sensor_period_s')
