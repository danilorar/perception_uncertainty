# Distance and azimuth dependent physical-size error

`size_bearing` changes the physical length and width **perceived by the controller**.
It uses the existing Python plugin and C++ SIZE bridge interface; adding or changing
this plugin needs no engine rebuild. The bridge restores true dimensions immediately
after the controller step. Height is not currently an output of that interface.

The existing `size` plugin retains its meaning: a size assumption represented as
range scale/bias. `size_bearing` supplies dimension errors directly.

## Model

Distance r is planar sensor-origin to true target-box centre distance, in metres.
Azimuth beta is the wrapped angle from sensor forward to the target, in radians
internally and degrees in logs/configuration. It is NOT target heading or aspect angle.
The sensor starts at the ego-box centre, with optional `sensor_forward_m`,
`sensor_left_m` and `sensor_yaw_deg` mounting offsets. Positive lateral offset and
positive azimuth are left. No image renderer or sensor estimator runs here.

Define

```
u = log(1 + r/reference_distance_m)
v = (sin(beta)/sin(reference_bearing_deg))**2
phi = [1, u, v, u*v]
mu_k = dot(k_mu, phi)
sigma_k = k_sigma_ref * exp(dot(k_sigma_log, phi))
e_k = clip(mu_k + sigma_k*z_k, -max_abs_log_error, max_abs_log_error)
perceived_k = incoming_k * exp(e_k)       k = length or width
```

Used alone, incoming dimensions are the truth. Multiple dimension-changing plugins
compose multiplicatively in chain order. The features assume equal behaviour on the
left and right; add an odd angular term only when data support asymmetry.

`mu` and `sigma` are the mean and standard deviation of the UNCLIPPED log-ratio,
not metres or percentages. Before clipping, median output/input is exp(mu), mean is
exp(mu+sigma^2/2), and coefficient of variation is sqrt(exp(sigma^2)-1). Consequently
mu=0 with nonzero sigma is median-unbiased, not mean-unbiased. Clipping changes these
moments, so clipped samples must be included when validating the actual model.

The sigma reference is an amplitude, not automatically the standard deviation at
`reference_distance_m`: with a nonzero distance coefficient, u=log(2) there. For
identifiable fitting, either fix sigma_ref (e.g. to 1) and fit all four sigma-log
coefficients, or fix their constant term to 0 and fit sigma_ref plus the remaining
three. Do not fit the sigma reference and log-intercept freely at the same time.

Noise z is stationary unit Gaussian OU with rho=exp(-dt/correlation_time_s).
Each plugin label and target ID has independent seeded streams. Optional
`length_width_correlation` correlates length/width innovations and stationary states.
`correlation_time_s=0` gives independent samples every simulation step. This is a
simulation-step process; it does not implement a separate camera frame rate or delay.

## Neutral defaults, domain and limits

All default mean coefficients and sigma references are zero: a neutral plugin keeps
its incoming dimensions unchanged. Default domain: 0.1–150 m, |azimuth| <= 60 degrees.
Outside the domain it passes through unchanged and records `outside_domain`; it does
not hide the object or simulate field-of-view detection. By default it also passes
through when a preceding identification plugin marks the class wrong. Set
`only_correct_class=false` only when the experiment requires dimension errors in those
frames. Place this plugin before detection so prediction/copying has a clear order.

The default log-error limit ln(4) bounds each plugin's multiplier to [0.25,4]. Clipping
is a declared numerical/model guard, NOT an inferred sensor property; both the raw
draw and clipping indicator are logged. Parameters are rejected if the deterministic
log-mean exceeds that limit or if log-ratio standard deviation exceeds 1 anywhere in
the configured domain. Those checks use the corners of the bilinear feature domain.

## Run

From the repository root, using the existing activated environment:

```
python -m pipeline list
python -m pipeline run configs/size_bearing_demo.json --output output/size_bearing_demo
python -m pipeline grid experiments/size_bearing_smoke.json --workers 1
python -m unittest discover tests -v
```

Use a new single-run output folder for each run. Set TME180_RUNTIME to the existing
engine location when it is outside this checkout. The installed local setup uses
`../rt` relative to this checkout, rather than its default `runtime/` directory.

`configs/size_bearing_demo.json` is deliberately marked as uncalibrated. Its nonzero
coefficients demonstrate increasing random variation and negative log-bias with
distance/absolute azimuth. They are not measured camera performance. The neutral
configuration and the no-error reference are controls. The nine-run smoke grid has
one seed per cell; it is an integration check, not a collision probability study.

Logs under `size_bearing.*` contain sensor distance/azimuth, application/skip flags,
true/input/estimated dimensions, mu/sigma, standard noise, raw log-errors, clipping
flags and final dimensional error in metres. Settings and `parameter_source` are
stored in the run summary and reproducibility metadata follows the normal pipeline.

## Calibrate and validate

Collect timestamp, track ID, true/estimated physical length and width, sensor distance
and azimuth, plus object class/pose where possible. Form log(estimated/true) residuals.
Fit conditional means and variances; constrain sigma positive and validate on held-out
tracks/sequences. Describe the coefficient source in `parameter_source`. Control for
target pose/class/occlusion so their effects are not silently attributed to azimuth.

Current TASC rear-end runs have azimuths around zero (the existing smoke runs are
within about +/-0.6 degrees). Large-azimuth behaviour needs independent validation
data and suitable geometry. Offline surface sweeps validate equations, not sensor
accuracy. To change azimuth, change target position or sensor pointing; rotating only
the target changes its aspect angle. Compare AEB variants on the same geometry and
seeds. The example controller reads length/width, enabling a bridge effect test;
an unchanged AEB result from another controller may indicate low input sensitivity.

Required checks include neutral equivalence, geometry/rotation, angular symmetry,
the distance-angle interaction, log-error statistics, OU and cross-dimension
correlation, target/label stream independence, clipping, protocol SIZE flags, true
dimension restoration and an actual controller response. Results remain sensitivity
results until validated against representative sensor dimension estimates.
