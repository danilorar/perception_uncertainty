# ACC output schema

Author: Zhuo Ma

Current run configuration schema: **6**. Ego name, progress definition, native-resolved initialization/limits and requested scenario parameter overrides are recorded alongside sensor settings; uncertainty remains disabled. Historical files retain their original schemas.

| File | Content |
|---|---|
| `results.csv` | Boundary truth: step, time, ego x/y/heading/speed, progress (reference projection or accumulated planar travel, per `progress_source`), truth minimum gap/target/TTC/distance and collision |
| `detections.csv` | One row per ACC step: `step`, `controller_time_s`, `observation_mode` (`ideal`, `all-targets`, `reference`), `detected_target_ids` (quoted JSON array, including `[]`) |
| `native_acc.csv` | Internal ACC diagnostics and post-step ego state, described below |
| `summary.json` | Completion, collision/end reason, duration, first collision and speeds, minimum distance/gap/TTC, first deceleration, minimum acceleration, final ego speed and close-gap stop flag |
| `run_config.json` | Resolved ACC settings, source/road fingerprints, engine identity and observation/reference mode |
| `generated/` | Generated ACC XOSC and byte-identical road snapshot |
| `truth.csv`, `metrics.csv` | Optional detailed world states and safety metrics |
| `sim.dat`, `run.log` | esmini recording and log |

Detailed logs are off by default; the native comparison enables them. No control/dropout CSVs are emitted. `results.csv` contains pre-step truth and one terminal row, without unused controller/perception columns. Progress is a measurement, never an ego movement command. Empty metric cells mean unavailable/non-finite, not zero.

## Native diagnostics

- `time_s`: esmini clock at the internal ACC step, after scenario actions.
- `input_speed_mps`: live ego speed immediately before ACC.
- `ego_speed_mps`, `ego_x_m`, `ego_y_m`, `ego_z_m`: post-step ego state.
- `lead_id`: selected target inferred from upstream ACC's lookahead marker; `-1`
  means none. This is unambiguous for the supported two-vehicle scenarios.
- `target_count`: number of non-ego objects supplied to ACC.
- `observed_gap_m`: post-ACC free-space gap to the selected target, or infinity.
  This diagnostic is not the internal road-distance gap used by ACC's main gate.

Zero-time initialization uses esmini's normal controller lifecycle and is not logged in `native_acc.csv`. Native ACC can apply its set-speed ceiling during initialization. ACC receives detected snapshots in ideal mode, all non-ego snapshots in all-target mode, or live entities in reference mode. `target_count` counts the supplied observations. `detections.csv` uses the same controller-time timestamp; it has no terminal or initialization row.

`scenario_parameter_overrides` records optional NCAP `Ego_speed_kph`, `GVT_init_speed_kph` and `isCCRbraking` overrides. A null value means no override was requested; esmini retains the source default. Overrides affect Init through the native parameter callback and derived expressions are re-evaluated before vehicles are initialized.
