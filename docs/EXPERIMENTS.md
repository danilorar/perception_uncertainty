# Experiments

Every grid in `experiments/` gets a row here. Results live in `output/grids/<name>/` (not in Git),
so this file is the shared record of what was run and what it showed.

## Protocol
1. **One question per grid.** Write the question and the decision rule here *before* running it.
2. **Error values are assumed.** Report results as sensitivity ("collisions start above X % range
   error"), never as measured sensor risk.
3. **Always include a reference:** a `no_error` group on the same scenarios, so every change is
   compared with the error-free controller.
4. **Paired comparisons.** Two variants run on the same scenarios and seeds are compared seed by seed
   (difference per seed, then mean and a bootstrap 95% CI), not as two separate averages.
5. **Report the seed count.** Rates from few seeds (e.g. 2 collisions in 50) are uncertain: give the
   count and a confidence interval, not only the percentage.
6. **Reproducibility check:** the same config and seed must give the same `summary.json` on the same
   machine. Windows and macOS may differ slightly; say which machine produced a number.
7. **Negative results are results.** "No effect up to X" goes in the table with one line of reason.

## Grid log

| Grid | Question | Runs | Date run | Machine | Main result | Verdict |
|---|---|---|---|---|---|---|
| `smoke` | Does the pipeline work end to end? | 6 | | | | |
| `size_bearing_smoke` | Does the physical-size plugin pass through neutrally and run with assumed distance/azimuth coefficients? | 9 | 2026-11-12 | Windows 11 x64 | 9 completed; neutral ego outputs equal all 3 references; 6 road-geometry warnings | Integration passed; coefficients uncalibrated |
| `range_factors` | Size, identification, detection and OU errors, each alone: which matter? | 1899 | | | | |
| `detection_seeds_50_99` | Extra seeds 50–99 for the detection groups of `range_factors` | 600 | | | | |
| `ou_size` | How large must the OU error be before safety changes? | 282 | | | | |
| `ou_tau` | OU error: mean error at 100 m x correlation time | 1803 | | | | |
| `lost_target` | What the vehicle gets while the target is lost (vanish / predict / frozen) | 3000 | | | | |

## Notes per grid

### size_bearing_smoke

Preregistered decision rule (2026-11-12): every baseline/neutral/demo run must complete;
the neutral controller outputs must equal the matching baseline outputs. Perception
logs must contain applied physical dimensions, and demo dimensions must differ from
truth. This checks integration only: current rear-end scenarios do not cover large
azimuths, and one seed per cell cannot establish a collision probability. Assumed demo
coefficients and model details are in SIZE_BEARING.md. Separately, the integration test
must show that increased perceived length reaches the example AEB and changes its
braking time while true dimensions remain unchanged.
<!-- Optional: one short section per grid with the decision rule (written before the run),
     the key table, and a link to plots. Delete this comment when the first one is added. -->
