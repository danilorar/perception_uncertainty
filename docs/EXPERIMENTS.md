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
| `smoke` | Does the pipeline work end to end? | 6 | 2026-11-08 | Windows 11, `247f6cc` | 6/6 finish; only `gidas_1554431` without error crashes | Works; same numbers before and after a fresh setup from `main` |
| `range_factors` | Size, identification, detection and OU errors, each alone: which matter? | 1899 | 2026-11-08 | Windows 11, `247f6cc` | Safe scenarios: target vanishing while missed 30/400 crash (95% CI 5.3–10.5%); wrong class ×1.77 at 20% 9/60 (8.1–26.1%), at 5% 1/60; size scale 1.1 1/2, 1.2 2/2; OU 0.8% and 1.4% 0/60; all other cells 0 | At these assumed values detection, identification (×1.77) and size (from +10%) can cause crashes, OU at 0.8–1.4% does not. Same as the 2026-11-01 studio sweep, run by run |
| `detection_seeds_50_99` | Extra seeds 50–99 for the detection groups of `range_factors` | 600 | | | | |
| `ou_size` | How large must the OU error be before safety changes? | 282 | 2026-11-08 | Windows 11, `247f6cc` | Safe scenarios: 0/60 at 5%, 5/60 at 10% (3.6–18.1%), 8/60 at 20% (6.9–24.2%). `gidas_1554431`: 17, 14, 12 of 30 crash, median impact 17.5, 19.6, 24.7 km/h (no error 10.5) | Safe scenarios start crashing between 5% and 10%; on `gidas_1554431` fewer but harder crashes as the error grows. Sizes are sensitivity levels, not sensor values. Same as the 2026-11-03 studio sweep, run by run |
| `ou_tau` | OU error: mean error at 100 m x correlation time | 1803 | 2026-11-08 | Windows 11, `8954bdf` | Safe scenarios, crashes of 60 at tau 0.05 / 0.2 / 0.5 / 1 / 2 s: 5% 0/0/0/0/0; 10% 0/4/5/4/5; 20% 0/1/8/8/9; 50% 0/0/1/3/9 (4/60: 2.6–15.9%, 9/60: 8.1–26.1%). At tau 0.05 s no crash up to 50%, but the brake triggers more than 0.1 s early in 47–60 of 60 runs from 10% | Up to 5% no crash at any correlation time. From 10% crashes need an error that lasts (none at 0.05 s, 4–5/60 from 0.2 s); larger errors that change fast trigger early braking instead of crashes. Sensitivity levels, not sensor values |
| `ou_bias` | Same mean error size: does a constant too-far reading hurt more than random error, and where does bias plus random error start to crash? | 579 | 2026-11-08 | Windows 11, `685e3a0` | Constant bias, one run per level: reading too near (−5, −10%) never crashes and also avoids the no-error crash on `gidas_1554431`; reading too far first crashes at +15% on `gidas_1554254` and +20% on `gidas_1549178` (about 2.4 m at braking start), while the gap shrinks from +5% (`gidas_1554254` 1.90 → 0.56 m at +10%). Random error alone (`ou_tau`, tau 0.5 s) already crashes 5/60 at 10%. Bias + random, safe scenarios: random 2.5% with bias up to +5% 0/60; random 5% 0/60, with +2.5% bias 2/60, with +5% bias 4/60 (2.6–15.9%). `gidas_1554431`: random 2.5% 20/30 crash, with +5% bias 30/30, median impact 13.2 → 22.1 km/h | Direction matters: too near is safe here, too far eats the margin in every run and crashes for sure from about 15–20%. A random error of the same mean size crashes earlier (10%) but only in some runs, as it is sometimes two to three times its mean. A +5% bias and 5% random error, harmless alone, crash together about as often as 10% random alone. Sensitivity levels, not sensor values |
| `lost_target` | What the vehicle gets while the target is lost (vanish / predict / frozen) | 3000 | 2026-11-08 | Windows 11, `247f6cc` | Safe scenarios, against 30/400 when the target vanishes: object speed held (`ou_speed`) 17/400 (2.7–6.7%); speed difference held (`frozen`, `ou_frozen`) 14/400; true speed kept (`predict`, `ou_predict`) 4/400. `gidas_1554431`: 162/200 with OU, 198–200/200 without | Holding the object speed avoids 13 of the 30 vanish crashes and adds none. The 4/400 groups still get the true target speed, so they flatter the prediction. Same as the 2026-11-02/03 studio sweeps, run by run |

## Notes per grid

### Scenarios and the 2026-11-08 check
- `gidas_1549178` and `gidas_1554254` do not crash without error ("safe scenarios" above);
  `gidas_1554431` crashes without error at 10.5 km/h. The data are TASC, although the names say GIDAS.
- `smoke`, `range_factors`, `ou_size` and `lost_target` ran on `main` at `247f6cc` with no local changes
  and 0 failed runs. Each run was compared with the simulation-studio sweeps of 2026-11-01 to 11-03
  (collision, impact speed, trigger time, minimum gap and TTC): `range_factors` 1899/1899 and
  `lost_target` 3000/3000 identical; `ou_size` 279/279 identical (its 3 `no_error` runs had no
  counterpart there and match `range_factors`).
- CIs are Wilson 95% intervals on the crash count.

### `range_factors`
- Detection seeds 0–49 miss 11.9% of frames on average (IID 10.3%, T = 0.2 s 11.2%, 0.5 s 12.7%,
  1 s 13.4%) against the set 10%, so 30/400 may be high; `detection_seeds_50_99` is the check.
- Size runs once per level and scenario (no randomness), so where crashes start is a coarse boundary.
- Identification at 1% gives a wrong class in only 3 of 30 seeds, so 0/60 there proves little.

### `ou_tau`
- Seeds 1–30 (`ou_size` used 0–29). The tau 0.5 s runs on the 29 shared seeds equal `ou_size` run by run
  (261/261), and 0 of 1803 failed.
- OU changes every 0.01 s step, faster than the 0.05 s sensor frame of `detection`, so tau 0.05 s is
  close to independent noise per frame.
- "Early" means the first trigger is more than 0.1 s before the no-error run on the same scenario.
- `gidas_1554431` (crashes without error at 10.5 km/h): at tau 0.5 s 18, 15, 12, 3 of 30 crash at 5, 10,
  20, 50%, median impact 18.7, 20.5, 24.7, 18.6 km/h; at tau 0.05 s 16, 7, 0, 0 of 30.

### `ou_bias`
- `bias_pct` uses the same distance law as `mae_pct`, so the same percentage gives the same mean error size. At
  braking start (no-error run) 1% is about 0.12 m on `gidas_1549178`, 0.16 m on `gidas_1554254` and 0.23 m on
  `gidas_1554431`.
- Bias-only runs have no randomness: one run per level and scenario, so where crashes start is a coarse boundary.
- Random 5% with bias 0 equals `ou_tau` 5% at tau 0.5 s run by run (90/90). With `bias_pct` 0 the plugin skips
  the bias, and 120 `ou_tau` runs repeated exactly before the grid ran.
- `size` scale 1.1 crashed `gidas_1554254` in `range_factors`, while bias +10% here does not: the size law
  (scale − 1) × r gives a larger offset at that range (1.89 against 1.64 m when braking starts).

### `lost_target`
- The vanish reference is the detection groups of `range_factors`: same seeds, same misses.
- `frozen` holds the speed difference of the last seen step, so its distance does not follow the ego's
  braking; `ou_speed` holds the object speed and uses the ego's live speed. No `ou_speed` run without OU
  exists, so 17/400 is with OU only.
- OU enters only as the error of the last seen reading and stays constant through a miss.
