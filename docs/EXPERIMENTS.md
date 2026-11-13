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
| `size_geometry` | A wrong reported length of the target box (`box` plugin): does braking follow the box length, and how much does safety change when the centre or the near end stays right? | 27 | 2026-11-10 | Windows 11, `711fd6a` | Target is 5.0 m long, so ±10% and ±20% move each end by 0.25 and 0.5 m. Near end right: all 12 runs equal the no-error run (differences below 1e-5). Centre right, safe scenarios: no crash; shorter box (near end read too far) lowers the minimum gap, `gidas_1549178` 3.20 → 3.17 → 3.13 m and `gidas_1554254` 1.90 → 1.84 → 1.74 m at −10 and −20%; longer box raises it on `gidas_1554254` (2.11, 2.60 m) and changes nothing on `gidas_1549178`. `gidas_1554431` (crashes without error at 10.5 km/h relative): 12.6 and 17.5 km/h at −10 and −20%, 3.1 km/h at +10%, no crash at +20% (gap 0.18 m) | The controller reacts to the gap to the near end of the box. A length error changes safety only if the centre, not the near end, is what is measured right, and then by half the length error: same direction as a range bias (too far is worse), no new crash up to ±20% on the safe scenarios. One run per level, no randomness. Sensitivity levels, not sensor values |
| `lost_target` | What the vehicle gets while the target is lost (vanish / predict / frozen) | 3000 | 2026-11-08 | Windows 11, `247f6cc` | Safe scenarios, against 30/400 when the target vanishes: object speed held (`ou_speed`) 17/400 (2.7–6.7%); speed difference held (`frozen`, `ou_frozen`) 14/400; true speed kept (`predict`, `ou_predict`) 4/400. `gidas_1554431`: 162/200 with OU, 198–200/200 without | Holding the object speed avoids 13 of the 30 vanish crashes and adds none. The 4/400 groups still get the true target speed, so they flatter the prediction. Same as the 2026-11-02/03 studio sweeps, run by run |
| `tasc_baseline` | No error on every imported TASC case: which ones does HiDrive avoid, and do converted copies match the shared files? | 133 | 2026-11-11 | Windows 11, `e6f1024` | Car-car 81 of 101 safe; the 20 crashes are 17 cases in lane at ego 95–110 km/h and 3 side contacts where HiDrive never triggers. Cyclist 20 of 24 safe; the 4 crashes enter the ego path late or touch at the side. Cut-in 5 of 5 crash and HiDrive never triggers. `tasc_car_1554254` and `1554431` equal the shared runs exactly; `tasc_car_1549178` (real road instead of the straight one) differs by under 0.01 m | Every car-car case up to ego 75 km/h is safe without error; the cut-in cases cannot show a range-error effect with this controller |
| `tasc_tolerance` | Do the limits found on the three shared scenarios hold across the TASC car-car and cyclist cases? | 5625 | 2026-11-11 | Windows 11, `e6f1024` | Car-car, 81 cases safe without error, 10 seeds each: OU 5/10/20% crash in 1.2% (0.7–2.3%), 6.8% (5.3–8.7%), 16.8% (14.4–19.5%) of runs (shared safe scenarios 0/60, 5/60, 8/60); 10% miss rate 5.2% (3.9–6.9%). Too-far bias crashes 2, 3, 14, 23 of 81 cases at +5, +10, +15, +20%; 58 do not crash up to +20%. Cluster ego/target 65/55 km/h (17 cases): 11 and 16 crash at +15 and +20%; cluster 45/35 (44 cases): 0 and 1 (`1549178`). Cyclist (a car-sized box), 20 safe cases: no crash from OU or bias up to 20%, miss rate 4/200. Car-car cases that crash without error and trigger (17): median relative impact +3, +8, +10 km/h at OU 5/10/20% (22–38% of those runs avoid the crash), +10 to +22 km/h with bias, never avoided | The random-error onset near 10% holds on average. "Too far crashes for sure from 15–20%" holds only where the margin is small: the minimum gap without error, divided by the range at braking start, ranks the cases (Spearman −0.66), and cases under 15% hold 20 of the 23 crashes at +20%. The three shared scenarios are among the more sensitive cases of their clusters. Sensitivity levels, not sensor values |

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

### `size_geometry`
- `box` changes only the reported length; `keep: near_end` also moves the centre by half the length error along
  the line of sight, so the end facing the ego stays where it is. `size` (range scaled by an assumed size) is
  the camera-type link and gives much larger errors: scale 1.1 is 1.89 m at braking start on `gidas_1554254`, against 0.25 m here.
- The first trigger moves only on `gidas_1554254` at +20% (3.20 → 3.16 s) and on `gidas_1554431` (3.23 s without
  error; 3.28, 3.24, 3.21, 3.19 s at −20, −10, +10, +20%). Why a longer box changes nothing on `gidas_1549178`
  is not known; the controller is a black box here.
- Which point a real perception system measures right is an assumption, not a result of this grid.

### `lost_target`
- The vanish reference is the detection groups of `range_factors`: same seeds, same misses.
- `frozen` holds the speed difference of the last seen step, so its distance does not follow the ego's
  braking; `ou_speed` holds the object speed and uses the ego's live speed. No `ou_speed` run without OU
  exists, so 17/400 is with OU only.
- OU enters only as the error of the last seen reading and stays constant through a miss.

### TASC cases (`tasc_baseline`, `tasc_tolerance`, `tasc_bicycle`)
- Source: the teacher's upload of 2026-11-11 (Cut-in 7, Rear-end car-car 110, Rear-end car-cyclist 32 files), imported
  with `python -m pipeline import-tasc`. The files stay out of Git (teacher data, public repository); each registry
  entry in `pipeline/scenarios/local/scenarios.json` keeps the SHA-1 of its source file.
- 130 of 149 imported. Skipped: 7 whose boxes overlap from the start (car-car 1553190, 1556158, 1571692; cut-in 1539179,
  1567320; cyclist 1587666, 1614846), 6 car-car copies of cut-in cases (the teacher's cut-in `_s` files swap the roles
  and have their own road), and 6 cyclist cases where object_2, the car, strikes the cyclist object_1 from behind.
- Those 6 were first run with the trajectories swapped: in 5 the ego jumped about 10 m along the road at brake onset.
  The road follows object_1's path, so a swap needs a road along the new ego's path; they are skipped until one exists.
- Cyclists are a 5.0 x 1.8 m car box in the files (category `car`, model `car_white.osgb`, the same as the ego), so
  the `tasc_cyclist_*` results are about a slow car-sized target.
- `tasc_bicycle` (2026-11-13, 1080 runs, none failed): `import-tasc --bicycle` writes the same 24 cases a second time
  as `tasc_bicycle_*`, with object_2 as category `bicycle`, a 1.8 x 0.6 x 1.7 m box and `cyclist.osgb`. The size is
  assumed; the files hold none. The box stays centred on the trajectory point. Groups and levels are those of
  `tasc_tolerance`, so each case compares with its car-box run.
  - HiDrive brakes for the bicycle as for the car. Without error 4 of 24 crash with either box, the same 4 cases
    (`1554559`, `1555622`, `1588450`, `1598752`), 3–4 km/h slower with the bicycle box.
  - Crashes, car box against bicycle box: OU 5% 40/240 and 40/240, OU 10% 35/240 and 37/240, OU 20% 27/240 and
    28/240, bias +5 to +20% 4/24 at every level with both, 10% misses 44/240 and 45/240.
  - In the 20 cases that are safe without error, the minimum gap changes by −0.18 to +0.23 m: HiDrive brakes on the
    distance to the box, so a shorter box moves the stopping point, not the gap.
  - These runs were made before the importer change was committed (the grid notes "files differ from commit c9ecdde").
- Cut-in: contact is at the side, with the cutting car faster than the ego, and HiDrive never triggers in any of the 5
  cases. A range error cannot change them; a lateral or lane-assignment error with a controller that reacts to cut-ins
  could.
- Polyline roads make esmini report "Last geometry of road 0 does not end at road end", so those runs are
  `completed_with_warnings`. The shared `gidas_1554254` and `gidas_1554431` give the same line; results are unaffected.
- Check: the 68 OU and bias runs of `tasc_car_1554254` and `tasc_car_1554431` equal the `ou_tau` (tau 0.5 s) and
  `ou_bias` runs with the same seed.
- Car-car safe cases by margin without error (minimum gap / range at braking start): under 10%, 3 cases, crash at +20%
  bias in 3 and OU 10% in 14/30 runs; 10–15%, 18 cases, 17 and 23/180; 15–20%, 5 cases, 2 and 1/50; 20–30%, 47 cases,
  1 and 17/470; over 30%, 8 cases, 0 and 0/80. The two cases that crash already at OU 5% and bias +5% (`1571693`,
  `1620061`, ego 95–100 km/h) keep 0.2–0.3 m without error.
- Per-case table: `output/grids/tasc_tolerance/per_case.csv` (with the analysis script next to it).
