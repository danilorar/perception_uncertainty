# Results of earlier grids

These tables come from older code, HiDrive on esmini 3.8.1, run on Windows from 2026-11-01 to 2026-11-03. Their columns
differ from what `python -m pipeline grid` writes today. The `.md` tables are in Chinese.

| Folder | Grid in `experiments/` | Files |
|---|---|---|
| `range_factors` | `range_factors` | `results.csv`, `cell_summary.csv`, `tolerance_table.md` |
| `lost_target` | `lost_target` | `results.csv`, `cell_summary.csv`, `lost_target_table.md` |
| `ou_size` | `ou_size` | `results.csv`, `size_table.md` |

- `gidas_*` are scenario IDs. The data are TASC data.
- Error values are assumed, so these are sensitivity results, not measured sensor risk.
- `ou_tau` is not here: its earlier run used esmini 2.40, which the project no longer uses. Run the grid again if you need it.
- `detection_seeds_50_99` is not here either.
