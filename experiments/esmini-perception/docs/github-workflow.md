# GitHub 整理与提交 / GitHub organization and submission

作者 / Author: Zhuo Ma

用户选择：保留报告和汇总 CSV，忽略批量原始结果。本指南所有 Git 命令从仓库根目录执行，运行工具从 `simulation/acc_cpp/` 执行。整理不自动 stage、commit 或 push。

Selected policy: retain reports and summary CSVs, ignore bulk raw results. Run Git commands from the repository root and experiment tools from `simulation/acc_cpp/`. Organization does not automatically stage, commit or push.

## 提交边界 / Tracking policy

| 内容 / Content | Git | 原因 / Reason |
|---|---|---|
| 自写源码、配置、测试、文档 / Project code, configs, tests, docs | 提交 / Track | 可复现实现 / Reproducible implementation |
| `Data/*.xosc`, `Data/*.xodr` | 保留原有跟踪 / Keep tracked | 原始输入，保持来源与字节 / Original inputs, preserve attribution and bytes |
| 必需 pugixml 源码及第三方许可 / Required pugixml source and licenses | 提交 / Track | 从零构建依赖 / Clean-build dependencies |
| `simulation/acc_cpp/reports/` | 提交 / Track | 精选报告、汇总、参数和指纹 / Selected reports, summaries, settings, hashes |
| `simulation/acc_cpp/build/` | 忽略，本机保留 / Ignore, retain locally | 可重建引擎源码缓存与编译产物 / Rebuildable source cache and products |
| `simulation/acc_cpp/results/` | 忽略，本机保留 / Ignore, retain locally | 逐帧 CSV、DAT、日志、图像和视频 / Frame CSVs, DAT, logs, images, videos |
| `simulation/generated/` | 忽略，按需生成 / Ignore, regenerate | Python 教学派生场景 / Derived Python teaching scenarios |
| 下载的引擎、虚拟环境、缓存、GUI 日志 / Downloads, environments, caches, GUI logs | 忽略 / Ignore | 本机产物 / Local products |

原生 `build_native.py` 自行下载依赖，不要求根目录 `esmini/` 子模块已初始化。GUI 回放另需完整图形版安装。根目录 `esmini/` 是独立子模块，不能用普通文件方式随 experiments 提交。

Native `build_native.py` fetches dependencies independently and does not require an initialized root `esmini/` submodule. GUI replay needs a separate complete graphical installation. Root `esmini/` is a separate submodule and is not submitted as ordinary experiment files.

## 导出新结果 / Export new results

```bash
# 从 acc_cpp/ 运行 / Run from acc_cpp/
task_batch="$(cat results/latest.txt)"
python3 tools/export_report.py --batch-dir "$task_batch" --name aeb_comparison_my_run

task_sensitivity="$(cat results/latest_aeb_markov_sensitivity.txt)"
python3 tools/export_report.py --batch-dir "$task_sensitivity" --name aeb_sensitivity_my_run
```

输出到 `reports/<name>/`，拒绝覆盖已有目录。仅导出白名单内 comparison/sensitivity CSV 和 batch/experiment manifest；配对比较另保存完整单次配置为 `run_configs.json`。自动生成双语 README 与 `export_manifest.json`。`sensitivity_runs.csv` 是每个 seed 的一行指标，不是逐帧数据。

Output goes to `reports/<name>/` and existing directories are never overwritten. Only whitelisted comparison/sensitivity CSVs and batch/experiment manifests are exported; paired comparisons also retain full per-run settings in `run_configs.json`. A bilingual README and `export_manifest.json` are generated. `sensitivity_runs.csv` has one metrics row per seed rather than frame data.

数字单元格和哈希不变，本机绝对路径改成 `source_batch/...` 或 `esmini-perception/...`。manifest 记录原文件 SHA256 和转换后 SHA256。原始日志仍在本机；这些相对引用不表示逐帧数据已经随报告发布。导出后更新 [报告索引](../simulation/acc_cpp/reports/README.md)，并写明控制器、参数及结论。

Numeric cells and hashes stay unchanged; local absolute paths become `source_batch/...` or `esmini-perception/...`. The manifest records original and transformed file SHA256 values. Raw logs stay local; relative references do not mean frame data is published. Update the [report index](../simulation/acc_cpp/reports/README.md) after export, identifying controller, settings and conclusions.

## 检查后由自己提交 / Review and submit yourself

```bash
# 从 Git 仓库根目录运行 / Run from Git repository root
git rev-parse --show-toplevel
git status --short
git diff --stat -- experiments
git diff --check -- experiments

# 只暂存本次实验范围，包含删除和新文件 / Stage experiment additions, changes and deletions only
git add -A -- experiments
git diff --cached --name-only
git diff --cached --stat
```

检查暂存清单包含新的 AEB 桥接、测试、工具和 reports，且没有 build/results、dylib/DLL、DAT、缓存。`git diff` 不显示未跟踪文件内容，所以暂存后必须再核对清单。仓库根目录的 `.gitmodules` 和 `esmini/` 已有独立未提交变化，以上限定路径不会暂存它们。

Confirm staging includes new AEB bridges, tests, tools and reports, without build/results, dylib/DLL, DAT or caches. `git diff` omits untracked contents, so inspect the staged list. Existing separate changes in root `.gitmodules` and `esmini/` are excluded by the scoped staging command.

```bash
# 确认清单和目标分支后自行执行 / Execute after checking the list and target branch
git branch --show-current
git remote -v
git commit -m "Organize perception experiments and bilingual documentation"
git push
```

没有 upstream 时，按自己确认的远程和分支使用 `git push -u origin <branch>`。不要把访问令牌写入脚本、README 或仓库。原始数据和派生场景保持提供方已有的分享范围，不新增项目整体许可来覆盖老师数据或第三方许可。

Without an upstream, use `git push -u origin <branch>` with the remote and branch you have confirmed. Keep access tokens out of scripts, docs and Git. Preserve the provider's existing sharing scope for original/derived scenarios; no overall project license is added to override teacher data or third-party terms.

## 本次清理范围 / Cleanup scope

合并三份旧中文指南为双语文档；合并两个重复 simulation README；移除过时静态清单 `project-files.txt` 和五个 macOS `.command` 快捷方式；删除可再生成的 Python XOSC/manifest、缓存、`.DS_Store`、GUI 临时日志。用 Python 入口替代快捷方式。

Old Chinese guides were consolidated into bilingual docs, and two duplicate simulation READMEs were merged. The stale `project-files.txt` inventory and five macOS `.command` shortcuts were removed, together with regenerable Python XOSC/manifest files, caches, `.DS_Store` and temporary GUI logs. Python entries replace the shortcuts.

保留原始场景、全部本机科学结果、当前编译环境、早期教学源码、现有原生控制器修改和第三方许可。历史结果没有重新归类为当前 AEB 或 HiDrive；未启用 CI 模板、未提交或推送。

Original scenarios, all local scientific runs, the current build environment, teaching source, existing native-controller work and third-party licenses are retained. Historical results are not relabeled as current AEB or HiDrive. The CI template was not activated and no commit/push was made.
