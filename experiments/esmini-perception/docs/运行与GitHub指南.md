# 运行、代码结构与 GitHub 上传

Author: Zhuo Ma

## 1. 整理后的四类内容

| 内容 | 位置 | 用途 |
| --- | --- | --- |
| 原始数据 | `Data/C_original_<编号>.xosc`、`.xodr` | 保持原文件及署名，代码只读 |
| 生成数据 | `simulation/generated/C_aeb_<编号>.xosc` | 自车交给 Python 控制，目标车继续原始轨迹 |
| ideal sensor | `simulation/my_sensor_demo.py` | 原始轨迹回放，只观察理想检测，不控制车辆 |
| ideal sensor + AEB | `simulation/my_aeb_demo.py` | dropout 固定为 0，可切换 AEB off/on |
| dropout sensor + AEB | `simulation/my_aeb_dropout_demo.py` | 默认 10% 漏检，可切换 AEB off/on |

脚本顶部作者统一为 `Zhuo Ma`。原始数据的 `FileHeader` 仍保留提供方作者；
生成数据保留该来源署名，另在 `generation_manifest.json` 记录转换者和文件哈希。

保留了原入口文件名，因此以前的 `--aeb`、`--ttc`、`--decel`、`--drop-prob`、
`--perception-seed` 命令仍可用。**dropout 入口的默认概率现在是 0.1**，
需要无漏检参考时明确传 `--drop-prob 0`。单次 AEB 脚本仍默认 `--aeb off`。

整理前脚本保存在 `simulation/archive/pre_cleanup_20260929/`，便于对照。
旧 Vector/NCAP 示例仍在 `simulation/aeb/`；当前三场景主流程不再依赖它，
该旧目录和旧结果不属于本次 GitHub 文件清单。

## 2. 公共代码应该在哪里修改

| 想修改什么 | 文件/位置 |
| --- | --- |
| esmini、原始数据、生成场景和结果的路径 | `simulation/paths.py` |
| ctypes State、传感器位置/FOV/范围、读取检测列表 | `simulation/esmini_api.py`：`State`、`add_sensor`、`fetch_ids` |
| 漏检概率模型/如何产生随机缺失 | `simulation/sensor_model.py`：`DetectionDropout.apply` |
| TTC、AEB 触发、制动保持、积分 | `simulation/aeb_controller.py` |
| 循环、感知到控制的数据传递、CSV 输出、打印 | `simulation/aeb_runner.py`：`run` |
| 从原始 XML 生成 AEB XML | `simulation/generate_scenarios.py` |
| 四组比较和多个 seed 批运行 | `simulation/run_comparison.py` |
| 从已有结果提取碰撞、触发、漏检率指标 | `simulation/compare_aeb_dropout.py` |

`my_aeb_demo.py` 和 `my_aeb_dropout_demo.py` 现在是简短入口。
两者共用同一个循环，避免修改一个版本后忘记同步另一个版本。

数据流程是：

```text
SE_FetchSensorObjectList → raw_ids → DetectionDropout.apply → observed IDs
                                                               ↓
SE_GetObjectState → ground truth → 根据 detected 屏蔽 → AEB → 自车运动
                          ↓
                     真实状态/KPI 日志
```

`SE_AddObjectSensor` 返回 sensor ID；`SE_FetchSensorObjectList` 返回数量并写入
object ID 数组。位置、尺寸、速度、objectType、objectCategory 来自单独读取的真值。
当前只控制 `object_1`，对预先选定的 `object_2` 计算风险，不做多目标筛选。

## 3. 从头运行

在终端进入项目根目录。例如本机：

```bash
cd "/Users/zhuoma/Library/CloudStorage/OneDrive-Chalmers/Course/TME180"
python3 --version
```

使用 Python 3.9+。主流程不需要 pip 安装 Python 第三方包。
若要使用可选的 GIF 渲染脚本，才需要在自己的环境中安装 Pillow。

安装 [esmini 3.8.1 官方 demo 包](https://github.com/esmini/esmini/releases/tag/v3.8.1)。
macOS 使用包含 `bin/libesminiLib.dylib`、`bin/esmini` 和 `resources/` 的完整包。
将它解压为 `simulation/esmini-demo/`；不要只复制 dylib。
本次实际验证的平台为 macOS，Windows/Linux 文件名适配不等于已完成平台验证。

如果安装在别处，可以使用：

```bash
export ESMINI_HOME="$HOME/tools/esmini-demo"
```

没有设置该变量时就使用项目内的默认路径。默认结果写到本机
`~/TME180-local/results/data-scenarios/`，避免高频写入 OneDrive。

生成三个场景的外部控制版本：

```bash
python3 simulation/generate_scenarios.py --scenario all --overwrite
```

`--overwrite` 只覆盖生成的 `.xosc`，不会改写 `Data/`。
生成器移除自车原来的轨迹 ManeuverGroup，替换 HiDriveController 为
ExternalController，将控制器激活放入单独 PrivateAction，并修正道路相对路径。
自车原始路径仍由 Python 从 `Data/` 读取；因此不能删掉原始 `.xosc`。
道路文件继续复用原始 `.xodr`，不产生没有修改必要的重复道路副本。

只查看 ideal sensor：

```bash
python3 simulation/my_sensor_demo.py --scenario 1549178
```

ideal 感知，分别关闭/开启 AEB：

```bash
python3 simulation/my_aeb_demo.py --scenario 1549178 --aeb off
python3 simulation/my_aeb_demo.py --scenario 1549178 --aeb on
```

10% dropout，使用同一个 seed 比较 AEB：

```bash
python3 simulation/my_aeb_dropout_demo.py --scenario 1549178 --aeb off --drop-prob 0.1 --perception-seed 42
python3 simulation/my_aeb_dropout_demo.py --scenario 1549178 --aeb on --drop-prob 0.1 --perception-seed 42
```

以上单次 sensor/AEB 命令默认显示窗口。加 `--headless` 则不显示窗口、取消实时等待，
仿真时间步仍为 0.01 s。例如：

```bash
python3 simulation/my_aeb_demo.py --scenario 1549178 --aeb on --headless
```

切换场景编号即可运行 `1554254` 或 `1554431`。纯原始轨迹批量验证：

```bash
python3 simulation/run_scenarios.py
```

该批运行脚本现在也写入统一的结果目录，不再往 `Data/` 创建结果子目录。
新 `latest.txt` 指向这次原始轨迹批运行；旧结果留在原位置，可单独查看。

## 4. 四组批运行及已有结果比较

```bash
python3 simulation/run_comparison.py --scenario 1549178 --seeds 42
```

自动运行 AEB off/on × dropout 0/0.1，所有运行共用 dt、TTC、减速度和 seed。
终端最后打印 `comparison.csv` 的绝对路径。对应 batch 文件夹还包含四个运行目录、
每次 console 日志及 `runs.json`。如需多个 seed：

```bash
python3 simulation/run_comparison.py --scenario 1549178 --seeds 1 2 3 4 5
```

这是 5 × 4 = 20 次仿真。0-dropout 参考组在每个 seed 下重复，用于保持每组结构一致。
有限样本的实际漏检率不一定正好 10%；AEB 改变轨迹后，off/on 的可检测样本数也可能不同。
同一 sensor/object/tick 的随机决定保持一致，不能据单个 seed 断言 10% 漏检没有影响。

只汇总已有运行时，提供**运行文件夹**，不要提供 CSV 文件：

```bash
python3 simulation/compare_aeb_dropout.py \
  "/absolute/path/to/run_off" \
  "/absolute/path/to/run_on" \
  --output "/absolute/path/to/new_comparison.csv"
```

把示例路径替换成终端打印的真实路径。输出文件必须是新文件；使用 `--output`
可避免之前将 `dropout_comparison_seed42.csv` 误当成运行文件夹的问题。
若输入列表意外包含文件，脚本会在 stderr 提示并跳过。

## 5. 上传到 GitHub：已有团队仓库

当前课程目录还不是 Git 仓库。若团队已经有仓库，建议把整理后的内容导入一个独立
子目录，避免覆盖团队已有 README、配置或其他成员代码。

先替换下面的 `REPOSITORY_URL` 为团队真实 GitHub URL：

```bash
git clone REPOSITORY_URL "$HOME/TME180-project"
git -C "$HOME/TME180-project" switch -c feature/esmini-perception
```

回到这份代码根目录，按照明确的文件清单复制，原始数据和生成场景都包含在内：

```bash
cd "/Users/zhuoma/Library/CloudStorage/OneDrive-Chalmers/Course/TME180"
mkdir -p "$HOME/TME180-project/experiments/esmini-perception"
rsync -av --files-from=project-files.txt ./ "$HOME/TME180-project/experiments/esmini-perception/"
```

如果 `$HOME/TME180-project` 已经是你的仓库，不要重新 clone，直接在该仓库切换新分支。
复制目标子目录应为空，或确认属于你这次提交的内容。

然后审阅、提交、推送分支：

```bash
cd "$HOME/TME180-project"
git status --short
git add experiments/esmini-perception
git diff --cached --stat
git diff --cached
git commit -m "Add ideal sensor and dropout AEB simulations"
git push -u origin feature/esmini-perception
```

如果 Git 尚未配置提交身份，在 commit 前设置本仓库的姓名和真实 GitHub 邮箱：

```bash
git config user.name "Zhuo Ma"
git config user.email "YOUR_GITHUB_EMAIL"
```

最后在 GitHub 打开 Compare & pull request，说明 ideal/dropout 入口、三份原始数据、
三份生成场景及验证方式。这里的脚本 Author 和 Git 提交身份是两个不同设置。

## 6. 上传到 GitHub：创建独立新仓库

在 GitHub 建立空仓库，不预先添加 README、license 或 .gitignore。
随后在当前代码根目录运行以下命令，替换 `REPOSITORY_URL`：

```bash
git init -b main
git add .gitignore README.md project-files.txt docs Data simulation tests
git status --short
git diff --cached --stat
git commit -m "Organize esmini perception and AEB experiments"
git remote add origin REPOSITORY_URL
git remote -v
git push -u origin main
```

`.gitignore` 已排除 esmini 下载目录、虚拟环境、旧 Vector 实验、备份、运行结果及课程文件。
`Data/` 和 `simulation/generated/` 都会提交；不会忽略所有 CSV，以免以后需要共享
精选比较表时受到全局规则阻碍。不要通过 `git add -f` 把整个 esmini 或 results 加进去。

原始文件说明其来源是 GIDAS rear-end crash data，并保留 CAP/Chalmers 署名。
当前文件夹未附数据再分发许可；原始和生成数据的分享范围应遵循提供方约定，
脚本作者信息不代表取得了数据再分发权。

流程参考：[GitHub 官方本地代码上传指南](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)。

## 7. 队友 clone 后如何运行

进入 `experiments/esmini-perception/`（独立仓库则进入仓库根目录），安装 esmini 3.8.1
完整 demo 包到 `simulation/esmini-demo/`，或设置自己的 `ESMINI_HOME`。
随后运行上面的生成器和 `run_comparison.py` 即可。代码不依赖 Zhuo Ma 的绝对路径。
数据目录保留大写 `Data`，Linux 上不要改成小写 `data`。

检查 Python 逻辑和生成场景转换：

```bash
python3 -m unittest discover -s tests -v
```

当前验证记录见 [验证记录](验证记录.md)。输出字段定义见 [输出字段](输出字段.md)。
