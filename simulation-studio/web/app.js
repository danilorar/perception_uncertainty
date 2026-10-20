const $ = id => document.getElementById(id);
let boot, selectedRun = null, current = null, activeRun = null, tab = 'video', timer = null, toastTimer = null;
let language = 'zh', historyRuns = [], busy = false, serviceStopped = false, initialized = false, formError = null;
let managingHistory = false, deletingHistory = false, historyRequest = 0, plotRevision = 0, pendingHistoryDeletion = [];
const checkedHistory = new Set();
let loadingRun = false, configRun = null, selectionRequest = 0, runRequest = 0, runningAction = 'new';
let comparing = false, compareTab = 'speed', compareRuns = [], compareRequest = 0, compareShowAll = false, compareChart = null;
try { if (localStorage.getItem('simulation-studio.language') === 'en') language = 'en'; } catch { /* Storage may be unavailable in private browsing. */ }
const messages = {
  'parameters': ['实验参数', 'Parameters'],
  'secScene': ['场景', 'Scene'],
  'secEgo': ['自车', 'Ego vehicle'],
  'secPerception': ['感知误差', 'Perception error'],
  'secController': ['控制器', 'Controller'],
  'secOutput': ['输出', 'Output'],
  'secSensor': ['传感器', 'Sensor'],
  'maeShort': ['参考处 MAE', 'MAE at reference'],
  'tauShort': ['相关时间 τ', 'Correlation time τ'],
  'fromScene': ['来自场景', 'from scene'],
  'noVideoShort': ['无视频', 'No video'],
  'minGap': ['最小车距', 'Minimum gap'],
  'historyFilter': ['筛选历史记录', 'Filter history'],
  'filterScene': ['本场景', 'This scene'],
  'filterAll': ['全部', 'All'],
  'historySceneEmpty': ['当前场景还没有记录', 'No records for this scene yet'],
  'compareHint': ['勾选两条记录即可对比', 'Tick two records to compare'],
  'compareHintOne': ['再勾选一条记录', 'Tick one more record'],
  'compareHintReady': ['A 为先勾选的记录', 'A is the record ticked first'],
  'compareSelected': ['对比所选 ({count}/2)', 'Compare selected ({count}/2)'],
  'compareCheckbox': ['加入对比：{title}', 'Add to comparison: {title}'],
  'compareTitle': ['对比两次实验', 'Compare two runs'],
  'swapCompare': ['交换 A / B', 'Swap A / B'],
  'closeCompare': ['返回单次结果', 'Back to single run'],
  'compareViews': ['对比视图', 'Comparison views'],
  'compareSpeed': ['速度叠加', 'Speed overlay'],
  'compareGap': ['车距叠加', 'Gap overlay'],
  'compareError': ['测距误差', 'Range error'],
  'compareVideo': ['视频并排', 'Videos side by side'],
  'paramsHeader': ['参数差异', 'Settings that differ'],
  'resultsHeader': ['结果', 'Results'],
  'sameParams': ['其余 {count} 项参数相同', '{count} other settings are identical'],
  'noDiffParams': ['两次实验参数完全相同', 'Both runs use identical settings'],
  'showAll': ['展开', 'Show'],
  'hideAll': ['收起', 'Hide'],
  'sameValue': ['相同', 'Same'],
  'finalSpeed': ['结束时自车速度', 'Final ego speed'],
  'minTtc': ['最小 TTC', 'Minimum TTC'],
  'differentScene': ['两条记录来自不同场景，叠加曲线仅供参考。', 'These runs use different scenes; overlays are indicative only.'],
  'chartLoading': ['正在读取数据…', 'Loading data…'],
  'chartUnavailable': ['记录缺少所需数据文件', 'A record lacks the required data file'],
  'chartTime': ['时间 (s)', 'Time (s)'],
  'speedAxis': ['自车速度 (km/h)', 'Ego speed (km/h)'],
  'gapAxis': ['最小车距 (m)', 'Minimum gap (m)'],
  'errorAxis': ['有效测距误差 (m)', 'Valid range error (m)'],
  'playBoth': ['同时播放', 'Play both'],
  'pauseBoth': ['同时暂停', 'Pause both'],
  'compareNeedsComplete': ['只能对比已完成的记录', 'Only completed runs can be compared'],
  'compareLoadFailed': ['无法读取对比记录：{message}', 'Could not load the compared runs: {message}'],
  'nativeAccOption':['原生 C++ ACC · 团队基准', 'Native C++ ACC · team baseline'],
  'hidriveOption':['老师的 HiDrive AEB', 'Teacher’s HiDrive AEB'],
  'hidriveHint':['唯一的控制器：场景文件自带的 HiDriveController，在隔离编译的 esmini 3.8.1 中运行，参数取自场景文件。选择 OU 测距误差时，每个控制器步内把目标移到“感知位置”，步后恢复真值。首次碰撞即结束。', 'The only controller: the HiDriveController declared in the scene, run in the isolated esmini 3.8.1 build with parameters from the scene file. With OU range error, targets are moved to their perceived position during each controller step and restored afterwards. The run stops at first contact.'],
  'egoHidriveHint':['使用场景初速和航向；HiDrive 控制器从 t=0 接管自车。', 'Uses the scene’s initial speed and heading; the HiDrive controller drives the ego from t=0.'],
  'acc_time_gap_s': ['ACC 跟车时距', 'ACC time gap'],
  'nativeAccHint': ['从 t=0 起由原生 ACC 控制速度，沿原空间路径行驶。巡航速度取场景初速；加减速度上限来自场景车辆参数。TTC 仅作诊断。', 'Native ACC controls speed from t=0 along the original spatial path. Cruise speed uses the initial scene speed; actuator limits come from the source vehicle. TTC is diagnostic only.'],
  'egoAccHint': ['使用场景初速和航向，ACC 从 t=0 接管纵向控制。', 'Uses the scene’s initial speed and heading; ACC takes longitudinal control at t=0.'],
  'rawProcessing': ['直接使用有效观测', 'Direct valid observations'],
  'legacyFiltered': ['旧版滤波结果', 'Archived filtered result'],
  'exactProcessing': ['准确距离直接传递', 'Exact range passed directly'],
  'rawProcessingHelp': ['有效测距直接交给控制器，RMSE 统计本次全部有效观测相对真值的误差。', 'Valid readings go directly to the controller. RMSE measures all valid observations against this run’s truth.'],
  'removedNoiseConfig': ["独立高斯误差已移除；请在界面载入旧配置并重新保存为 OU 相关误差配置。", "Independent Gaussian error has been removed. Load the old settings in the interface and save an OU configuration."],
  'removedControllerConfig': ['配置中的“无制动 · 恒速对照”已从界面移除，请选择现有控制器后重新保存配置。', 'No braking · constant speed has been removed from the interface. Choose an available controller and save a new configuration.'],
  'observation_period_s': ['观测更新周期', 'Observation interval'],
  'legacyClock': ['跟随仿真步长 · 旧版', 'Every simulation step · legacy'],
  'aebFollowOption': ['旧版 Python AEB · 轨迹接管', 'Legacy Python AEB · path takeover'],
  'recommendedAeb': ['对齐团队 ACC 参数', 'Apply team ACC settings'],
  'aeb_trigger_mode': ['触发方式', 'Trigger policy'],
  'confirmedOption': ['持续确认后制动', 'Confirm sustained risk'],
  'instantOption': ['单帧触发并保持 · 旧版 AEB', 'Single-hit latch · legacy AEB'],
  'aeb_confirmation_s': ['危险持续确认', 'Risk confirmation time'],
  'followHint': ['介入前保留原有减速；实际开始制动后，由 AEB 沿原路径更新车速，替代原驾驶行为。', 'Retain original deceleration until braking starts; AEB then replaces driver speed control along the original path.'],
  'egoFollowHint': ['轨迹接管使用原始初速和航向，实际制动前保留原驾驶过程。', 'Trajectory takeover uses original initial conditions and driver motion until AEB braking.'],
  'assessmentTitle': ['制动与感知诊断', 'Braking and perception diagnostics'],
  'brakeAdvance': ['相对理想感知提前制动', 'Braking advance vs ideal range'],
  'brakeRange': ['开始制动时真实中心距离', 'True centre range at brake onset'],
  'rangeRmse': ['有效测距 RMSE', 'Valid range RMSE'],
  'confirmationCancelled': ['取消的待确认事件', 'Cancelled confirmation attempts'],
  'assessmentHelp': ['正值表示比独立理想感知对照更早制动，负值表示更晚；提前制动不直接等于误制动。RMSE 使用本次全部有效测距。', 'Positive advance means earlier braking than the independent ideal run; negative means later. Early braking is not automatically a false positive. RMSE uses all valid readings.'],
  'unmatchedBrake': ['本次制动，理想感知对照在时限内未制动；需结合场景判断原因。', 'This run braked while the ideal-range baseline did not within the horizon; interpret in context.'],
  'presetApplied': ['已应用团队原生 ACC 参数；保留当前误差模型和随机种子。', 'Native ACC settings applied; the current error model and seed are retained.'],
  'language': ['界面语言', 'Interface language'],
  'lightMode': ['白天模式', 'Day mode'],
  'darkMode': ['黑夜模式', 'Night mode'],
  'switchToLight': ['切换到白天模式', 'Switch to day mode'],
  'switchToDark': ['切换到黑夜模式', 'Switch to night mode'],
  'preview': ['实验预览', 'Experiment preview'],
  'ready': ['就绪', 'Ready'],
  'metrics': ['本次实验指标', 'Current experiment metrics'],
  'collision': ['碰撞结果', 'Collision outcome'],
  'impact': ['碰撞时自车速度', 'Ego speed at impact'],
  'braking': ['AEB 开始制动', 'AEB braking onset'],
  'nativeDeceleration': ['ACC 首次减速', 'ACC first deceleration'],
  'controlCsv': ['ACC 控制数据 CSV', 'ACC control CSV'],
  'metricsCsv': ['安全指标 CSV', 'Safety metrics CSV'],
  'history': ['历史记录', 'History'],
  'refresh': ['刷新', 'Refresh'],
  'refreshHistory': ['刷新历史记录', 'Refresh history'],
  'manageHistory': ['管理', 'Manage'],
  'finishHistory': ['完成', 'Done'],
  'selectAllHistory': ['全选', 'Select all'],
  'historySelected': ['已选 {count} 条', '{count} selected'],
  'selectHistoryRecord': ['选择记录：{title}，{id}', 'Select record: {title}, {id}'],
  'historyRunning': ['正在运行的实验无法删除', 'Running experiments cannot be deleted'],
  'deleteSelectedHistory': ['删除所选', 'Delete selected'],
  'deleteHistoryTitle': ['删除历史记录', 'Delete history records'],
  'deleteHistoryConfirm': ['确定删除所选的 {count} 条记录？删除后，它们将不再出现在历史列表中。', 'Delete the {count} selected records? They will no longer appear in history.'],
  'cancelDeleteHistory': ['取消', 'Cancel'],
  'confirmDeleteHistory': ['确认删除', 'Delete records'],
  'deletingHistory': ['正在删除…', 'Deleting…'],
  'historyDeleted': ['已删除 {count} 条记录', 'Deleted {count} records'],
  'historyDeletePlotWarning': ['记录已删除，但部分曲线更新失败；请重新生成受影响的结果图。', 'Records were deleted, but some plots could not be updated. Regenerate the affected plots.'],
  'historyDeleteActive': ['所选记录中有正在运行的实验，请等待完成或停止后再删除。', 'A selected experiment is running. Wait for it to finish or stop it before deleting.'],
  'historyDeleteInvalid': ['请选择 1 至 100 条有效记录。', 'Select between 1 and 100 valid records.'],
  'historyServiceOutdated': ['后台服务尚未更新，记录没有删除。请点击页面底部“停止后台服务”，再从桌面快捷方式启动并刷新页面。', 'The local service has not been updated. No records were deleted. Click “Stop local service” below, launch the desktop shortcut again, then reload this page.'],
  'runNew': ['运行新模拟', 'Run new simulation'],
  'overwriteRun': ['覆盖选中的记录', 'Overwrite selected record'],
  'overwritingRun': ['正在覆盖…', 'Overwriting…'],
  'selectHistoryFirst': ['请先选择一条历史记录', 'Select a history record first'],
  'overwriteTarget': ['覆盖当前记录：{id}', 'Overwrite the selected record: {id}'],
  'overwriteServiceOutdated': ['覆盖功能需要更新后台。请先停止后台服务，再从桌面快捷方式启动并刷新页面。', 'Overwriting requires an updated local service. Stop the service, launch the desktop shortcut again, and reload this page.'],
  'historyConfigFailed': ['无法载入这条记录的设置：{message}', 'Could not load this record’s settings: {message}'],
  'runConflict': ['已有实验运行中，请等待完成或停止。', 'An experiment is already running. Wait for it to finish or stop it.'],
  'overwriteActive': ['正在运行的记录不能覆盖。', 'A running record cannot be overwritten.'],
  'historyEmpty': ['运行后自动保存在这里', 'Experiments are saved here automatically.'],
  'historyScroll': ['历史实验，可滚动查看', 'Past experiments; scroll to browse'],
  'shutdown': ['停止后台服务', 'Stop local service'],
  'shutdownHint': ['关闭网页只关闭界面，后台服务和正在运行的实验仍会继续。此按钮停止后台服务；请先完成或停止实验。', 'Closing this page leaves the local service and any running experiment active. This button stops the service; finish or stop the experiment first.'],
  'results': ['视频与数据', 'Video and data'],
  'views': ['结果视图', 'Result views'],
  'video': ['场景视频', 'Scene video'],
  'speed': ['速度曲线', 'Speed curve'],
  'range': ['距离与误差', 'Range & error'],
  'videoLabel': ['本次实验场景视频', 'Current experiment video'],
  'plotLabel': ['本次实验的结果曲线', 'Current experiment plot'],
  'speedAlt': ['本次实验的自车速度曲线', 'Ego speed plot for this experiment'],
  'rangeAlt': ['本次实验的距离与误差曲线', 'Range and error plot for this experiment'],
  'emptyTitle': ['准备好，开始一次实验', 'Ready for an experiment'],
  'emptyDescription': ['在实验设置中调整参数，点击「运行新模拟」。', 'Adjust the experiment settings, then select Run new simulation.'],
  'openFolder': ['打开结果文件夹 ↗', 'Open results folder ↗'],
  'exportData': ['导出数据', 'Export data'],
  'egoCsv': ['自车 CSV ↓', 'Ego CSV ↓'],
  'obsCsv': ['观测 CSV ↓', 'Observations CSV ↓'],
  'truthCsv': ['真值 CSV ↓', 'Ground truth CSV ↓'],
  'summaryFile': ['结果摘要 ↓', 'Summary ↓'],
  'videoFile': ['MP4 视频 ↓', 'MP4 video ↓'],
  'resultHelp': ['结果说明', 'About results'],
  'resultHelpText': ['碰撞按真值包围盒重叠判定；「未碰撞」仅指记录时限内。原始回放保留两车轨迹，不模拟碰撞后物理响应。速度由位置估算；加速度以 0.30 秒窗口拟合位置估算，并非文件记录值，制动转折会略有平滑。', 'Collisions use ground-truth bounding boxes within the recorded time span. Original replay retains both trajectories without physical crash response. Speeds are derived from positions; acceleration is estimated with a 0.30 s position fit, not measured. Braking transitions are slightly smoothed.'],
  'settings': ['实验设置', 'Experiment settings'],
  'loadConfig': ['载入配置', 'Load config'],
  'saveConfig': ['保存配置', 'Save config'],
  'loadConfigShort': ['载入', 'Load'],
  'saveConfigShort': ['保存', 'Save'],
  'stopRun': ['停止实验', 'Stop experiment'],
  'run': ['运行实验', 'Run experiment'],
  'runningButton': ['实验运行中…', 'Running…'],
  'step1': ['选择场景', 'Choose a scene'],
  'scenario': ['数据场景', 'Data scenario'],
  'scenarioInfo': ['场景说明', 'Scenario details'],
  'fixedObjects': ['固定对象', 'Fixed targets'],
  'fixedHint': ['目标按基准轨迹回放，不随自车设置改变。', 'Targets replay their reference trajectories, independent of ego settings.'],
  'step2': ['设置自车', 'Ego vehicle'],
  'ego_speed_kph': ['初始速度', 'Initial speed'],
  'ego_heading_deg': ['初始航向', 'Initial heading'],
  'headingHint': ['初速由轨迹首段（0.01 秒）计算，航向读取首帧（世界坐标）。', 'Initial speed uses the first source segment (0.01 s); heading comes from the first frame in world coordinates.'],
  'egoReplayHint': ['原始回放使用文件初始状态；切换到“AEB · 恒速起步”后可调整。', 'Original replay uses the source initial state. Select AEB · constant-speed approach to edit it.'],
  'egoEditableHint': ['可调整初始状态；“AEB · 恒速起步”保持初始航向。', 'Initial state is editable. AEB · constant-speed approach holds this initial heading.'],
  'step3': ['感知条件', 'Perception'],
  'uncertainty': ['误差模型', 'Error model'],
  'idealOption': ['理想观测 · 无误差', 'Ideal · no range error'],
  'recordedOption': ['理想探测 · 不附加误差', 'Ideal detection · no added error'],
  'nativeSensor': ['esmini 理想传感器', 'esmini ideal sensor'],
  'sensorPipeline': ['真实位置 → OU 测距误差 → 控制器看到的位置 → HiDrive AEB', 'True position → OU range error → position seen by the controller → HiDrive AEB'],
  'distanceNoiseOption': ["时间相关测距误差 · OU", "Temporally correlated range error · OU"],
  'perceptionBoxHint': ['视频误差框已自动开启：黑框表示控制器看到的车辆位置，与目标车等长、等宽、同朝向。每步更新；测距无效时隐藏。', 'Video error outlines are enabled automatically: black boxes show the vehicle position seen by the controller, with the target’s length, width and heading. They update every step and disappear on invalid measurements.'],
  'ouNoiseShort': ["OU 时间相关误差", "OU correlated range error"],
  'noise_correlation_time_s': ["误差相关时间 τ", "Error correlation time τ"],
  'correlationTimeHelp': ["τ 越大，偏差延续越久。默认 0.5 秒为敏感性试验起点；论文依据见流程分析。", "Larger τ keeps errors correlated for longer. The default 0.5 s is a sensitivity-study starting point. See Flow analysis for sources."],
  'correlationPreview': ["当前步长 {dt} s：相邻误差相关系数 ρ ≈ {rho}。", "At the current {dt} s step: adjacent error correlation ρ ≈ {rho}."],
  'noise_zero_range_m': ['零误差距离', 'Zero-error distance'],
  'noise_reference_range_m': ['参考距离', 'Reference distance'],
  'noise_reference_mae_pct': ['参考处平均绝对误差', 'Mean absolute error at reference'],
  'distanceNoiseHint': ['≤ {zero} m 无误差；{reference} m 处平均绝对误差 {mae} m（σ ≈ {sigma} m），正负对称。', 'No error at ≤ {zero} m. At {reference} m: mean absolute error {mae} m (σ ≈ {sigma} m), symmetric about zero.'],
  'distanceNoiseInfo': ["OU 误差幅度说明", "OU error amplitude"],
  'distanceNoiseGrowthHint': ["距离指两车中心的水平距离。固定距离下的理论误差均值为 0；超过 {zero} m 后，每增加 1 m，MAE 增加约 {maeSlope} m、σ 增加约 {sigmaSlope} m，超过参考距离继续增长。这条距离规律是项目实验设定，尚未用实测数据标定。", "Range means horizontal bounding-box centre distance. At fixed range the theoretical error mean is zero. Beyond {zero} m, each extra metre increases MAE by {maeSlope} m and σ by {sigmaSlope} m; the slope continues beyond the reference. This distance law is an uncalibrated project assumption."],
  'noiseAnchorInvalid': ['误差参考距离必须大于零误差距离。', 'Reference distance must exceed the zero-error distance.'],
  'noiseParametersInvalid': ['请填写有效的零误差距离、参考距离和误差百分比。', 'Enter valid zero-error distance, reference distance and error percentage values.'],
  'sensor_range_m': ['感知范围', 'Sensor range'],
  'sensor_fov_deg': ['视场角', 'Field of view'],
  'seed': ['随机种子', 'Random seed'],
  'perceptionHelp': ['感知说明', 'Perception details'],
  'perceptionHelpText': ['控制器用自己的方式选择前车；Studio 不另加传感器量程或视场限制。误差只改变目标中心位置，速度、航向和尺寸保持真值。', 'The controller selects its lead vehicle its own way; Studio adds no separate sensor range or field-of-view limit. The error changes only the target centre position; speed, heading and size remain true.'],
  'perceptionNoiseHelpText': ["OU 过程为每个目标生成时间相关误差，有独立的随机流和持续状态，每步更新。τ 控制偏差延续时间，σ(r) 控制距离相关幅度。负测距保存在 CSV 并标为无效，该步目标被放到自车后方。公式、适用边界与本地论文见流程分析中的测距误差模块。", "An OU process generates correlated errors with an independent stream and persistent state for each object, updated every step. τ controls temporal persistence and σ(r) controls range-dependent amplitude. Negative ranges are logged as invalid, and for that step the target is placed behind the ego. See the range-error flow module for equations, limitations and local papers."],
  'perceptionProcessingHelpText': ['原始轨迹回放中，感知结果仅用于分析。当前理想传感器不模拟遮挡、随机漏检或速度误差。', 'In original trajectory replay, perception is used only for analysis. The current ideal sensor does not simulate occlusion, random missed detections or velocity error.'],
  'step4': ['控制策略', 'Control strategy'],
  'controller': ['控制器', 'Controller'],
  'replayOption': ['原始轨迹回放', 'Original trajectory replay'],
  'replayHint': ['两辆车按原始轨迹回放，保留原有减速和方向变化；感知设置仅影响观测曲线。', 'Both vehicles follow their source trajectories, including deceleration and heading changes. Perception settings affect observation plots only.'],
  'aebOption': ['旧版 Python AEB · 恒速起步', 'Legacy Python AEB · constant-speed approach'],
  'constantOption': ['无制动 · 恒速对照', 'No braking · constant speed'],
  'ttc_threshold_s': ['触发阈值', 'Trigger threshold'],
  'brake_decel_mps2': ['减速度', 'Deceleration'],
  'reaction_delay_s': ['响应延迟', 'Response delay'],
  'constantHint': ['保持初始速度，用于对照制动效果。', 'Hold the initial speed as a baseline for comparison.'],
  'modelHelp': ['模型说明', 'Model details'],
  'modelHelpText': ['老师提供的控制器源码按要求保密，Studio 只把它（已由我们移植到 3.8.1 接口）编译进 esmini 3.8.1，并从日志读取 AEB_active 与加速度，用于统计首次触发和首次制动。', 'The teacher’s controller source is confidential. Studio compiles it (ported by us to the 3.8.1 API) into esmini 3.8.1 and reads AEB_active and acceleration from the log to find the first trigger and first braking.'],
  'step5': ['运行输出', 'Run & output'],
  'duration_s': ['数据记录时长', 'Recorded duration'],
  'dt_s': ['时间步长', 'Time step'],
  'exportVideo': ['导出 MP4 视频', 'Export MP4 video'],
  'outputHint': ['按原场景停止条件结束，首次碰撞即结束；曲线和 CSV 自动保存。', 'Runs to the original stop condition or first contact. Plots and CSV files are saved automatically.'],
  'status.running': ['运行中', 'Running'],
  'status.starting': ['准备中', 'Preparing'],
  'status.completed': ['已完成', 'Completed'],
  'status.completed_with_warnings': ['完成 · 有提示', 'Completed · see note'],
  'status.failed': ['失败', 'Failed'],
  'status.cancelled': ['已停止', 'Stopped'],
  'status.interrupted': ['已中断', 'Interrupted'],
  'stage.simulation': ['闭环仿真中', 'Running simulation'],
  'stage.ideal_reference': ['计算独立理想对照', 'Running independent ideal reference'],
  'stage.plots': ['生成曲线', 'Generating plots'],
  'stage.video': ['导出场景视频', 'Exporting video'],
  'stage.starting': ['准备实验', 'Preparing experiment'],
  'stage.done': ['已完成', 'Completed'],
  'hit': ['发生碰撞', 'Collision'],
  'safe': ['未碰撞', 'No collision'],
  'notTriggered': ['未触发', 'Not triggered'],
  'constantShort': ['恒速', 'Constant speed'],
  'constantBaseline': ['恒速对照', 'Constant speed'],
  'idealShort': ['理想观测', 'Ideal'],
  'recordedShort': ['原始数据', 'Original data'],
  'replayShort': ['原始回放', 'Original replay'],
  'notApplicable': ['不适用', 'Not applicable'],
  'category.recorded': ['老师提供的场景', 'Teacher-provided scenes'],
  'gidas_1549178': ['GIDAS 1549178 · 追尾场景', 'GIDAS 1549178 · Rear-end'],
  'gidas_1554254': ['GIDAS 1554254 · 追尾场景', 'GIDAS 1554254 · Rear-end'],
  'gidas_1554431': ['GIDAS 1554431 · 追尾场景', 'GIDAS 1554431 · Rear-end'],
  'description.gidas_1549178': ['原始双车追尾轨迹；object_1 为自车，object_2 为前车。', 'Original rear-end trajectories; object_1 is ego and object_2 is the lead vehicle.'],
  'description.gidas_1554254': ['原始双车追尾轨迹；object_1 为自车，object_2 为前车。', 'Original rear-end trajectories; object_1 is ego and object_2 is the lead vehicle.'],
  'description.gidas_1554431': ['原始双车追尾轨迹，包含轻微方向变化；object_1 为自车。', 'Original rear-end trajectories with small heading changes; object_1 is ego.'],
  'originalRoad': ['原始道路', 'Source road'],
  'unknownScene': ['该配置的场景已不在当前三个数据场景中。', 'This configuration does not use one of the three current data scenes.'],
  'noiseShort': ["独立高斯误差 · 历史", "Independent Gaussian · historical"],
  'distanceNoiseShort': ["独立距离高斯误差 · 历史", "Independent range Gaussian · historical"],
  'ccrs': ['CCRs · 静止前车', 'CCRs · Stationary vehicle'],
  'cpna': ['CPNA · 行人横穿', 'CPNA · Pedestrian crossing'],
  'cccscp': ['CCCscp · 路口交叉来车', 'CCCscp · Crossing traffic'],
  'cpfa': ['CPFA · 行人远侧横穿', 'CPFA · Farside pedestrian crossing'],
  'cpla': ['CPLA · 行人沿道路行走', 'CPLA · Pedestrian walking along road'],
  'cbna': ['CBNA · 自行车近侧横穿', 'CBNA · Nearside cyclist crossing'],
  'cbla': ['CBLA · 自行车沿道路骑行', 'CBLA · Cyclist riding along road'],
  'cmrs': ['CMRs · 静止摩托车', 'CMRs · Stationary motorcycle'],
  'cmrb': ['CMRb · 前方摩托车制动', 'CMRb · Braking motorcycle ahead'],
  'ccrm': ['CCRm · 移动前车', 'CCRm · Moving vehicle ahead'],
  'ccrb': ['CCRb · 前车制动', 'CCRb · Braking vehicle ahead'],
  'description.ccrs': ['自车接近静止前车。', 'The ego vehicle approaches a stationary vehicle.'],
  'description.cpna': ['成年行人从近侧横穿道路。', 'An adult pedestrian crosses from the near side.'],
  'description.cccscp': ['自车直行通过交叉路口。', 'The ego vehicle drives straight through an intersection.'],
  'description.cpfa': ['成年行人从远侧横穿；采用原版 50 km/h 参数集。', 'An adult pedestrian crosses from the far side, using the original 50 km/h parameter set.'],
  'description.cpla': ['成年行人沿道路同向行走，按原版时序起步至 5 km/h。', 'An adult pedestrian walks along the road in the same direction, accelerating to 5 km/h on the original schedule.'],
  'description.cbna': ['骑行者在路口从近侧横穿自车路径，保留原版停放车辆及 50 km/h 参数集。', 'A cyclist crosses the ego path from the near side at an intersection, retaining the original parked vehicles and 50 km/h parameter set.'],
  'description.cbla': ['自行车沿道路同向骑行，按原版时序起步至 15 km/h。', 'A cyclist rides along the road in the same direction, accelerating to 15 km/h on the original schedule.'],
  'description.cmrs': ['自车接近前方静止摩托车，保留原版类型、尺寸和位置。', 'The ego vehicle approaches a stationary motorcycle with its original type, dimensions and position.'],
  'description.cmrb': ['双方以 50 km/h 起步，前方摩托车按原版时序制动。', 'Both start at 50 km/h; the motorcycle ahead brakes on the original schedule.'],
  'description.ccrm': ['自车以 50 km/h 接近前方以 20 km/h 同向行驶的车辆。', 'The ego vehicle starts at 50 km/h, approaching a vehicle travelling at 20 km/h in the same direction.'],
  'description.ccrb': ['双方以 50 km/h 起步，前车按原版时序制动。', 'Both start at 50 km/h; the vehicle ahead brakes on the original schedule.'],
  'category.car': ['汽车', 'Cars'],
  'category.pedestrian': ['行人', 'Pedestrians'],
  'category.bicycle': ['自行车', 'Cyclists'],
  'category.motorbike': ['摩托车', 'Motorcycles'],
  'straightRoad': ['直路', 'Straight road'],
  'intersection': ['十字路口', 'Intersection'],
  'pedestrian': ['行人', 'Pedestrian'],
  'vehicle': ['车辆', 'Vehicle'],
  'bicycle': ['自行车', 'Bicycle'],
  'motorbike': ['摩托车', 'Motorcycle'],
  'experimentRunning': ['实验进行中', 'Experiment in progress'],
  'autoResults': ['完成后会自动显示结果。', 'Results will appear automatically when ready.'],
  'experimentState': ['实验状态：{status}', 'Experiment {status}'],
  'incomplete': ['未完成', 'incomplete'],
  'retryHint': ['调整实验设置后可重新运行。', 'Adjust the experiment settings and run again.'],
  'noVideo': ['本次未导出视频', 'Video was not requested'],
  'videoUnavailable': ['视频暂不可用', 'Video unavailable'],
  'videoFailedHint': ['视频导出失败，可查看曲线或打开结果文件夹。', 'Video export failed. View the plots or open the results folder.'],
  'switchPlotHint': ['切换到「速度曲线」或「距离与误差」查看数据。', 'Select Speed curve or Range & error to view the data.'],
  'plotUnavailable': ['本次曲线不可用', 'Plot unavailable'],
  'openDataHint': ['打开结果文件夹查看已保存的数据。', 'Open the results folder to view the saved data.'],
  'configSaved': ['配置已保存', 'Configuration saved'],
  'configLoaded': ['配置已载入', 'Configuration loaded'],
  'loadFailed': ['载入失败：{message}', 'Could not load configuration: {message}'],
  'runStopped': ['已停止实验', 'Experiment stopped'],
  'serviceStopped': ['服务已停止', 'Service stopped'],
  'shutdownDone': ['后台服务已停止。可关闭网页；下次双击桌面快捷方式重新打开。', 'The local service has stopped. You can close this page and relaunch from the desktop shortcut.'],
  'connectionFailed': ['无法连接本地程序：{message}', 'Cannot connect to the local service: {message}'],
  'requestFailed': ['请求失败', 'Request failed'],
  'objectRequired': ['配置必须是 JSON 对象', 'Configuration must be a JSON object'],
  'objectRequiredServer': ['配置必须是 JSON 对象。', 'Configuration must be a JSON object.'],
  'unauthorized': ['请求未授权；请刷新本地页面。', 'Request not authorized; refresh this page.'],
  'alreadyRunning': ['已有实验运行中，请等待完成或停止。', 'An experiment is already running. Wait for it to finish or stop it.'],
  'stopFirst': ['请先停止正在运行的实验。', 'Stop the running experiment first.'],
  'notRunning': ['该实验没有运行', 'This experiment is not running'],
  'runIdInvalid': ['无效运行编号', 'Invalid run ID'],
  'runMissing': ['运行记录不存在', 'Run not found'],
  'pathInvalid': ['无效文件路径', 'Invalid file path'],
  'filePending': ['文件尚未生成', 'The file has not been generated yet'],
  'requestInvalid': ['请求过大或为空', 'The request is too large or empty'],
  'cancelledWarning': ['运行已停止；输出可能不完整。', 'The run was stopped; output may be incomplete.'],
  'interruptedWarning': ['上一次应用运行被中断，请重新运行以获取完整结果。', 'The previous session was interrupted. Run again to obtain complete results.'],
  'videoWarning': ['仿真与数据已完成，但视频导出失败；详见 video.log。', 'Simulation and data are complete, but video export failed. See video.log.'],
  'stepInvalid': ['时间步长必须为 0.01、0.02 或 0.05 秒。', 'Time step must be 0.01, 0.02 or 0.05 seconds.'],
  'seedInvalid': ['随机种子必须为 0–2147483647 的整数。', 'Random seed must be an integer from 0 to 2147483647.'],
  'videoInvalid': ['export_video 必须是布尔值。', 'Export video must be a boolean.']
};

function t(key, values = {}) {
  const text = messages[key]?.[language === 'en' ? 1 : 0] ?? key;
  return text.replace(/\{(\w+)\}/g, (match, name) => values[name] ?? match);
}

function localizeMessage(message = '') {
  message = String(message ?? '');
  if (language !== 'en') return message;
  const known = Object.values(messages).find(pair => pair[0] === message);
  if (known) return known[1];
  let match;
  if ((match = message.match(/^(.+?): 范围为 (.+)$/))) return `${t(match[1])}: allowed range ${match[2]}`;
  if ((match = message.match(/^(.+?): 必须是数值$/))) return `${t(match[1])}: enter a number`;
  if ((match = message.match(/^(.+?): 无效选项$/))) return `${t(match[1])}: invalid option`;
  if ((match = message.match(/^不支持的配置项(?:（目标对象数据不可修改）)?[：:]\s*(.+)$/))) return `Unsupported configuration fields (target data is read-only): ${match[1]}`;
  if ((match = message.match(/^运行进程异常退出 \((.+)\)；请查看 worker.log。$/))) return `The worker exited unexpectedly (${match[1]}). See worker.log.`;
  if ((match = message.match(/^固定对象轨迹偏差超过容差: (.+)$/))) return `Fixed-target trajectory error exceeds tolerance: ${match[1]}`;
  return message;
}

function statusName(status) { return messages[`status.${status}`] ? t(`status.${status}`) : status; }
function scenarioName(id, fallback = '') { return messages[id] ? t(id) : fallback; }
function controllerName(value) { return value === 'native_acc' ? t('nativeAccOption') : value === 'hidrive' ? t('hidriveOption') : value === 'original_trajectory' ? t('replayShort') : value === 'aeb_follow' ? t('aebFollowOption') : value === 'aeb' ? t('aebOption') : t('constantShort'); }
function perceptionName(value, result) {
  const native = result?.sensor?.backend === 'esmini_ideal_object_sensor';
  const label = t(value === 'recorded_truth' ? (native ? 'idealShort' : 'recordedShort') : value === 'distance_ou_range' ? 'ouNoiseShort' : value === 'distance_gaussian_range' ? 'distanceNoiseShort' : value === 'gaussian_range' ? 'noiseShort' : 'idealShort');
  return native ? `${t('nativeSensor')} · ${label}` : label;
}
function processingName(config) {
  const exact = ['recorded_truth','ideal'].includes(config.uncertainty) ||
    (['distance_ou_range','distance_gaussian_range'].includes(config.uncertainty) && Number(config.noise_reference_mae_pct) === 0) ||
    (config.uncertainty === 'gaussian_range' && Number(config.range_sigma_m) === 0 && Number(config.range_bias_m) === 0);
  return t(config.observation_filter === 'kalman' ? 'legacyFiltered' : exact ? 'exactProcessing' : 'rawProcessing');
}
function visibleResultNote(result) {
  if (result.error) return result.error;
  // Hide the specific source-road notice the user asked to remove; keep its
  // diagnostic and all other warnings available in the saved summary/logs.
  return String(result.warning || '').replaceAll('原始道路末段长度与 road length 不一致；保留老师道路并记录 esmini 提示。', '').replace(/^[；\s]+|[；\s]+$/g, '');
}
function setFormError(message = '', prefix = '') {
  formError = message ? {message, prefix} : null;
  $('formError').textContent = message ? (prefix ? t(prefix, {message:localizeMessage(message)}) : localizeMessage(message)) : '';
}

function updateThemeToggle() {
  const dark = document.documentElement.dataset.theme === 'dark';
  $('themeToggle').setAttribute('aria-checked', String(dark));
  $('themeToggle').title = t(dark ? 'switchToLight' : 'switchToDark');
  $('themeLabel').textContent = t(dark ? 'darkMode' : 'lightMode');
}

function setTheme(value) {
  const theme = value === 'dark' ? 'dark' : 'light';
  document.documentElement.dataset.theme = theme;
  try { localStorage.setItem('simulation-studio.theme', theme); } catch { /* Switching still works when storage is unavailable. */ }
  updateThemeToggle();
}

function setLanguage(value) {
  language = value === 'en' ? 'en' : 'zh';
  try { localStorage.setItem('simulation-studio.language', language); } catch { /* The switch also works without storage. */ }
  document.documentElement.lang = language === 'en' ? 'en' : 'zh-CN';
  document.querySelectorAll('[data-i18n]').forEach(element => element.textContent = t(element.dataset.i18n));
  for (const attribute of ['aria-label', 'title', 'alt']) {
    document.querySelectorAll(`[data-i18n-${attribute}]`).forEach(element => element.setAttribute(attribute, t(element.getAttribute(`data-i18n-${attribute}`))));
  }
  document.querySelectorAll('[data-language]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.language === language)));
  updateThemeToggle();
  if (boot) {
    for (const option of $('scenario').options) option.textContent = scenarioName(option.value, option.textContent);
    for (const group of $('scenario').querySelectorAll('optgroup')) group.label = t(`category.${group.dataset.category}`);
    updateScenario(false);
    renderHistory(historyRuns);
  }
  if (current) showResult(current);
  else renderView();
  setBusy(busy);
  if (serviceStopped) $('statusBadge').textContent = t('serviceStopped');
  if (formError) setFormError(formError.message, formError.prefix);
  clearTimeout(toastTimer);
  $('toast').hidden = true;
  refreshFlow();
  updateCompareControls();
  if (comparing) renderCompare();
}

const isComplete = s => ['completed', 'completed_with_warnings'].includes(s.status);
const isRunning = s => ['running', 'starting'].includes(s.status);

function toast(text) {
  clearTimeout(toastTimer);
  $('toast').textContent = localizeMessage(text);
  $('toast').hidden = false;
  toastTimer = setTimeout(() => $('toast').hidden = true, 3000);
}

async function api(path, data) {
  const options = data === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json', 'X-Studio-Token':boot.token}, body:JSON.stringify(data)};
  const response = await fetch(path, options);
  const result = await response.json();
  if (!response.ok) throw Object.assign(Error(result.error || messages.requestFailed[0]), {status:response.status, apiPath:path});
  return result;
}

const retiredConfigKeys = new Set(['observation_filter','kalman_noise_mode','kalman_range_sigma_m','kalman_accel_sigma_mps2','kalman_warmup_s','range_bias_m','range_sigma_m',
  'sensor_range_m','sensor_fov_deg','acc_time_gap_s','ttc_threshold_s','brake_decel_mps2','reaction_delay_s','observation_period_s','aeb_trigger_mode','aeb_confirmation_s']);

function getConfig() {
  const config = {};
  for (const [key, value] of Object.entries(boot.default)) {
    if (retiredConfigKeys.has(key)) continue;
    config[key] = typeof value === 'boolean' ? $(key).checked : typeof value === 'number' ? Number($(key).value) : $(key).value;
  }
  return config;
}

function setConfig(config) {
  config = {...boot.default, ...Object.fromEntries(Object.entries(config).filter(([key]) => !retiredConfigKeys.has(key)))};
  if (!boot.scenarios.some(s => s.id === config.scenario)) throw Error(messages.unknownScene[0]);
  if (['gaussian_range','distance_gaussian_range'].includes(config.uncertainty)) config = {...config, uncertainty:'distance_ou_range', noise_correlation_time_s:boot.default.noise_correlation_time_s};
  if ((config.controller || 'hidrive') !== 'hidrive') throw Error(messages.removedControllerConfig[0]);
  if (config.uncertainty === 'ideal') config = {...config, uncertainty:'recorded_truth'};
  const scene = boot.scenarios.find(s => s.id === config.scenario);
  config = {...config, ego_speed_kph:config.ego_speed_kph ?? scene.speed, ego_heading_deg:config.ego_heading_deg ?? scene.heading_deg};
  for (const key of Object.keys(boot.default)) {
    if (retiredConfigKeys.has(key)) continue;
    if (key in config) {
      if (typeof boot.default[key] === 'boolean') $(key).checked = config[key];
      else $(key).value = config[key];
    }
  }
  updateControls();
  updateScenario(false);
  refreshFlow();
}

function toggleFields(keys, enabled) {
  for (const key of keys) {
    const input = $(key);
    // A hidden, inactive field must not block the next experiment.
    input.disabled = false;
    if (!enabled && !input.checkValidity()) input.value = boot.default[key];
    input.disabled = !enabled;
  }
}

function updateControls() {
  const noisy = $('uncertainty').value === 'distance_ou_range';
  toggleFields(['noise_zero_range_m','noise_reference_range_m','noise_reference_mae_pct','noise_correlation_time_s','seed'], noisy);
  $('distanceNoiseFields').hidden = !noisy;
  $('perceptionBoxHint').hidden = !noisy;
  // The HiDrive controller drives the ego from the scene's own initial state.
  $('ego_speed_kph').readOnly = true;
  $('ego_heading_deg').readOnly = true;
  $('egoModeHint').textContent = t('egoHidriveHint');
  fillInitialState();
  updateDuration();
  updateNoiseProfile();
  updateStepSummaries();
}

// One-line summaries let collapsed sections still show their current values.
function updateStepSummaries() {
  if (!boot) return;
  const config = getConfig();
  const scene = boot.scenarios.find(s => s.id === config.scenario);
  $('sceneSummary').textContent = scenarioName(config.scenario, scene?.title || '');
  $('egoSummary').textContent = `${fmt(config.ego_speed_kph)} km/h · ${fmt(config.ego_heading_deg)}°${$('ego_speed_kph').readOnly ? ` · ${t('fromScene')}` : ''}`;
  $('perceptionSummary').textContent = perceptionSummary(config);
  $('controllerSummary').textContent = controllerSummary(config);
  $('outputSummary').textContent = `${fmt(config.duration_s)} s · dt ${plain(config.dt_s)}${config.export_video ? ' · MP4' : ` · ${t('noVideoShort')}`}`;
}

function savedConfig(config) {
  const scene = boot.scenarios.find(s => s.id === (config.scenario || boot.default.scenario));
  return {...boot.default, ego_speed_kph:scene?.speed, ego_heading_deg:scene?.heading_deg, ...config};
}

function updateNoiseProfile() {
  const zero = Number($('noise_zero_range_m').value);
  const reference = Number($('noise_reference_range_m').value);
  const pct = Number($('noise_reference_mae_pct').value);
  const enabled = $('uncertainty').value === 'distance_ou_range';
  const dt = Number($('dt_s').value);
  const tau = Number($('noise_correlation_time_s').value);
  $('correlationPreview').textContent = tau > 0 ? t('correlationPreview', {dt, rho:Math.exp(-dt/tau).toFixed(4)}) : '';
  $('distanceNoiseInfoButton').hidden = !enabled;
  if (!enabled && $('distanceNoiseInfo').matches(':popover-open')) $('distanceNoiseInfo').hidePopover();
  $('noise_reference_range_m').setCustomValidity(enabled && reference <= zero ? t('noiseAnchorInvalid') : '');
  const valid = ['noise_zero_range_m','noise_reference_range_m','noise_reference_mae_pct'].every(id => $(id).validity.valid) && reference > zero;
  $('distanceNoiseGrowthHint').hidden = !valid;
  if (!valid) {
    $('distanceNoiseHint').textContent = t(reference <= zero ? 'noiseAnchorInvalid' : 'noiseParametersInvalid');
    positionInfoPopover('distanceNoiseInfo');
    return;
  }
  const mae = reference*pct/100;
  $('distanceNoiseHint').textContent = t('distanceNoiseHint', {zero,reference,mae:mae.toFixed(2),sigma:(mae*Math.sqrt(Math.PI/2)).toFixed(2)});
  const maeSlope = mae/(reference-zero);
  $('distanceNoiseGrowthHint').textContent = t('distanceNoiseGrowthHint', {zero,reference,maeSlope:maeSlope.toFixed(3),sigmaSlope:(maeSlope*Math.sqrt(Math.PI/2)).toFixed(3)});
  positionInfoPopover('distanceNoiseInfo');
}

function fillInitialState() {
  const scenario = boot.scenarios.find(s => s.id === $('scenario').value);
  if (!scenario) return;
  $('ego_speed_kph').value = scenario.speed.toFixed(2);
  $('ego_heading_deg').value = scenario.heading_deg == null ? '' : scenario.heading_deg.toFixed(2);
  $('ego_heading_deg').required = scenario.heading_deg != null;
}

function updateDuration() {
  const scenario = boot?.scenarios.find(s => s.id === $('scenario').value);
  if (!scenario) return;
  const dt = Number($('dt_s').value);
  const duration = scenario.timing_policy === 'team_source_stop_trigger'
    ? (Math.floor(Number((scenario.stop_trigger_s / dt).toFixed(8))) + 1) * dt
    : scenario.duration_s;
  $('duration_s').value = duration.toFixed(2);
}

function updateScenario(resetSpeed) {
  const scenario = boot.scenarios.find(s => s.id === $('scenario').value);
  if (!scenario) return;
  updateDuration();
  $('scenarioDescription').textContent = messages[`description.${scenario.id}`] ? t(`description.${scenario.id}`) : scenario.description;
  $('roadLabel').textContent = scenario.road === '直路' ? t('straightRoad') : scenario.road === '十字路口' ? t('intersection') : scenario.road === '原始道路' ? t('originalRoad') : scenario.road;
  $('fixedObjects').replaceChildren();
  for (const object of scenario.fixed_objects) {
    const row = document.createElement('div');
    const typeKey = {'行人':'pedestrian','车辆':'vehicle','自行车':'bicycle','摩托车':'motorbike'}[object.type];
    const type = typeKey ? t(typeKey) : object.type;
    row.textContent = `${object.name} · ${type} · ${object.initial_speed_kph} km/h`;
    $('fixedObjects').appendChild(row);
  }
  if (resetSpeed) fillInitialState();
  updateControls();
  if (historyFilter === 'scene') renderHistory(historyRuns);
}

function setBusy(isBusy) {
  busy = isBusy;
  const blocked = busy || deletingHistory || loadingRun || !initialized || serviceStopped;
  $('runButton').disabled = blocked;
  $('runButton').textContent = t(busy && runningAction === 'new' ? 'runningButton' : 'runNew');
  const overwriteSupported = boot?.capabilities?.overwrite_run === true;
  const overwriteReady = configRun === selectedRun && current?.run_id === selectedRun && !isRunning(current);
  $('overwriteButton').disabled = blocked || !selectedRun || !overwriteReady || !overwriteSupported;
  $('overwriteButton').textContent = t(busy && runningAction === 'overwrite' ? 'overwritingRun' : 'overwriteRun');
  $('overwriteButton').title = !overwriteSupported && boot ? t('overwriteServiceOutdated') : selectedRun ? t('overwriteTarget', {id:selectedRun}) : t('selectHistoryFirst');
  $('runServiceNotice').hidden = !boot || overwriteSupported || serviceStopped;
  $('cancelButton').hidden = !busy;
  $('cancelButton').disabled = !activeRun;
  $('shutdown').disabled = serviceStopped || deletingHistory;
  updateHistoryControls();
}

function videoFile(result = current) {
  return language === 'en' && result?.files?.includes('video.en.mp4') ? 'video.en.mp4' : 'video.mp4';
}

function renderView() {
  const files = current?.files || [];
  const base = selectedRun ? `/runs/${selectedRun}/` : '';
  const wanted = tab === 'video' ? videoFile() : `${tab}.png`;
  const video = $('videoView');
  video.hidden = true;
  $('plotView').hidden = true;
  $('emptyView').hidden = true;
  if (files.includes(wanted) && (tab !== 'video' || current.video_status === 'ready')) {
    if (tab === 'video') {
      if (video.getAttribute('src') !== base + wanted) {
        const sameRun = video.getAttribute('src')?.startsWith(base);
        const time = sameRun ? video.currentTime : 0;
        const resume = sameRun && !video.paused;
        const rate = video.playbackRate;
        video.onloadedmetadata = () => {
          video.currentTime = Math.min(time, video.duration || time);
          video.playbackRate = rate;
          if (resume) video.play().catch(() => {});
        };
        video.src = base + wanted;
        video.load();
      }
      video.hidden = false;
    } else {
      video.pause();
      $('plotView').src = base + wanted + (plotRevision ? `?v=${plotRevision}` : '');
      $('plotView').alt = t(tab === 'speed' ? 'speedAlt' : 'rangeAlt');
      $('plotView').hidden = false;
    }
    return;
  }
  video.pause();
  $('emptyView').hidden = false;
  let title = t('emptyTitle');
  let description = t('emptyDescription');
  if (current) {
    if (isRunning(current)) {
      title = t('experimentRunning');
      description = t('autoResults');
    } else if (!isComplete(current)) {
      title = t('experimentState', {status:statusName(current.status) || t('incomplete')});
      description = localizeMessage(current.error || current.warning) || t('retryHint');
    } else if (tab === 'video') {
      title = t(current.video_status === 'not_requested' ? 'noVideo' : 'videoUnavailable');
      description = t(current.video_status === 'failed' ? 'videoFailedHint' : 'switchPlotHint');
    } else {
      title = t('plotUnavailable');
      description = t('openDataHint');
    }
  }
  $('emptyView').querySelector('h3').textContent = title;
  $('emptyView').querySelector('p').textContent = description;
}

const fmt = (value, digits = 2) => value == null || value === '' || !Number.isFinite(Number(value)) ? '—' : Number(value).toFixed(digits);
const plain = value => value == null || value === '' ? '—' : String(+value);
function formatDate(iso, full = false) {
  return new Date(iso).toLocaleString(language === 'en' ? 'en-GB' : 'zh-CN', full ? {hour12:false} : {month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',hour12:false});
}
function controllerSummary(config) {
  if (config.controller === 'native_acc') return `ACC · ${plain(config.acc_time_gap_s)} s`;
  if (config.controller === 'hidrive') return 'HiDrive AEB';
  if (config.controller === 'aeb' || config.controller === 'aeb_follow') return `AEB · TTC ${plain(config.ttc_threshold_s)} s`;
  return controllerName(config.controller);
}
function perceptionSummary(config) {
  if (config.uncertainty !== 'distance_ou_range') return t('idealShort');
  return `OU · τ ${plain(config.noise_correlation_time_s)} s · MAE ${plain(config.noise_reference_mae_pct)}%`;
}
function renderChips(result) {
  const config = result.config;
  const items = config ? [[controllerSummary(config)], [perceptionSummary(config), config.uncertainty === 'distance_ou_range' ? 'accent' : ''], [`${fmt(config.ego_speed_kph)} km/h`]] : [];
  if (config && result.created_utc) items.push([formatDate(result.created_utc), 'mono']);
  $('resultChips').replaceChildren(...items.map(([text, tone]) => {
    const chip = document.createElement('span');
    chip.className = 'chip' + (tone ? ` ${tone}` : '');
    chip.textContent = text;
    return chip;
  }));
}

function metric(id, value, unit) {
  $(id).textContent = value;
  if (unit) {
    const suffix = document.createElement('span');
    suffix.className = 'unit';
    suffix.textContent = ` ${unit}`;
    $(id).appendChild(suffix);
  }
}

function showResult(result) {
  current = result;
  const config = result.config || {};
  document.querySelector('[data-i18n="braking"]').textContent = t(config.controller === 'native_acc' ? 'nativeDeceleration' : 'braking');
  const scenario = boot.scenarios.find(s => s.id === config.scenario);
  const complete = isComplete(result);
  $('resultTitle').textContent = scenarioName(config.scenario, scenario?.title || result.run_id);
  $('statusBadge').textContent = statusName(result.status);
  $('statusBadge').className = 'status ' + (isRunning(result) ? 'running' : ['failed','cancelled','interrupted'].includes(result.status) ? 'bad' : '');
  $('collisionMetric').textContent = complete ? t(result.collision ? 'hit' : 'safe') : '—';
  $('collisionCard').className = 'metric collision-card' + (complete ? (result.collision ? ' hit' : ' safe') : '');
  metric('impactMetric', result.collision_details ? result.collision_details.ego_speed_kph.toFixed(1) : '—', result.collision_details ? 'km/h' : '');
  metric('brakeMetric', config.controller === 'original_trajectory' && complete ? t('notApplicable') : result.first_brake_time_s != null ? result.first_brake_time_s.toFixed(2) : complete ? t('notTriggered') : '—', result.first_brake_time_s != null ? 's' : '');
  metric('minGapMetric', complete && result.minimum_gap_m != null ? fmt(result.minimum_gap_m) : '—', complete && result.minimum_gap_m != null ? 'm' : '');
  renderChips(result);
  $('cancelledMetric').parentElement.hidden = config.controller === 'native_acc';
  const assessment = result.assessment;
  $('assessmentPanel').hidden = !complete || !assessment;
  if (assessment) {
    const value = (n,unit='') => n == null ? '—' : `${Number(n).toFixed(2)}${unit}`;
    $('advanceMetric').textContent = value(assessment.brake_advance_vs_ideal_s,' s');
    $('brakeRangeMetric').textContent = value(assessment.true_center_range_at_brake_m,' m');
    $('rmseMetric').textContent = value(assessment.observed_range_rmse_m,' m');
    $('cancelledMetric').textContent = assessment.cancelled_confirmations ?? '—';
    $('assessmentNote').textContent = (processingName(config) === t('rawProcessing') ? `${t('rawProcessingHelp')} ` : '') + t(assessment.braked_without_ideal_intervention ? 'unmatchedBrake' : 'assessmentHelp');
  }
  $('progressWrap').hidden = !isRunning(result);
  const progress = Math.round((result.progress?.fraction || 0) * 100);
  $('progressFill').style.width = `${progress}%`;
  $('progressText').textContent = `${t(messages[`stage.${result.progress?.stage}`] ? `stage.${result.progress.stage}` : 'stage.starting')} · ${progress}%`;
  const note = visibleResultNote(result);
  $('resultNote').className = 'result-note' + (note ? ' warning' : '');
  $('resultNote').textContent = localizeMessage(note) || (complete ? `${controllerName(config.controller)} · ${perceptionName(config.uncertainty,result)} · ${processingName(config)} · ${Number(config.ego_speed_kph).toFixed(2)} km/h · ${Number(result.end_time_s).toFixed(2)} s` : '');
  $('downloads').hidden = !complete;
  for (const [id, file] of [['csvLink','ego.csv'],['obsLink','observations.csv'],['truthLink','truth.csv'],['controlLink','control.csv'],['metricsLink','metrics.csv'],['summaryLink','summary.json'],['videoLink',videoFile(result)]]) {
    const link = $(id);
    link.href = `/runs/${result.run_id}/${file}`;
    link.download = file;
    link.hidden = !(result.files || []).includes(file) || (id === 'videoLink' && result.video_status !== 'ready');
  }
  renderView();
}

function highlightHistory() {
  document.querySelectorAll('.history-row').forEach(row => {
    const selected = row.dataset.id === selectedRun;
    row.classList.toggle('active', selected);
    (row.querySelector('.history-open') || row).setAttribute('aria-current', String(selected));
  });
}

function clearResult() {
  ++selectionRequest;
  loadingRun = false;
  configRun = null;
  selectedRun = null;
  current = null;
  const video = $('videoView');
  video.pause();
  video.removeAttribute('src');
  video.load();
  $('plotView').removeAttribute('src');
  $('resultTitle').textContent = t('preview');
  $('statusBadge').textContent = t(serviceStopped ? 'serviceStopped' : 'ready');
  $('statusBadge').className = 'status';
  $('collisionCard').className = 'metric collision-card';
  for (const id of ['collisionMetric', 'impactMetric', 'brakeMetric', 'minGapMetric']) $(id).textContent = '—';
  $('resultChips').replaceChildren();
  for (const id of ['downloads', 'assessmentPanel', 'progressWrap']) $(id).hidden = true;
  $('resultNote').textContent = '';
  $('resultNote').className = 'result-note';
  highlightHistory();
  renderView();
  setBusy(busy);
}

async function selectRun(id) {
  const request = ++selectionRequest;
  selectedRun = id;
  configRun = null;
  loadingRun = true;
  setBusy(busy);
  $('videoView').pause();
  highlightHistory();
  try {
    const result = await api(`/api/run/${id}`);
    if (request !== selectionRequest || selectedRun !== id) return;
    showResult(result);
    try {
      if (!result.config) throw Error(t('incomplete'));
      setConfig(savedConfig(result.config));
      configRun = id;
      setFormError();
    } catch (error) {
      setFormError(error.message, 'historyConfigFailed');
    }
  } catch (error) {
    if (request === selectionRequest) toast(error.message);
  } finally {
    if (request === selectionRequest) {
      loadingRun = false;
      setBusy(busy);
    }
  }
}

async function refreshHistory() {
  const request = ++historyRequest;
  const runs = await api('/api/runs');
  if (request !== historyRequest) return historyRuns;
  historyRuns = runs;
  const ongoing = runs.find(isRunning);
  if (ongoing) {
    activeRun = ongoing.run_id;
    setBusy(true);
    startPolling();
  }
  renderHistory(runs);
  return runs;
}

function canDeleteHistory(result) {
  return !isRunning(result) && result.run_id !== activeRun;
}

function updateHistoryControls() {
  const selectable = historyRuns.filter(canDeleteHistory);
  const valid = new Set(selectable.map(result => result.run_id));
  for (const id of checkedHistory) if (!valid.has(id)) checkedHistory.delete(id);
  $('manageHistory').textContent = t(managingHistory ? 'finishHistory' : 'manageHistory');
  $('manageHistory').setAttribute('aria-expanded', String(managingHistory));
  $('manageHistory').disabled = deletingHistory || serviceStopped || !initialized || (!historyRuns.length && !managingHistory);
  $('refreshHistory').disabled = deletingHistory || serviceStopped;
  $('historyToolbar').hidden = !managingHistory;
  $('historySelectionCount').textContent = t('historySelected', {count:checkedHistory.size});
  $('selectAllHistory').disabled = deletingHistory || serviceStopped || !selectable.length;
  $('selectAllHistory').checked = selectable.length > 0 && checkedHistory.size === selectable.length;
  $('selectAllHistory').indeterminate = checkedHistory.size > 0 && checkedHistory.size < selectable.length;
  $('deleteHistory').disabled = deletingHistory || serviceStopped || !checkedHistory.size;
  $('deleteHistory').textContent = t(deletingHistory ? 'deletingHistory' : 'deleteSelectedHistory');
  for (const checkbox of $('history').querySelectorAll('.history-checkbox')) {
    checkbox.checked = checkedHistory.has(checkbox.value);
    checkbox.disabled = deletingHistory || serviceStopped || !valid.has(checkbox.value);
    checkbox.closest('.history-row').classList.toggle('checked', checkbox.checked);
  }
}

let historyFilter = 'scene';
try { if (localStorage.getItem('simulation-studio.historyFilter') === 'all') historyFilter = 'all'; } catch { /* Storage may be unavailable. */ }
const compareIds = [];

function sceneLabel(result) {
  return scenarioName(result.config?.scenario, boot.scenarios.find(s => s.id === result.config?.scenario)?.title || result.run_id);
}

function renderHistory(runs) {
  const list = $('history');
  const scrollTop = list.scrollTop, scrollLeft = list.scrollLeft;
  // Drop comparison picks whose records were deleted or are no longer complete.
  let pruned = false;
  for (const id of [...compareIds]) {
    if (!runs.some(run => run.run_id === id && isComplete(run))) {
      compareIds.splice(compareIds.indexOf(id), 1);
      pruned = true;
    }
  }
  if (pruned && comparing) closeCompare();
  list.replaceChildren();
  const scene = $('scenario').value;
  const sceneRuns = runs.filter(run => run.config?.scenario === scene);
  const visible = historyFilter === 'scene' ? sceneRuns : runs;
  $('historyCount').textContent = runs.length;
  $('historySceneCount').textContent = sceneRuns.length;
  $('historyAllCount').textContent = runs.length;
  document.querySelectorAll('[data-history-filter]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.historyFilter === historyFilter)));
  $('historyEmpty').hidden = visible.length > 0;
  $('historyEmpty').textContent = t(runs.length ? 'historySceneEmpty' : 'historyEmpty');
  for (const result of visible) {
    const config = result.config || {};
    const row = document.createElement(managingHistory ? 'label' : 'div');
    row.className = 'history-row' + (managingHistory ? ' managing' : '');
    row.dataset.id = result.run_id;
    const body = document.createElement('span');
    const top = document.createElement('span');
    const title = document.createElement('b');
    const outcome = document.createElement('span');
    const meta = document.createElement('small');
    const date = document.createElement('time');
    body.className = 'history-body';
    top.className = 'history-top';
    title.textContent = result.config ? perceptionSummary(config) : result.run_id;
    outcome.className = 'outcome' + (isComplete(result) ? (result.collision ? ' hit' : '') : ' pending');
    outcome.textContent = isComplete(result) ? t(result.collision ? 'hit' : 'safe') : statusName(result.status);
    top.append(title, outcome);
    const metaParts = [controllerSummary(config), `${fmt(config.ego_speed_kph)} km/h`];
    if (config.observation_filter === 'kalman') metaParts.push(t('legacyFiltered'));
    meta.textContent = metaParts.join(' · ');
    if (result.created_utc) {
      date.dateTime = result.created_utc;
      date.textContent = formatDate(result.created_utc);
      row.title = `${result.run_id}\n${formatDate(result.created_utc, true)}`;
    }
    body.append(top);
    if (historyFilter === 'all') {
      const sceneLine = document.createElement('small');
      sceneLine.className = 'history-scene';
      sceneLine.textContent = sceneLabel(result);
      body.append(sceneLine);
    }
    const bottom = document.createElement('span');
    bottom.className = 'history-bottom';
    bottom.append(meta, date);
    body.append(bottom);
    const compareIndex = compareIds.indexOf(result.run_id);
    if (compareIndex >= 0 && !managingHistory) {
      const badge = document.createElement('span');
      badge.className = `compare-badge ${compareIndex ? 'b' : 'a'}`;
      badge.textContent = compareIndex ? 'B' : 'A';
      top.prepend(badge);
    }
    if (managingHistory) {
      const checkbox = document.createElement('input');
      checkbox.type = 'checkbox';
      checkbox.className = 'history-checkbox';
      checkbox.value = result.run_id;
      checkbox.setAttribute('aria-label', t('selectHistoryRecord', {title:`${sceneLabel(result)} · ${title.textContent}`, id:result.run_id}));
      if (!canDeleteHistory(result)) {
        row.classList.add('unavailable');
        row.title = t('historyRunning');
      }
      checkbox.onchange = () => {
        if (checkbox.checked) checkedHistory.add(result.run_id);
        else checkedHistory.delete(result.run_id);
        updateHistoryControls();
      };
      row.append(checkbox, body);
    } else {
      const compare = document.createElement('input');
      compare.type = 'checkbox';
      compare.className = 'compare-checkbox';
      compare.checked = compareIndex >= 0;
      compare.disabled = !isComplete(result);
      compare.title = isComplete(result) ? '' : t('compareNeedsComplete');
      compare.setAttribute('aria-label', t('compareCheckbox', {title:`${sceneLabel(result)} · ${title.textContent}`}));
      compare.onchange = () => toggleCompare(result.run_id, compare.checked);
      const open = document.createElement('button');
      open.type = 'button';
      open.className = 'history-open';
      open.append(body);
      open.onclick = () => {
        if (comparing) closeCompare();
        selectRun(result.run_id);
      };
      row.classList.toggle('compared', compareIndex >= 0);
      row.append(compare, open);
    }
    list.appendChild(row);
  }
  updateHistoryControls();
  updateCompareControls();
  highlightHistory();
  list.scrollTop = scrollTop;
  list.scrollLeft = scrollLeft;
}

function toggleCompare(id, checked) {
  const index = compareIds.indexOf(id);
  if (checked && index < 0) {
    compareIds.push(id);
    if (compareIds.length > 2) compareIds.shift();
  }
  if (!checked && index >= 0) compareIds.splice(index, 1);
  renderHistory(historyRuns);
  // Re-rendering replaces the row; keep keyboard focus on the same checkbox.
  $('history').querySelector(`.history-row[data-id="${CSS.escape(id)}"] .compare-checkbox`)?.focus();
  if (comparing) {
    if (compareIds.length === 2) openCompare();
    else closeCompare();
  }
}

function updateCompareControls() {
  const count = compareIds.length;
  $('compareButton').textContent = t('compareSelected', {count});
  $('compareButton').disabled = count !== 2 || serviceStopped;
  $('compareButton').classList.toggle('ready', count === 2);
  $('compareHint').textContent = t(count === 0 ? 'compareHint' : count === 1 ? 'compareHintOne' : 'compareHintReady');
}

function startPolling() {
  if (timer) return;
  timer = setInterval(async () => {
    if (!activeRun) { clearInterval(timer); timer = null; return; }
    const id = activeRun, request = runRequest;
    try {
      const result = await api(`/api/run/${id}`);
      if (id !== activeRun || request !== runRequest) return;
      if (selectedRun === activeRun) showResult(result);
      if (!isRunning(result)) {
        activeRun = null;
        setBusy(false);
        clearInterval(timer);
        timer = null;
        await refreshHistory();
        plotRevision = Date.now();
        if (current) renderView();
      }
    } catch (error) { /* A newly launched worker may not have created its directory yet. */ }
  }, 800);
}

async function startExperiment(overwrite = false) {
  if ($(overwrite ? 'overwriteButton' : 'runButton').disabled || !$('configForm').reportValidity()) return;
  const replaceId = overwrite ? selectedRun : null;
  if (comparing) closeCompare();
  setFormError();
  runningAction = overwrite ? 'overwrite' : 'new';
  ++selectionRequest;
  loadingRun = false;
  setBusy(true);
  try {
    const config = getConfig();
    if (overwrite) {
      $('videoView').pause();
      $('videoView').removeAttribute('src');
      $('videoView').load();
    }
    const result = await api(overwrite ? '/api/run/overwrite' : '/api/run', overwrite ? {run_id:replaceId, config} : config);
    ++selectionRequest;
    ++runRequest;
    ++historyRequest;
    loadingRun = false;
    activeRun = result.run_id;
    selectedRun = result.run_id;
    configRun = result.run_id;
    setConfig(result.config || config);
    plotRevision = Date.now();
    showResult({run_id:result.run_id, status:'starting', config:result.config || config, files:[], video_status:config.export_video ? 'pending' : 'not_requested'});
    setBusy(true);
    highlightHistory();
    startPolling();
    setTimeout(() => refreshHistory().catch(() => {}), 1000);
  } catch (error) {
    setFormError(overwrite && error.status === 404 ? messages.overwriteServiceOutdated[0] : error.message);
    setBusy(Boolean(activeRun));
    renderView();
  }
}
$('configForm').onsubmit = event => {
  event.preventDefault();
  startExperiment();
};
$('overwriteButton').onclick = () => startExperiment(true);

$('scenario').onchange = () => updateScenario(true);
$('uncertainty').onchange = updateControls;
for (const id of ['noise_zero_range_m','noise_reference_range_m','noise_reference_mae_pct','noise_correlation_time_s']) $(id).oninput = updateNoiseProfile;
$('dt_s').onchange = updateControls;
$('saveConfig').onclick = () => {
  const blob = new Blob([JSON.stringify(getConfig(), null, 2)], {type:'application/json'});
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = 'experiment.json';
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  toast(t('configSaved'));
};
$('loadConfig').onclick = () => $('configFile').click();
$('configFile').onchange = async event => {
  if (!event.target.files.length) return;
  try {
    const config = JSON.parse(await event.target.files[0].text());
    if (typeof config !== 'object' || Array.isArray(config) || !config) throw Error('配置必须是 JSON 对象');
    for (const key of Object.keys(config)) if (!(key in boot.default) && !retiredConfigKeys.has(key)) throw Error(`不支持的配置项：${key}`);
    setConfig(savedConfig(config));
    toast(t('configLoaded'));
  } catch (error) { toast(t('loadFailed', {message:localizeMessage(error.message)})); }
  event.target.value = '';
};
$('cancelButton').onclick = async () => {
  try { await api('/api/cancel', {run_id:activeRun}); toast(t('runStopped')); }
  catch (error) { toast(error.message); }
};
$('openFolder').onclick = async () => {
  try { await api('/api/open-folder', {run_id:selectedRun}); }
  catch (error) { toast(error.message); }
};
$('refreshHistory').onclick = () => refreshHistory().catch(error => toast(error.message));
$('manageHistory').onclick = () => {
  if ($('manageHistory').disabled) return;
  managingHistory = !managingHistory;
  checkedHistory.clear();
  renderHistory(historyRuns);
};
$('selectAllHistory').onchange = event => {
  checkedHistory.clear();
  if (event.target.checked) historyRuns.filter(canDeleteHistory).forEach(result => checkedHistory.add(result.run_id));
  updateHistoryControls();
};
$('deleteHistory').onclick = () => {
  updateHistoryControls();
  if ($('deleteHistory').disabled) return;
  pendingHistoryDeletion = [...checkedHistory];
  $('deleteHistoryDescription').textContent = t('deleteHistoryConfirm', {count:pendingHistoryDeletion.length});
  $('deleteHistoryDialog').showModal();
};
$('cancelDeleteHistory').onclick = () => $('deleteHistoryDialog').close();
$('confirmDeleteHistory').onclick = async () => {
  if (deletingHistory || !pendingHistoryDeletion.length) return;
  const ids = pendingHistoryDeletion;
  pendingHistoryDeletion = [];
  $('deleteHistoryDialog').close();
  deletingHistory = true;
  $('historyServiceNotice').hidden = true;
  ++historyRequest; // Discard any refresh that started before this deletion.
  setBusy(busy);
  try {
    // Release Windows' video file handle before moving a viewed result.
    if (ids.includes(selectedRun)) {
      $('videoView').pause();
      $('videoView').removeAttribute('src');
      $('videoView').load();
    }
    const result = await api('/api/runs/delete', {run_ids:ids});
    ++historyRequest;
    plotRevision = Date.now();
    const deleted = new Set(result.deleted);
    historyRuns = historyRuns.filter(run => !deleted.has(run.run_id));
    checkedHistory.clear();
    const removedCurrent = deleted.has(selectedRun);
    if (removedCurrent) clearResult();
    renderHistory(historyRuns);
    await refreshHistory();
    if (removedCurrent && historyRuns.length) await selectRun(activeRun || historyRuns[0].run_id);
    else if (current) renderView(); // Reload plots after their shared axes change.
    toast(result.warnings?.length ? t('historyDeletePlotWarning') : t('historyDeleted', {count:deleted.size}));
  } catch (error) {
    renderView();
    if (error.status === 404 && error.apiPath === '/api/runs/delete') $('historyServiceNotice').hidden = false;
    else toast(error.message);
  } finally {
    deletingHistory = false;
    if (!historyRuns.length) managingHistory = false;
    renderHistory(historyRuns);
    setBusy(busy);
    $('manageHistory').focus();
  }
};
$('shutdown').onclick = async () => {
  try {
    await api('/api/shutdown', {});
    serviceStopped = true;
    $('statusBadge').textContent = t('serviceStopped');
    setBusy(false);
    clearInterval(timer);
    toast(t('shutdownDone'));
  } catch (error) { toast(error.message); }
};

const tabs = [...document.querySelectorAll('[data-tab]')];
for (const button of tabs) {
  button.onclick = () => {
    tab = button.dataset.tab;
    for (const item of tabs) {
      const selected = item === button;
      item.classList.toggle('selected', selected);
      item.setAttribute('aria-selected', String(selected));
      item.tabIndex = selected ? 0 : -1;
    }
    $('resultViewer').setAttribute('aria-labelledby', button.id);
    renderView();
  };
  button.onkeydown = event => {
    if (event.ctrlKey || event.metaKey || event.altKey || !['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) return;
    event.preventDefault();
    const index = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length - 1 : (tabs.indexOf(button) + (event.key === 'ArrowRight' ? 1 : -1) + tabs.length) % tabs.length;
    tabs[index].focus();
    tabs[index].click();
  };
}
function positionInfoPopover(id) {
  const popup = $(id);
  if (!popup.matches(':popover-open')) return;
  const anchor = $(`${id}Button`).getBoundingClientRect();
  const bounds = popup.getBoundingClientRect();
  const inset = 16;
  popup.style.left = `${Math.max(inset, Math.min(anchor.right - bounds.width, innerWidth - bounds.width - inset))}px`;
  const below = anchor.bottom + 8;
  popup.style.top = `${below + bounds.height <= innerHeight - inset ? below : Math.max(inset, anchor.top - bounds.height - 8)}px`;
}

for (const id of ['scenarioInfo', 'distanceNoiseInfo']) {
  $(id).addEventListener('toggle', () => {
    $(`${id}Button`).setAttribute('aria-expanded', String($(id).matches(':popover-open')));
    positionInfoPopover(id);
  });
  const close = () => {
    if ($(id).matches(':popover-open')) $(id).hidePopover();
  };
  $('configForm').addEventListener('scroll', close, {passive:true});
  window.addEventListener('scroll', close, {passive:true});
  window.addEventListener('resize', () => positionInfoPopover(id));
}

document.addEventListener('click', event => {
  document.querySelectorAll('.result-actions details[open]').forEach(details => {
    if (!details.contains(event.target)) details.open = false;
  });
});
document.addEventListener('keydown', event => {
  if (event.key === 'Escape') document.querySelectorAll('.result-actions details[open]').forEach(details => {
    details.open = false;
    if (details.contains(document.activeElement)) details.querySelector('summary').focus();
  });
});

// Comparison view: two completed runs side by side, read from their saved summaries and CSV files.
const compareParamUnits = {ego_speed_kph:'km/h', ego_heading_deg:'°', noise_zero_range_m:'m', noise_reference_range_m:'m', noise_reference_mae_pct:'%', noise_correlation_time_s:'s', sensor_range_m:'m', sensor_fov_deg:'°', acc_time_gap_s:'s', ttc_threshold_s:'s', brake_decel_mps2:'m/s²', reaction_delay_s:'s', duration_s:'s', dt_s:'s', observation_period_s:'s', aeb_confirmation_s:'s'};
const noiseParams = new Set(['noise_zero_range_m','noise_reference_range_m','noise_reference_mae_pct','noise_correlation_time_s','seed']);
const aebParams = new Set(['ttc_threshold_s','brake_decel_mps2','reaction_delay_s','aeb_trigger_mode','aeb_confirmation_s']);
const csvCache = new Map();

function paramActive(config, key) {
  if (noiseParams.has(key)) return config.uncertainty === 'distance_ou_range';
  if (aebParams.has(key)) return ['aeb','aeb_follow'].includes(config.controller) && (key !== 'aeb_confirmation_s' || config.aeb_trigger_mode === 'confirmed');
  if (key === 'acc_time_gap_s') return config.controller === 'native_acc';
  if (['sensor_range_m','sensor_fov_deg','observation_period_s'].includes(key)) return config.controller !== 'hidrive';
  return true;
}

function paramText(config, key) {
  if (!paramActive(config, key)) return '—';
  const value = config[key];
  if (value == null) return '—';
  if (key === 'scenario') return scenarioName(value, value);
  if (key === 'uncertainty') return value === 'distance_ou_range' ? 'OU' : t('idealShort');
  if (key === 'controller') return controllerName(value);
  if (key === 'aeb_trigger_mode') return t(value === 'instant' ? 'instantOption' : 'confirmedOption');
  if (key === 'export_video') return value ? '✓' : '—';
  if (key === 'observation_period_s' && Number(value) === 0) return t('legacyClock');
  const text = ['ego_speed_kph','ego_heading_deg'].includes(key) ? fmt(value) : plain(value);
  return compareParamUnits[key] ? `${text} ${compareParamUnits[key]}` : text;
}

function compareMetrics(a, b) {
  const native = a.config?.controller === 'native_acc' && b.config?.controller === 'native_acc';
  return [
    [t('collision'), run => isComplete(run) ? t(run.collision ? 'hit' : 'safe') : '—', null],
    [t('impact'), run => run.collision_details?.ego_speed_kph, 'km/h'],
    [t(native ? 'nativeDeceleration' : 'braking'), run => run.first_brake_time_s, 's'],
    [t('minGap'), run => run.minimum_gap_m, 'm'],
    [t('minTtc'), run => run.minimum_ttc_s, 's'],
    [t('finalSpeed'), run => run.final_speed_kph, 'km/h'],
    [t('rangeRmse'), run => run.assessment?.observed_range_rmse_m, 'm'],
  ];
}

function cell(tag, text, className = '') {
  const element = document.createElement(tag);
  if (className) element.className = className;
  element.textContent = text;
  return element;
}

function runHeading(run, slot) {
  const wrap = document.createElement('span');
  wrap.className = 'compare-run-label';
  wrap.append(cell('span', slot.toUpperCase(), `compare-badge ${slot}`));
  const text = document.createElement('span');
  text.append(cell('b', perceptionSummary(run.config || {})), cell('small', [controllerSummary(run.config || {}), run.created_utc ? formatDate(run.created_utc) : ''].filter(Boolean).join(' · ')));
  wrap.append(text);
  return wrap;
}

function renderCompareTable(a, b) {
  const table = $('compareTable');
  table.replaceChildren();
  const head = document.createElement('thead');
  const headRow = document.createElement('tr');
  const thA = document.createElement('th'), thB = document.createElement('th');
  thA.append(runHeading(a, 'a'));
  thB.append(runHeading(b, 'b'));
  headRow.append(cell('th', ''), thA, thB, cell('th', 'B − A', 'numeric'));
  head.append(headRow);
  const body = document.createElement('tbody');
  const section = text => {
    const row = document.createElement('tr');
    row.className = 'compare-section';
    const th = cell('th', text);
    th.colSpan = 4;
    row.append(th);
    body.append(row);
  };
  section(t('paramsHeader'));
  const keys = Object.keys(boot.default).filter(key => !retiredConfigKeys.has(key));
  const differs = key => paramText(a.config || {}, key) !== paramText(b.config || {}, key);
  const changed = keys.filter(differs), same = keys.filter(key => !differs(key));
  const labelOf = key => t(key === 'export_video' ? 'exportVideo' : key === 'scenario' ? 'secScene' : key);
  for (const key of [...changed, ...(compareShowAll ? same : [])]) {
    const row = document.createElement('tr');
    if (differs(key)) row.className = 'changed';
    row.append(cell('td', labelOf(key)), cell('td', paramText(a.config || {}, key), 'mono'), cell('td', paramText(b.config || {}, key), 'mono'), cell('td', ''));
    body.append(row);
  }
  const toggleRow = document.createElement('tr');
  toggleRow.className = 'compare-toggle';
  const toggleCell = document.createElement('td');
  toggleCell.colSpan = 4;
  toggleCell.append(cell('span', changed.length ? t('sameParams', {count:same.length}) : t('noDiffParams')));
  if (same.length) {
    const toggle = cell('button', t(compareShowAll ? 'hideAll' : 'showAll'), 'link-button');
    toggle.type = 'button';
    toggle.setAttribute('aria-expanded', String(compareShowAll));
    toggle.onclick = () => { compareShowAll = !compareShowAll; renderCompareTable(a, b); };
    toggleCell.append(' · ', toggle);
  }
  toggleRow.append(toggleCell);
  body.append(toggleRow);
  section(t('resultsHeader'));
  for (const [label, read, unit] of compareMetrics(a, b)) {
    const row = document.createElement('tr');
    const va = read(a), vb = read(b);
    let textA, textB, delta = '';
    if (unit === null) {
      textA = va; textB = vb;
      delta = va === '—' || vb === '—' ? '' : va === vb ? t('sameValue') : '≠';
    } else {
      textA = va == null ? '—' : `${fmt(va)} ${unit}`;
      textB = vb == null ? '—' : `${fmt(vb)} ${unit}`;
      if (va != null && vb != null) {
        const diff = Number(vb) - Number(va);
        delta = Math.abs(diff) < 0.005 ? t('sameValue') : `${diff > 0 ? '+' : '−'}${Math.abs(diff).toFixed(2)} ${unit}`;
      }
    }
    if (delta && delta !== t('sameValue')) row.className = 'changed';
    const outcomeClass = value => unit === null && value !== '—' ? (value === t('hit') ? 'mono hit' : 'mono safe') : 'mono';
    row.append(cell('td', label, 'metric-name'), cell('td', textA, outcomeClass(textA)), cell('td', textB, outcomeClass(textB)), cell('td', delta, 'numeric delta'));
    body.append(row);
  }
  table.append(head, body);
}

function renderCompare() {
  if (!comparing || compareRuns.length !== 2) return;
  const [a, b] = compareRuns;
  const sameScene = a.config?.scenario === b.config?.scenario;
  $('compareSubtitle').textContent = sameScene ? sceneLabel(a) : `${sceneLabel(a)}  /  ${sceneLabel(b)}`;
  $('compareNote').textContent = sameScene ? '' : t('differentScene');
  $('compareNote').className = 'result-note' + (sameScene ? '' : ' warning');
  renderCompareTable(a, b);
  $('compareLegend').replaceChildren(...[a, b].map((run, index) => {
    const item = document.createElement('span');
    item.append(cell('i', '', index ? 'swatch b' : 'swatch a'), `${index ? 'B' : 'A'} · ${perceptionSummary(run.config || {})}`);
    return item;
  }));
  document.querySelectorAll('[data-compare-tab]').forEach(button => {
    const selected = button.dataset.compareTab === compareTab;
    button.classList.toggle('selected', selected);
    button.setAttribute('aria-selected', String(selected));
    button.tabIndex = selected ? 0 : -1;
  });
  renderCompareView();
}

async function openCompare() {
  if (compareIds.length !== 2) return;
  const request = ++compareRequest;
  comparing = true;
  $('videoView').pause();
  $('singleView').hidden = true;
  $('comparePanel').hidden = false;
  try {
    const runs = await Promise.all(compareIds.map(id => api(`/api/run/${id}`)));
    if (request !== compareRequest) return;
    compareRuns = runs;
    renderCompare();
  } catch (error) {
    if (request !== compareRequest) return;
    toast(t('compareLoadFailed', {message:localizeMessage(error.message)}));
    closeCompare();
  }
}

function stopCompareVideos() {
  $('compareViewer').querySelectorAll('video').forEach(video => {
    video.pause();
    video.removeAttribute('src');
    video.load();
  });
}

function closeCompare() {
  ++compareRequest;
  comparing = false;
  compareRuns = [];
  compareChart = null;
  stopCompareVideos();
  $('compareViewer').replaceChildren();
  $('comparePanel').hidden = true;
  $('singleView').hidden = false;
}

function splitCsvLine(line) {
  if (!line.includes('"')) return line.split(',');
  const values = [];
  let value = '', quoted = false;
  for (let i = 0; i < line.length; i++) {
    const char = line[i];
    if (quoted) {
      if (char === '"' && line[i + 1] === '"') { value += '"'; i++; }
      else if (char === '"') quoted = false;
      else value += char;
    } else if (char === '"') quoted = true;
    else if (char === ',') { values.push(value); value = ''; }
    else value += char;
  }
  values.push(value);
  return values;
}

async function loadCsv(runId, file) {
  const key = `${runId}/${file}/${plotRevision}`;
  if (!csvCache.has(key)) {
    csvCache.set(key, fetch(`/runs/${runId}/${file}`).then(response => {
      if (!response.ok) throw Error(t('chartUnavailable'));
      return response.text();
    }).then(text => {
      const lines = text.split(/\r?\n/).filter(Boolean);
      const header = splitCsvLine(lines.shift() || '');
      return lines.map(line => {
        const values = splitCsvLine(line);
        return Object.fromEntries(header.map((name, index) => [name, values[index]]));
      });
    }));
    csvCache.get(key).catch(() => csvCache.delete(key));
  }
  return csvCache.get(key);
}

const compareSpecs = {
  speed: {file:'ego.csv', axis:'speedAxis', points: rows => rows.map(row => [Number(row.time_s), Number(row.speed_kph)])},
  gap: {file:'metrics.csv', axis:'gapAxis', points: rows => rows.map(row => [Number(row.time_s), Number(row.minimum_gap_m)])},
  error: {file:'observations.csv', axis:'errorAxis', zeroLine:true, points: rows => {
    // Use the target with the most valid readings so parked objects do not interleave.
    const valid = rows.filter(row => row.observation_valid === 'True' && row.actual_range_error_m !== '');
    const counts = new Map();
    for (const row of valid) counts.set(row.object_id, (counts.get(row.object_id) || 0) + 1);
    const target = [...counts.entries()].sort((x, y) => y[1] - x[1])[0]?.[0];
    return valid.filter(row => row.object_id === target).map(row => [Number(row.time_s), Number(row.actual_range_error_m)]);
  }},
};

function chartMessage(text) {
  return cell('p', text, 'chart-message');
}

async function renderCompareView() {
  const viewer = $('compareViewer');
  stopCompareVideos();
  compareChart = null;
  const runs = compareRuns;
  if (compareTab === 'video') return renderCompareVideos(runs);
  const spec = compareSpecs[compareTab];
  const request = compareRequest, tabAtStart = compareTab;
  viewer.replaceChildren(chartMessage(t('chartLoading')));
  const series = await Promise.all(runs.map(async (run, index) => {
    if (!(run.files || []).includes(spec.file)) return null;
    try {
      const points = spec.points(await loadCsv(run.run_id, spec.file)).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
      return points.length ? {slot:index ? 'b' : 'a', points} : null;
    } catch { return null; }
  }));
  if (request !== compareRequest || tabAtStart !== compareTab) return;
  if (!series.some(Boolean)) {
    viewer.replaceChildren(chartMessage(t('chartUnavailable')));
    return;
  }
  compareChart = {series:series.filter(Boolean), spec, partial:series.some(item => !item)};
  drawCompareChart();
}

function niceStep(range, count) {
  const raw = range / Math.max(count, 1);
  const power = 10 ** Math.floor(Math.log10(raw));
  const unit = raw / power;
  return (unit <= 1 ? 1 : unit <= 2 ? 2 : unit <= 5 ? 5 : 10) * power;
}

function svgElement(name, attributes = {}) {
  const element = document.createElementNS('http://www.w3.org/2000/svg', name);
  for (const [key, value] of Object.entries(attributes)) element.setAttribute(key, value);
  return element;
}

function drawCompareChart() {
  const viewer = $('compareViewer');
  if (!compareChart || compareTab === 'video') return;
  const {series, spec, partial} = compareChart;
  const width = Math.max(320, viewer.clientWidth), height = 360;
  const margin = {left:58, right:18, top:18, bottom:46};
  const all = series.flatMap(item => item.points);
  let xMin = Math.min(...all.map(p => p[0])), xMax = Math.max(...all.map(p => p[0]));
  let yMin = Math.min(...all.map(p => p[1])), yMax = Math.max(...all.map(p => p[1]));
  if (spec.zeroLine) { yMin = Math.min(yMin, 0); yMax = Math.max(yMax, 0); }
  if (xMax === xMin) xMax = xMin + 1;
  if (yMax === yMin) { yMax += 1; yMin -= 1; }
  const yStep = niceStep(yMax - yMin, 5);
  yMin = Math.floor(yMin / yStep) * yStep;
  yMax = Math.ceil(yMax / yStep) * yStep;
  const xStep = niceStep(xMax - xMin, 8);
  const plotW = width - margin.left - margin.right, plotH = height - margin.top - margin.bottom;
  const sx = x => margin.left + (x - xMin) / (xMax - xMin) * plotW;
  const sy = y => margin.top + (1 - (y - yMin) / (yMax - yMin)) * plotH;
  const svg = svgElement('svg', {viewBox:`0 0 ${width} ${height}`, width, height, class:'compare-chart', role:'img', 'aria-label':`${t(spec.axis)} · A / B`});
  const decimals = step => Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
  for (let y = yMin; y <= yMax + yStep / 2; y += yStep) {
    svg.append(svgElement('line', {x1:margin.left, x2:width - margin.right, y1:sy(y), y2:sy(y), class:Math.abs(y) < yStep / 2 && spec.zeroLine ? 'chart-zero' : 'chart-grid'}));
    const label = svgElement('text', {x:margin.left - 8, y:sy(y) + 4, 'text-anchor':'end', class:'chart-tick'});
    label.textContent = y.toFixed(decimals(yStep));
    svg.append(label);
  }
  for (let x = Math.ceil(xMin / xStep) * xStep; x <= xMax + 1e-9; x += xStep) {
    const label = svgElement('text', {x:sx(x), y:height - margin.bottom + 18, 'text-anchor':'middle', class:'chart-tick'});
    label.textContent = x.toFixed(decimals(xStep));
    svg.append(label);
  }
  const xTitle = svgElement('text', {x:margin.left + plotW / 2, y:height - 8, 'text-anchor':'middle', class:'chart-axis-title'});
  xTitle.textContent = t('chartTime');
  const yTitle = svgElement('text', {x:-(margin.top + plotH / 2), y:14, transform:'rotate(-90)', 'text-anchor':'middle', class:'chart-axis-title'});
  yTitle.textContent = t(spec.axis);
  svg.append(xTitle, yTitle);
  for (const item of series) {
    const step = Math.max(1, Math.floor(item.points.length / (plotW * 1.5)));
    const points = item.points.filter((_, index) => index % step === 0 || index === item.points.length - 1);
    svg.append(svgElement('polyline', {points:points.map(([x, y]) => `${sx(x).toFixed(1)},${sy(y).toFixed(1)}`).join(' '), class:`chart-line ${item.slot}`}));
  }
  const cursor = svgElement('line', {y1:margin.top, y2:margin.top + plotH, class:'chart-cursor', visibility:'hidden'});
  const dots = series.map(item => svgElement('circle', {r:4, class:`chart-dot ${item.slot}`, visibility:'hidden'}));
  const hit = svgElement('rect', {x:margin.left, y:margin.top, width:plotW, height:plotH, class:'chart-hit'});
  svg.append(cursor, ...dots, hit);
  const tooltip = cell('div', '', 'chart-tooltip');
  tooltip.hidden = true;
  const nearest = (points, x) => {
    let low = 0, high = points.length - 1;
    while (high - low > 1) {
      const mid = (low + high) >> 1;
      if (points[mid][0] < x) low = mid; else high = mid;
    }
    return Math.abs(points[low][0] - x) <= Math.abs(points[high][0] - x) ? points[low] : points[high];
  };
  const unit = spec.axis === 'speedAxis' ? 'km/h' : 'm';
  hit.addEventListener('pointermove', event => {
    const box = svg.getBoundingClientRect();
    const x = xMin + ((event.clientX - box.left) * width / box.width - margin.left) / plotW * (xMax - xMin);
    cursor.setAttribute('x1', sx(x)); cursor.setAttribute('x2', sx(x));
    cursor.setAttribute('visibility', 'visible');
    const lines = [`t = ${x.toFixed(2)} s`];
    series.forEach((item, index) => {
      const point = nearest(item.points, x);
      dots[index].setAttribute('cx', sx(point[0])); dots[index].setAttribute('cy', sy(point[1]));
      dots[index].setAttribute('visibility', 'visible');
      lines.push(`${item.slot.toUpperCase()}  ${point[1].toFixed(2)} ${unit}`);
    });
    tooltip.textContent = lines.join('\n');
    tooltip.hidden = false;
    const left = (event.clientX - box.left) + 14;
    tooltip.style.left = `${Math.min(left, box.width - 150)}px`;
    tooltip.style.top = `${Math.max(8, (event.clientY - box.top) - 20)}px`;
  });
  hit.addEventListener('pointerleave', () => {
    cursor.setAttribute('visibility', 'hidden');
    dots.forEach(dot => dot.setAttribute('visibility', 'hidden'));
    tooltip.hidden = true;
  });
  const frame = cell('div', '', 'chart-frame');
  frame.append(svg, tooltip);
  viewer.replaceChildren(frame);
  if (partial) viewer.append(chartMessage(t('chartUnavailable')));
}

function renderCompareVideos(runs) {
  const viewer = $('compareViewer');
  const grid = cell('div', '', 'compare-videos');
  const videos = [];
  runs.forEach((run, index) => {
    const slot = index ? 'b' : 'a';
    const figure = document.createElement('figure');
    const caption = document.createElement('figcaption');
    caption.append(cell('span', slot.toUpperCase(), `compare-badge ${slot}`), perceptionSummary(run.config || {}));
    const file = videoFile(run);
    if ((run.files || []).includes(file) && run.video_status === 'ready') {
      const video = document.createElement('video');
      video.controls = true;
      video.preload = 'metadata';
      video.src = `/runs/${run.run_id}/${file}`;
      video.setAttribute('aria-label', `${slot.toUpperCase()} · ${t('videoLabel')}`);
      videos.push(video);
      figure.append(video, caption);
    } else {
      figure.append(chartMessage(t(run.video_status === 'not_requested' ? 'noVideo' : 'videoUnavailable')), caption);
    }
    grid.append(figure);
  });
  viewer.replaceChildren(grid);
  if (videos.length === 2) {
    const both = cell('button', t('playBoth'), 'secondary play-both');
    both.type = 'button';
    both.onclick = () => {
      const playing = videos.some(video => !video.paused);
      if (playing) videos.forEach(video => video.pause());
      else {
        videos[1].currentTime = Math.min(videos[0].currentTime, videos[1].duration || videos[0].currentTime);
        videos.forEach(video => video.play().catch(() => {}));
      }
    };
    const sync = () => { both.textContent = t(videos.some(video => !video.paused) ? 'pauseBoth' : 'playBoth'); };
    videos.forEach(video => { video.addEventListener('play', sync); video.addEventListener('pause', sync); });
    viewer.append(both);
  }
}

$('compareButton').onclick = () => openCompare();
$('closeCompare').onclick = () => closeCompare();
$('swapCompare').onclick = () => {
  compareIds.reverse();
  compareRuns.reverse();
  renderHistory(historyRuns);
  renderCompare();
};
const compareTabs = [...document.querySelectorAll('[data-compare-tab]')];
for (const button of compareTabs) {
  button.onclick = () => {
    compareTab = button.dataset.compareTab;
    renderCompare();
  };
  button.onkeydown = event => {
    if (event.ctrlKey || event.metaKey || event.altKey || !['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) return;
    event.preventDefault();
    const index = event.key === 'Home' ? 0 : event.key === 'End' ? compareTabs.length - 1 : (compareTabs.indexOf(button) + (event.key === 'ArrowRight' ? 1 : -1) + compareTabs.length) % compareTabs.length;
    compareTabs[index].focus();
    compareTabs[index].click();
  };
}
new ResizeObserver(() => { if (compareChart) requestAnimationFrame(drawCompareChart); }).observe($('compareViewer'));

document.querySelectorAll('[data-history-filter]').forEach(button => button.onclick = () => {
  historyFilter = button.dataset.historyFilter;
  try { localStorage.setItem('simulation-studio.historyFilter', historyFilter); } catch { /* Filtering still works without storage. */ }
  renderHistory(historyRuns);
});

// Remember which parameter sections are open; open any section hiding an invalid field.
const stepState = (() => {
  try { return JSON.parse(localStorage.getItem('simulation-studio.steps')) || {}; } catch { return {}; }
})();
document.querySelectorAll('details.step').forEach(details => {
  if (typeof stepState[details.dataset.step] === 'boolean') details.open = stepState[details.dataset.step];
  details.addEventListener('toggle', () => {
    stepState[details.dataset.step] = details.open;
    try { localStorage.setItem('simulation-studio.steps', JSON.stringify(stepState)); } catch { /* Not essential. */ }
  });
});
$('configForm').addEventListener('invalid', event => {
  for (let details = event.target.closest('details'); details; details = details.parentElement.closest('details')) details.open = true;
}, true);
$('configForm').addEventListener('input', () => updateStepSummaries());
$('configForm').addEventListener('change', () => updateStepSummaries());

// Teaching view: descriptions refer to the implementation, never to simulated live values.
// Only the teacher's HiDrive AEB remains; its source is confidential, so the
// controller node shows the public esmini hook around it, never controller code.
let workspaceView = 'settings', flowSelection = 'overview';
const flowText = pair => pair[language === 'en' ? 1 : 0];
const flowNodes = [
  {
    id:'config', icon:'{ }', tone:'source', row:2, col:1,
    title:['配置与启动','Configure & launch'], sub:['JSON → 独立进程','JSON → worker process'],
    summary:['这一步像填写一张实验任务单：选哪个老师场景、距离会看偏多少、用哪个随机种子。程序先检查这些条件，再保存一份副本，让同样的实验能够重新运行。控制器固定为老师提供的 HiDrive AEB。','This is the experiment’s instruction sheet: which teacher scene to use, how uncertain the measured distance is, and which random seed to use. The program checks and saves these choices so the experiment can be repeated. The controller is always the teacher’s HiDrive AEB.'],
    input:['网页表单，或载入后显示在表单中的配置 JSON；场景初始状态和时长来自 references/<场景>/metadata.json。','The web form, including values loaded from a configuration JSON; initial state and duration come from references/<scenario>/metadata.json.'],
    output:['校验后的参数写入 work/<运行编号>.json，交给独立仿真进程；结果目录另存 config.json，供复现和结果展示读取。','Validated settings go to work/<run-id>.json for a separate simulation process. The result folder retains config.json for reproduction and presentation.'],
    steps:[
      [['收集并检查实验条件','Collect and check the settings'],['app.js 的 getConfig() 收集表单，通过 /api/run 交给 app.py；settings.py 的 validate() 补齐默认值、检查数值范围，并从场景资料读取初速、航向与时长。旧配置中已移除控制器的参数会被丢弃；指定其他控制器会被拒绝。','getConfig() in app.js sends the form to app.py through /api/run. validate() in settings.py fills defaults, checks ranges and reads initial speed, heading and duration from the scene metadata. Settings of removed controllers are dropped; requesting another controller is rejected.']],
      [['把任务交给仿真进程','Pass the task to the worker'],['app.py 将参数写入 work/ 下的 JSON，并启动 runner.py run --config …。runner.run() 校验后直接调用 hidrive_runner.run_hidrive()。','app.py writes a JSON under work/ and starts runner.py run --config …. After validation, runner.run() calls hidrive_runner.run_hidrive().']],
      [['保存本次实验的依据','Keep the inputs needed to reproduce the run'],['结果目录保存 config.json 和 reproduce.ps1；versions.json 记录 esmini 可执行文件、误差接口 PerceptionNoise.hpp 与实现代码的哈希。控制器源码只记录文件大小和修改时间。','The result folder keeps config.json and reproduce.ps1. versions.json records hashes of the esmini executable, the PerceptionNoise.hpp hook and the implementation. Controller sources are recorded only by size and modification time.']]
    ],
    upstream:[], downstream:['world','clock'], file:'runner.py · run',
    code: [
      `# 校验参数范围，补齐默认值；只接受 HiDrive 控制器
c = validate(config)
from hidrive_runner import run_hidrive
# 结果目录、场景、误差文件和 esmini 进程都在 run_hidrive 中完成
return run_hidrive(c, out, diagnostic_only=diagnostic_only, plot_axes_policy=plot_axes_policy)`,
      `# Validate ranges and fill defaults; only the HiDrive controller is accepted.
c = validate(config)
from hidrive_runner import run_hidrive
# run_hidrive creates the folder, scene, noise file and esmini process.
return run_hidrive(c, out, diagnostic_only=diagnostic_only, plot_axes_policy=plot_axes_policy)`
    ],
    note:['左图读取当前表单配置，是代码结构导览；节点颜色不代表正在运行到哪一步。','The map reads the current form settings. Its highlighting describes structure, not live execution progress.']
  },
  {
    id:'reference', icon:'≋', tone:'source', row:2, col:2,
    title:['老师场景','Teacher scenario'], sub:['原文件 · 只改路径','Unchanged · paths only'],
    summary:['实验直接使用老师提供的 GIDAS 追尾场景：道路、两车尺寸、前车轨迹，以及挂在自车 object_1 上的 HiDriveController 和它的参数。Studio 只把道路和目录改成绝对路径，其余保持原样。','The experiment uses the teacher’s GIDAS rear-end scene unchanged: road, vehicle sizes, the lead-vehicle trajectory, and the HiDriveController attached to ego object_1 with its properties. Studio only rewrites road and catalog paths to absolute paths.'],
    input:['references/<场景>/metadata.json 中登记的老师 .xosc、道路文件和基准 truth.csv。','The teacher .xosc, road file and reference truth.csv registered in references/<scenario>/metadata.json.'],
    output:['scene.xosc 交给 esmini 3.8.1；控制器参数（timeGap、mode 等）写入 summary.json，fixed_reference.csv 留作对照。','scene.xosc goes to esmini 3.8.1. Controller properties such as timeGap and mode are saved to summary.json; fixed_reference.csv is kept for comparison.'],
    steps:[
      [['读取并核对源文件','Read and verify the source'],['Reference 读取元数据并核对源文件哈希；build_hidrive_scene() 解析老师的 .xosc。','Reference reads the metadata and checks source hashes; build_hidrive_scene() parses the teacher .xosc.']],
      [['只改路径，确认控制器','Rewrite paths, confirm the controller'],['把道路文件和目录改为绝对路径，并确认 object_1 声明了 HiDriveController；否则立即报错。','Road and catalog paths become absolute, and object_1 must declare HiDriveController; otherwise the run stops.']],
      [['保存场景与控制器参数','Save the scene and controller properties'],['写出 scene.xosc，返回控制器属性供结果记录。前车的运动始终由场景里的原轨迹动作决定。','Writes scene.xosc and returns the controller properties for the record. The lead vehicle always follows the scene’s own trajectory actions.']]
    ],
    upstream:[], downstream:['world'], file:'hidrive_runner.py · build_hidrive_scene',
    code: [
      `# 只改道路与目录路径，场景内容保持老师原文件
root.find('./RoadNetwork/LogicFile').set('filepath', metadata['road_file'])
for directory in root.findall('./CatalogLocations/*/Directory'):
    directory.set('path', str((source.parent / directory.get('path')).resolve()))
# 自车必须挂着老师的 HiDriveController
controller = root.find('./Entities/ScenarioObject[@name="object_1"]/ObjectController/Controller')
if controller is None or controller.get('name') != 'HiDriveController':
    raise RuntimeError('源场景的 object_1 未声明 HiDriveController。')
tree.write(path, encoding='utf-8', xml_declaration=True)`,
      `# Only road and catalog paths change; the teacher scene stays as supplied.
root.find('./RoadNetwork/LogicFile').set('filepath', metadata['road_file'])
for directory in root.findall('./CatalogLocations/*/Directory'):
    directory.set('path', str((source.parent / directory.get('path')).resolve()))
# The ego must carry the teacher's HiDriveController.
controller = root.find('./Entities/ScenarioObject[@name="object_1"]/ObjectController/Controller')
if controller is None or controller.get('name') != 'HiDriveController':
    raise RuntimeError('源场景的 object_1 未声明 HiDriveController。')
tree.write(path, encoding='utf-8', xml_declaration=True)`
    ],
    note:['自车从场景初始状态出发，由控制器接管；界面中的初速和航向只作显示。','The ego starts from the scene’s initial state and is driven by the controller; speed and heading in the form are display only.']
  },
  {
    id:'clock', icon:'Δt', tone:'source', row:2, col:3,
    title:['仿真时钟','Simulation clock'], sub:['固定步长 dt','Fixed step dt'],
    summary:['esmini 以固定步长 dt 推进，控制器每一步运行一次，测距误差也每一步更新一次。运行在老师场景的停止条件或首次碰撞时结束。','esmini advances at a fixed step dt. The controller runs once per step, and the range error is updated once per step. The run ends at the scene’s stop condition or first contact.'],
    input:['dt_s（默认 0.01 s）和场景停止时刻。','dt_s (0.01 s by default) and the scene stop time.'],
    output:['--fixed_timestep 交给 esmini；误差文件按同一网格为每一步准备一个 OU 样本，并多留 2 s 余量。','--fixed_timestep is passed to esmini. The noise file holds one OU sample per step on the same grid, plus a 2 s margin.'],
    steps:[
      [['确定时长','Determine the duration'],['settings.original_duration() 由场景停止条件得到时长；误差序列覆盖到停止时刻之后。','settings.original_duration() derives the duration from the scene stop condition; the noise sequence extends beyond it.']],
      [['固定步长运行','Run at a fixed step'],['esmini 以 --fixed_timestep 运行，与墙钟时间无关；同一配置重复运行结果相同。','esmini runs with --fixed_timestep, independent of wall-clock time; the same settings reproduce the same result.']],
      [['按步取误差','Index the error by step'],['C++ 接口用 step = round(t / dt) 取对应样本，因此误差与控制器步严格一一对应。','The C++ hook uses step = round(t / dt) to pick the sample, so each error maps to exactly one controller step.']]
    ],
    upstream:['config'], downstream:['world','noise'], file:'hidrive_runner.py · run_hidrive',
    code: [
      `# 误差序列覆盖到场景停止时刻
duration = max(c['duration_s'], float(ref.meta.get('stop_trigger_s') or 0.0))
# …省略误差文件写入
# 固定步长推进 esmini
'--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',`,
      `# The noise sequence covers the scene stop time.
duration = max(c['duration_s'], float(ref.meta.get('stop_trigger_s') or 0.0))
# ... Noise file writing omitted.
# Advance esmini at a fixed step.
'--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',`
    ],
    note:['没有单独的观测周期：控制器每一步都读取一次目标。','There is no separate observation period: the controller reads targets at every step.']
  },
  {
    id:'world', icon:'◈', tone:'main', row:4, col:1,
    title:['esmini 3.8.1 世界','esmini 3.8.1 world'], sub:['隔离编译 · 含 HiDrive','Isolated build · HiDrive'],
    summary:['老师的控制器已由我们移植到 esmini 3.8.1 接口，Studio 用一份隔离编译的 esmini 3.8.1 运行整场实验：道路、前车轨迹、控制器、车辆运动和碰撞检测都在这个进程里完成，Python 只负责准备输入和读取结果。','The teacher’s controller has been ported by us to the esmini 3.8.1 API; Studio runs the whole experiment in an isolated esmini 3.8.1 build: road, lead trajectory, controller, vehicle motion and collision detection all happen inside that process. Python only prepares inputs and reads results.'],
    input:['scene.xosc、固定步长、可选的误差文件 perception_noise.tsv。','scene.xosc, the fixed step and the optional perception_noise.tsv.'],
    output:['hidrive_esmini.csv（每步真值与碰撞 ID）、esmini.log（控制器日志：AEB_active、Acceleration）、perception_applied.tsv（每步实际加入的误差）。','hidrive_esmini.csv (per-step truth and collision IDs), esmini.log (controller log: AEB_active, Acceleration) and perception_applied.tsv (the error actually applied each step).'],
    steps:[
      [['检查编译版本','Check the build'],['check_build() 确认可执行文件存在、控制器文件未更新、误差接口哈希与编译时一致；否则提示运行 hidrive/build.py。','check_build() confirms the executable exists, the controller files are unchanged and the hook hash matches the build; otherwise it asks for hidrive/build.py.']],
      [['离屏运行','Run off-screen'],['控制器需要渲染器对象，纯 --headless 会崩溃；因此加 --window 使用离屏缓冲，不显示窗口。','The controller needs a viewer object and crashes under pure --headless, so --window provides an off-screen buffer without showing a window.']],
      [['一次跑完整场','Run the whole scene in one go'],['esmini 进程从头运行到场景结束，Python 不逐步干预。','The esmini process runs from start to finish; Python does not intervene step by step.']]
    ],
    upstream:['reference','config','clock','restore'], downstream:['noise','audit'], file:'hidrive_runner.py · run_hidrive',
    code: [
      `# 离屏窗口 + 固定步长；误差文件仅在有误差时传入
args = [EXECUTABLE, '--osc', out / 'scene.xosc', '--headless', '--window', '0', '0', '320', '180',
        '--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',
        '--csv_logger', out / 'hidrive_esmini.csv', '--record', out / 'hidrive_replay.dat',
        '--logfile_path', out / 'esmini.log', '--disable_stdout', '--path', RUNTIME / 'resources',
        *noise_args]`,
      `# Off-screen window and fixed step; the noise file only when noisy.
args = [EXECUTABLE, '--osc', out / 'scene.xosc', '--headless', '--window', '0', '0', '320', '180',
        '--fixed_timestep', c['dt_s'], '--collision', '--seed', '0',
        '--csv_logger', out / 'hidrive_esmini.csv', '--record', out / 'hidrive_replay.dat',
        '--logfile_path', out / 'esmini.log', '--disable_stdout', '--path', RUNTIME / 'resources',
        *noise_args]`
    ],
    note:['视频回放另用完整 esmini 3.8.1 渲染；两者只共享保存的数据。','Video replay is rendered separately with full esmini 3.8.1; the two share only saved data.']
  },
  {
    id:'noise', icon:'±', tone:'main', row:4, col:2,
    title:['时间相关测距误差','Temporally correlated range error'], sub:['OU 记忆 · 距离幅度','OU memory · range amplitude'],
    summary:['传感器测距并不完美：车明明在一个位置，系统却可能觉得它更近或更远。而且这种偏差往往会持续一段时间——前一刻把距离看远了，下一刻多半仍然看远。这一步准备这种会延续的偏差。\n\nOU 误差来自 Ornstein–Uhlenbeck 过程，可以理解为一种“有记忆、又会慢慢回到零”的随机误差：每一步保留上一步误差的一部分，再叠加一点新的随机变化。相关时间 τ 描述记忆能维持多久：每经过 τ 秒，旧误差的影响衰减到原来的约 37%。\n\n误差拆成两部分：Python 预先为每个目标生成整段无单位的 OU 状态 z；以米计的幅度取决于当时的真实距离，在 esmini 内部按当前距离计算。这样不用在 Python 中预测车辆会走到哪里。','Range sensing is imperfect: a vehicle is at one position, but the system may perceive it as closer or farther away, and such offsets tend to persist. This step prepares that persistent error.\n\nOU error comes from the Ornstein–Uhlenbeck process: a random error with memory that gradually returns towards zero. Each step keeps part of the previous error and adds a small new random change. The correlation time τ says how long the memory lasts: every τ seconds the old error’s influence decays to about 37%.\n\nThe error has two parts. Python pre-generates a dimensionless OU state z for each target over the whole run; the amplitude in metres depends on the live true range and is computed inside esmini. Python therefore never has to predict where the vehicles will be.'],
    input:['config 中的随机种子、相关时间 τ、零误差距离、参考距离和 MAE 百分比；场景中的目标列表与 dt。','Seed, correlation time τ, zero-error range, reference range and MAE percentage from the configuration; the scene’s targets and dt.'],
    output:['perception_noise.tsv：观察者名称、幅度参数，以及每个目标每一步的 z。由 --perception_noise 交给 esmini。','perception_noise.tsv: the observer name, amplitude settings, and z for every target at every step, passed to esmini with --perception_noise.'],
    steps:[
      [['每个目标一条独立随机流','One random stream per target'],['CorrelatedRangeNoise 用 “种子:range-ou:目标ID” 初始化随机流，首个样本取平稳分布 N(0,1)；新增目标不会改变已有目标的误差序列。','CorrelatedRangeNoise seeds each stream with “seed:range-ou:object-id” and starts from the stationary N(0,1); a new target never changes an existing target’s sequence.']],
      [['按步推进 OU 状态','Advance the OU state each step'],['write_noise_file() 在 t = k·dt 上调用 sample()，得到 z₀, z₁, …；ρ = exp(−dt/τ)。','write_noise_file() calls sample() at t = k·dt to obtain z₀, z₁, …, with ρ = exp(−dt/τ).']],
      [['写出误差文件','Write the noise file'],['把幅度参数和各目标的 z 序列写成制表符分隔文本；无误差模式不生成这个文件，esmini 也不加载接口。','Writes the amplitude settings and each target’s z sequence as tab-separated text. Ideal mode writes no file, and esmini does not activate the hook.']]
    ],
    upstream:['world','clock'], downstream:['inject'], file:'models.py · CorrelatedRangeNoise / hidrive_runner.py · write_noise_file',
    code:[
      `# 用相邻两步的间隔计算记忆强度
elapsed = time_s - last_time
rho = math.exp(-elapsed / self.tau_s)
if elapsed > 0:
    # expm1 在小时间间隔下也能稳定计算 1 − exp(−2Δt/τ)
    innovation_scale = math.sqrt(-math.expm1(-2 * elapsed / self.tau_s))
    z = rho * z + innovation_scale * self.streams[object_id].gauss(0, 1)

# write_noise_file：为每个目标生成整段单位方差 OU 序列
noise = CorrelatedRangeNoise(c['seed'], c['noise_correlation_time_s'])
steps = round((duration_s + NOISE_MARGIN_S) / c['dt_s']) + 1
samples = [noise.sample(target['id'], round(k * c['dt_s'], 8))[0] for k in range(steps)]`,
      `# Memory depends on the interval between steps.
elapsed = time_s - last_time
rho = math.exp(-elapsed / self.tau_s)
if elapsed > 0:
    # expm1 accurately evaluates 1 - exp(-2*dt/tau) for small dt.
    innovation_scale = math.sqrt(-math.expm1(-2 * elapsed / self.tau_s))
    z = rho * z + innovation_scale * self.streams[object_id].gauss(0, 1)

# write_noise_file: a unit-variance OU sequence for every target.
noise = CorrelatedRangeNoise(c['seed'], c['noise_correlation_time_s'])
steps = round((duration_s + NOISE_MARGIN_S) / c['dt_s']) + 1
samples = [noise.sample(target['id'], round(k * c['dt_s'], 8))[0] for k in range(steps)]`
    ],
    note:['这是可调的误差试验模型，尚未用实测数据标定。它只改变目标中心位置，不改变速度、航向和尺寸。','This adjustable error model has not been calibrated against measurements. It changes only the target centre position, not speed, heading or size.']
  },
  {
    id:'inject', icon:'⇄', tone:'main', row:4, col:3,
    title:['把目标挪到“看到的位置”','Move targets to perceived poses'], sub:['esmini 公共接口 · C++','Public esmini hook · C++'],
    summary:['控制器源码不能修改，所以误差加在它“看到的世界”上：在自车控制器运行前的一瞬间，把每个目标沿自车视线挪到带误差的距离；控制器读到的就是看偏的位置。控制器运行完立即放回真实位置。','The controller source must not change, so the error is applied to the world it sees: just before the ego controller runs, each target is moved along the ego line of sight to the erroneous range. The controller reads the perceived position, and the true position is restored right after.'],
    input:['当前步的真实位姿（自车与目标包围盒中心）、误差文件中该步的 z、幅度参数。','This step’s true poses (ego and target box centres), the step’s z from the noise file, and the amplitude settings.'],
    output:['控制器 Step 期间目标处于观测位置；每步的真实距离、σ、z、误差、测距和观测中心写入 perception_applied.tsv。','During the controller Step, targets sit at their observed positions. True range, σ, z, error, measured range and observed centre are logged to perception_applied.tsv every step.'],
    steps:[
      [['只对观察者的控制器生效','Only for the observer’s controller'],['ScenarioEngine 在每个控制器 Step 前调用 Begin()；只有该控制器属于自车（observer）时才移动目标。','ScenarioEngine calls Begin() before each controller Step; targets move only when that controller belongs to the ego (observer).']],
      [['按当前真实距离算误差','Compute the error at the live true range'],['σ(r) 由当前包围盒中心距离计算，误差 e = σ(r)·z，测距 r̂ = r + e。','σ(r) is computed from the current centre-to-centre range; error e = σ(r)·z and measured range r̂ = r + e.']],
      [['沿视线挪动目标','Move the target along the line of sight'],['r̂ ≥ 0 时，观测中心 = 自车中心 + (目标中心 − 自车中心)·r̂/r，航向不变。r̂ < 0 为无效测距：该步把目标放到自车后方 1000 m，使其不会被当作前车。','For r̂ ≥ 0 the observed centre is ego centre + (target − ego)·r̂/r, heading unchanged. r̂ < 0 is invalid: the target is placed 1000 m behind the ego for that step, so it cannot be taken as the lead vehicle.']]
    ],
    upstream:['noise'], downstream:['controller'], file:'hidrive/PerceptionNoise.hpp · Begin',
    code:[
      `// 只处理自车（observer）的控制器步
void Begin(scenarioengine::Object* observer, std::vector<scenarioengine::Object*>& objects, double time)
long   step = std::lround(time / dt_);
// …省略逐目标遍历与序列长度检查
double z = sequence->second[static_cast<size_t>(step)];
// 按当前真实中心距离计算幅度
double mae   = reference_ * pct_ / 100.0 * std::max(0.0, range - zero_) / (reference_ - zero_);
double sigma = mae * std::sqrt(kPi / 2.0);
double error = sigma * z;
double raw   = range + error;
bool   valid = raw >= 0.0;
// …省略观测中心计算（有效沿视线缩放，无效放到自车后方）
saved_.emplace_back(obj, obj->pos_);
obj->pos_.SetInertiaPos(obj->pos_.GetX() + ox - tx, obj->pos_.GetY() + oy - ty, obj->pos_.GetH());`,
      `// Only the ego (observer) controller step is affected.
void Begin(scenarioengine::Object* observer, std::vector<scenarioengine::Object*>& objects, double time)
long   step = std::lround(time / dt_);
// ... Per-target loop and sequence-length checks omitted.
double z = sequence->second[static_cast<size_t>(step)];
// Amplitude from the live true centre range.
double mae   = reference_ * pct_ / 100.0 * std::max(0.0, range - zero_) / (reference_ - zero_);
double sigma = mae * std::sqrt(kPi / 2.0);
double error = sigma * z;
double raw   = range + error;
bool   valid = raw >= 0.0;
// ... Observed-centre computation omitted (scaled along the line of sight, or behind the ego).
saved_.emplace_back(obj, obj->pos_);
obj->pos_.SetInertiaPos(obj->pos_.GetX() + ox - tx, obj->pos_.GetY() + oy - ty, obj->pos_.GetH());`
    ],
    note:['控制器用自己的方式选前车（自身传感器命中或全部对象），这里不改变它的选择逻辑，只改变目标所在位置。','The controller selects its lead vehicle its own way (its sensor hits or all objects). The hook does not change that logic, only where targets are.']
  },
  {
    id:'controller', icon:'↯', tone:'decision', row:5, col:3,
    title:['老师的 HiDrive AEB','Teacher’s HiDrive AEB'], sub:['源码保密 · 原样编译','Confidential · compiled as supplied'],
    summary:['自车由老师提供的 HiDriveController 控制：它读取前车位置和速度，决定是否触发 AEB 并给出加速度，同时用自己的车辆模型更新自车。Studio 不阅读、不修改它的源码（移植到 3.8.1 接口由我们手工完成），只把它编译进 esmini 3.8.1。','The ego is driven by the teacher’s HiDriveController: it reads the lead vehicle’s position and speed, decides whether to trigger AEB, outputs acceleration and updates the ego with its own vehicle model. Studio neither reads nor modifies its source (we ported it to the 3.8.1 API by hand); it is compiled into esmini 3.8.1.'],
    input:['场景中的控制器参数（timeGap、mode 等）、自车状态，以及 Step 期间处于观测位置的目标。','Controller properties from the scene (timeGap, mode, …), the ego state, and targets at their observed positions during Step.'],
    output:['自车的新位姿与速度（写回 esmini）；日志中的 AEB_active 与 Acceleration 供 Studio 统计首次触发和首次制动。','The ego’s new pose and speed, written back to esmini; AEB_active and Acceleration in its log give Studio the first trigger and first braking time.'],
    steps:[
      [['编译控制器','Compile the controller'],['hidrive/build.py 把两个控制器文件作为编译输入复制进 esmini 3.8.1 源码树，只修改公共文件：注册控制器、加入误差接口和 --perception_noise 选项。','hidrive/build.py copies the two controller files into the esmini 3.8.1 tree as compiler inputs and edits only public files: controller registration, the noise hook and the --perception_noise option.']],
      [['在接口之间运行','Run between the hooks'],['引擎每步依次调用 Begin() → 控制器 Step() → End()。控制器看到的是带误差的目标，其余时间世界保持真值。','Each step the engine calls Begin() → controller Step() → End(). The controller sees uncertain targets; the rest of the step keeps true states.']],
      [['从日志读取决策','Read decisions from the log'],['控制器内部状态不可见；Studio 只读取它写入 esmini.log 的 AEB_active 和 Acceleration。','The controller’s internals are not visible; Studio reads only AEB_active and Acceleration from esmini.log.']]
    ],
    upstream:['inject'], downstream:['restore'], file:'hidrive/build.py · ScenarioEngine hook',
    code:[
      `# build.py 对 esmini 公共源码 ScenarioEngine.cpp 的修改：
# 控制器 Step 前后各加一行接口调用；控制器源码本身不改动
'                studio::PerceptionNoise::Inst().Begin(scenarioReader->controller_[i]->GetRoadObject(), entities_.object_, simulationTime_);\\n'
'                scenarioReader->controller_[i]->Step(deltaSimTime);\\n'
'                studio::PerceptionNoise::Inst().End();\\n'),`,
      `# build.py edits the public esmini file ScenarioEngine.cpp:
# one hook call before and after the controller Step; the controller is untouched.
'                studio::PerceptionNoise::Inst().Begin(scenarioReader->controller_[i]->GetRoadObject(), entities_.object_, simulationTime_);\\n'
'                scenarioReader->controller_[i]->Step(deltaSimTime);\\n'
'                studio::PerceptionNoise::Inst().End();\\n'),`
    ],
    note:['控制器源码按课程要求保密，这里不展示、也不分析其内部逻辑。','The controller source is confidential under the course rules; its internals are not shown or analysed here.']
  },
  {
    id:'restore', icon:'↺', tone:'decision', row:5, col:2,
    title:['恢复真实位置','Restore true poses'], sub:['End() · 回到真值','End() · back to truth'],
    summary:['控制器一运行完，就把目标放回真实位置。这样误差只影响控制器的判断，不会改变前车实际怎么走，碰撞检测和记录也都基于真值。','As soon as the controller finishes, targets go back to their true positions. The error affects only the controller’s decisions, never how the lead vehicle actually moves; collision checks and logs use truth.'],
    input:['Begin() 保存的各目标原始位姿。','The original poses saved by Begin().'],
    output:['真实世界状态；之后 esmini 照常处理碰撞检测和 CSV 记录，下一步再重复整个过程。','The true world state; esmini then checks collisions and writes the CSV as usual, and the next step repeats the cycle.'],
    steps:[
      [['完整复制位姿','Copy the full pose back'],['用 Position::Duplicate() 恢复，保留路线和轨迹指针；用赋值运算会丢失轨迹，导致前车跑偏。','Position::Duplicate() restores the pose and keeps route and trajectory pointers; plain assignment would drop the trajectory and derail the lead vehicle.']],
      [['重算路线位置','Recompute the route position'],['如果目标有路线，再调用 CalcRoutePosition()，与引擎自身的状态更新方式一致。','If a target has a route, CalcRoutePosition() is called, matching how the engine applies states.']],
      [['写出日志','Flush the log'],['每步结束时刷新 perception_applied.tsv，Python 之后据此填写 observations.csv。','perception_applied.tsv is flushed each step; Python later fills observations.csv from it.']]
    ],
    upstream:['controller'], downstream:['world'], file:'hidrive/PerceptionNoise.hpp · End',
    code:[
      `// 恢复真实位姿，方式与引擎应用网关状态相同
void End()
for (auto& item : saved_)
item.first->pos_.Duplicate(item.second);
if (item.first->pos_.GetRoute() != nullptr)
item.first->pos_.CalcRoutePosition();
// …省略日志刷新
saved_.clear();`,
      `// Restore true poses the same way the engine applies gateway states.
void End()
for (auto& item : saved_)
item.first->pos_.Duplicate(item.second);
if (item.first->pos_.GetRoute() != nullptr)
item.first->pos_.CalcRoutePosition();
// ... Log flushing omitted.
saved_.clear();`
    ],
    note:['误差幅度为 0 时，结果与无误差运行逐字节一致，说明挪动与恢复本身不改变仿真。','With zero amplitude the results match the ideal run byte for byte, so moving and restoring alone does not change the simulation.']
  },
  {
    id:'audit', icon:'✓', tone:'evidence', row:7, col:1,
    title:['真值与碰撞校验','Truth & collision audit'], sub:['记录 · 比较 · 验证','Record · compare · verify'],
    summary:['独立记录实际发生了什么：每一步的真实位置与速度、碰撞、控制器何时触发 AEB、何时开始制动。首次碰撞即结束记录。','Records what actually happened: true positions and speeds every step, collisions, when the controller triggered AEB and when braking began. Recording stops at first contact.'],
    input:['hidrive_esmini.csv（真值与碰撞 ID）、esmini.log（AEB_active、Acceleration）、perception_applied.tsv。','hidrive_esmini.csv (truth and collision IDs), esmini.log (AEB_active, Acceleration) and perception_applied.tsv.'],
    output:['truth.csv、ego.csv、observations.csv、metrics.csv 与 summary.json（碰撞、首次触发 / 制动、最小距离、测距 RMSE、无效测距次数）。','truth.csv, ego.csv, observations.csv, metrics.csv and summary.json (collision, first trigger / brake, minimum range, range RMSE, invalid measurements).'],
    steps:[
      [['逐步读取真值','Read truth step by step'],['read_esmini_csv() 读取每步对象状态；真实净距和 TTC 只作诊断，不参与控制。','read_esmini_csv() reads object states per step; true gap and TTC are diagnostics only and never feed control.']],
      [['配上当步的观测','Attach the step’s observation'],['按 (时间, 目标) 从 perception_applied.tsv 取控制器看到的测距和观测中心，写入 observations.csv。','The measured range and observed centre the controller saw are taken from perception_applied.tsv by (time, target) and written to observations.csv.']],
      [['判定触发、制动与碰撞','Detect trigger, braking and contact'],['首次 AEB_active = 1 为触发；首次 Acceleration < −0.01 m/s² 为制动；esmini 碰撞 ID 非空即碰撞并结束。','The first AEB_active = 1 is the trigger; the first Acceleration < −0.01 m/s² is braking; a non-empty esmini collision list is contact and ends the record.']]
    ],
    upstream:['world'], downstream:['output'], file:'hidrive_runner.py · run_hidrive',
    code: [
      `# esmini 真实包围盒碰撞；不由带误差的观测判定
hits = [int(v) for v in rows[ego_name].get('collision_ids', '').split() if v.strip().isdigit()]
hit = bool(hits)
# …省略距离与 TTC 诊断
active = values.get('AEB_active') == '1'
accel = number(values.get('Acceleration'))
if active and first_trigger is None:
    first_trigger = t
# …省略首次制动与停车时刻
if hit:
    break`,
      `# Truth-box collisions from esmini, never from uncertain observations.
hits = [int(v) for v in rows[ego_name].get('collision_ids', '').split() if v.strip().isdigit()]
hit = bool(hits)
# ... Range and TTC diagnostics omitted.
active = values.get('AEB_active') == '1'
accel = number(values.get('Acceleration'))
if active and first_trigger is None:
    first_trigger = t
# ... First braking and stop times omitted.
if hit:
    break`
    ],
    note:['真值只用于校验和评估，不会作为隐藏的准确距离交给控制器。','Truth supports auditing and evaluation, never a hidden exact-range input to the controller.']
  },
  {
    id:'output', icon:'▤', tone:'evidence', row:7, col:2,
    title:['曲线、视频与归档','Plots, video & archive'], sub:['CSV → PNG / MP4','CSV → PNG / MP4'],
    summary:['将一次实验转换为可以检查和展示的结果。曲线读取已保存的 CSV；视频把同一份运动数据交给完整 esmini 3.8.1 重新渲染，再由 FFmpeg 编码。','Turns the saved experiment into reviewable results. Plots read the CSV files; video replays the same motion in full esmini 3.8.1 and encodes the frames with FFmpeg.'],
    input:['本次实验的 CSV、summary.json 和配置。','This run’s CSV files, summary.json and configuration.'],
    output:['速度 / 加速度图、距离 / 误差图、可选 video.mp4；后台接口将历史与结果文件提供给前端。','Speed/acceleration plots, range/error plots and optional video.mp4. The service exposes history and result files to the frontend.'],
    steps:[
      [['生成曲线','Plot the curves'],['plots.make_plots() 读取 CSV；plot_axes 让同一场景的横轴一致、纵轴按幅度分组。','plots.make_plots() reads the CSV files; plot_axes keeps x axes consistent per scene and groups y axes by magnitude.']],
      [['回放视频','Replay the video'],['video.py 把保存的真值逐帧写入 esmini 3.8.1，取渲染像素叠加黑框，再交给 FFmpeg。','video.py writes the saved truth into esmini 3.8.1 frame by frame, overlays the outlines and passes the pixels to FFmpeg.']],
      [['记录状态','Record the status'],['summary.json 标明视频是否成功、esmini 是否有错误提示，供历史列表显示。','summary.json records whether the video succeeded and whether esmini reported errors, for the history list.']]
    ],
    upstream:['audit'], downstream:['overlay'], file:'runner.py · finish_run / video.py · export',
    code: [
      `# 读取已保存的 CSV 生成曲线
if not diagnostic_only:
    from plots import make_plots

    make_plots(out)

# video.py：回放已保存的状态，不重新运行控制器
for s in frames[t]:
    engine.report(s)
engine.step(t - previous)
# 取得渲染像素，后续交给 FFmpeg 编码为 MP4
width, height, pix, data = engine.image()`,
      `# Plot the saved CSV data.
if not diagnostic_only:
    from plots import make_plots

    make_plots(out)

# video.py: replay saved motion without rerunning the controller.
for s in frames[t]:
    engine.report(s)
engine.step(t - previous)
# Fetch rendered pixels for subsequent FFmpeg MP4 encoding.
width, height, pix, data = engine.image()`
    ],
    note:['视频是结果回放；控制输入来自 esmini 3.8.1 中的实际运行，不来自视频图像。','Video is an output replay. Control inputs come from the actual esmini 3.8.1 run, not video pixels.']
  },
  {
    id:'overlay', icon:'▱', tone:'evidence', row:8, col:2,
    title:['黑框如何实现','How black outlines work'], sub:['观测快照 → 车身轮廓','Observation snapshot → outline'],
    summary:['黑框表示控制器在该时刻“看到”的目标车位置。视频导出时，Python 读取保存的观测中心，用车身长宽和当时的朝向构造矩形，再将矩形投影到画面上。','The black outline shows where the controller perceived the target at that moment. During video export, Python reads the saved observed centre, builds a rectangle from the vehicle dimensions and heading, then projects it into the image.'],
    input:['observations.csv 提供时间、目标 ID、有效标记和 observed_center_x_m / observed_center_y_m；同一时刻的真值与车辆元数据提供朝向、长宽及高度。','observations.csv supplies timestamps, target IDs, validity and observed_center_x_m / observed_center_y_m. Matching truth states and vehicle metadata supply heading, length, width and height.'],
    output:['在 esmini 渲染像素上叠加黑色轮廓与白色细描边，再交给 FFmpeg 输出 video.mp4；video_metadata.json 保存显示规则和绘制数量。','Adds a black outline with white edging to the esmini image, then passes it to FFmpeg for video.mp4. video_metadata.json records display rules and drawing counts.'],
    upstream:['output'], downstream:[], file:'video_overlay.py · ObservationOverlay / PerceivedBox',
    code:[
      `# 取不晚于当前视频时刻的最新完整观测快照
index = bisect_right(self.times, time_s + 1e-9) - 1
return self.snapshots[self.times[index]] if index >= 0 else []

# PerceivedBox.corners：局部矩形按航向旋转、平移
co, si = math.cos(self.heading), math.sin(self.heading)
return [
    (self.x + co * x - si * y, self.y + si * x + co * y, self.z)
    for x, y in [
        (-self.length / 2, -self.width / 2),
        (self.length / 2, -self.width / 2),
        (self.length / 2, self.width / 2),
        (-self.length / 2, self.width / 2),
    ]
]

# ObservationOverlay.draw：投影四角，沿同一路径先画白边、再画黑线
boxes = self.boxes_at(time_s)
for box in boxes:
    points = [project_top_point(point, camera, width, height) for point in box.corners()]
    # 白边 4 像素，黑线 2 像素；闭合路径保持真实车身长宽
    draw.line(points + points[:1], fill=(250, 250, 250), width=4, joint='curve')
    draw.line(points + points[:1], fill=(12, 12, 12), width=2, joint='curve')`,
      `# Select the latest complete snapshot at or before the video time.
index = bisect_right(self.times, time_s + 1e-9) - 1
return self.snapshots[self.times[index]] if index >= 0 else []

# PerceivedBox.corners: rotate and translate using the heading.
co, si = math.cos(self.heading), math.sin(self.heading)
return [
    (self.x + co * x - si * y, self.y + si * x + co * y, self.z)
    for x, y in [
        (-self.length / 2, -self.width / 2),
        (self.length / 2, -self.width / 2),
        (self.length / 2, self.width / 2),
        (-self.length / 2, self.width / 2),
    ]
]

# ObservationOverlay.draw: project the corners and draw both strokes.
boxes = self.boxes_at(time_s)
for box in boxes:
    points = [project_top_point(point, camera, width, height) for point in box.corners()]
    # White edging: 4 px; black line: 2 px, following the same closed path.
    draw.line(points + points[:1], fill=(250, 250, 250), width=4, joint='curve')
    draw.line(points + points[:1], fill=(12, 12, 12), width=2, joint='curve')`
    ],
    note:['黑框复用已保存的观测，不重新抽样误差，也不反馈车辆控制。视频按 25 fps 取最近一步的观测；无效测距的步不画黑框。','The outline reuses saved observations without resampling and never feeds control. Video frames at 25 fps use the latest step’s observation; invalid steps draw no outline.']
  }
];

const flowEdges = [
  ['config','world','vertical'], ['reference','world','vertical'], ['clock','noise','vertical',true],
  ['world','noise','horizontal'], ['noise','inject','horizontal'], ['inject','controller','vertical'],
  ['controller','restore','horizontal'], ['restore','world','feedback'],
  ['world','audit','audit',true], ['audit','output','horizontal',true], ['output','overlay','vertical',true]
];
const escapeFlow = value => String(value).replace(/[&<>"']/g, char => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
const flowLabel = (zh, en) => language === 'en' ? en : zh;
function flowConfig() { return boot ? getConfig() : {}; }
function flowExact(c) {
  return ['recorded_truth','ideal'].includes(c.uncertainty) ||
    (c.uncertainty === 'distance_ou_range' && Number(c.noise_reference_mae_pct) === 0);
}

function flowImplementationHtml(item, id) {
  const steps = (item.steps || []).map(([title, body]) =>
    `<li><h4>${escapeFlow(flowText(title))}</h4><p>${escapeFlow(flowText(body))}</p></li>`
  ).join('');
  return `<section class="flow-implementation" aria-labelledby="flowImplementationTitle">
    <h3 id="flowImplementationTitle">${flowLabel('怎么实现','HOW IT WORKS')}</h3>
    ${steps ? `<ol class="flow-steps">${steps}</ol>` : ''}
    ${id === 'overlay' ? overlayExplanationHtml() : ''}
    ${id === 'noise' ? noiseExplanationHtml() : ''}
    ${item.note ? `<p class="flow-note">${escapeFlow(flowText(item.note))}</p>` : ''}
  </section>`;
}

function flowState(id, c) {
  if (!boot) return {label:flowLabel('等待配置','Awaiting settings'), detail:flowLabel('连接后台后显示当前配置对应的执行路径。','The execution path will appear once settings are loaded.')};
  const exact = flowExact(c), noisy = c.uncertainty === 'distance_ou_range';
  if (id === 'overlay') {
    return {
      label:!noisy ? flowLabel('理想模式 · 不显示','Ideal mode · hidden') : c.export_video ? flowLabel('视频中自动显示','Automatic in video') : flowLabel('待开启视频导出','Enable video export'),
      detail:!noisy ? flowLabel('当前为无误差运行，视频不绘制黑框。选择 OU 误差后，导出的视频会自动显示有效观测的黑框。','Ideal runs draw no outlines. Selecting OU range error automatically adds valid observed boxes to exported video.') : !c.export_video ? flowLabel('当前已选 OU 误差，但未开启 MP4 导出。勾选“导出 MP4 视频”后，新视频自动叠加黑框。','OU range error is selected, but MP4 export is off. Enable “Export MP4 video” to add outlines to new videos.') : flowLabel(`当前视频自动显示黑框；每步（${c.dt_s} s）都有新的观测。零误差幅度仍绘制；无效测距时隐藏。`,`Outlines are enabled automatically, with a new observation every step (${c.dt_s} s). Zero amplitude still draws boxes; invalid measurements hide them.`),
      bypassed:!noisy || !c.export_video
    };
  }
  const label = {
    config:flowLabel('当前表单配置','Current form settings'),
    reference:scenarioName(c.scenario, c.scenario),
    clock:`dt ${c.dt_s} s`,
    world:'esmini 3.8.1 · HiDrive',
    noise:exact ? flowLabel('不附加误差','No added error') : flowLabel('OU 时间相关误差','Temporally correlated OU'),
    inject:noisy ? flowLabel('每步挪动并恢复','Move & restore each step') : flowLabel('未启用','Inactive'),
    controller:'HiDriveController',
    restore:noisy ? 'Position::Duplicate' : flowLabel('未启用','Inactive'),
    audit:flowLabel('每一步记录','Recorded every step'),
    output:c.export_video ? 'CSV + PNG + MP4' : 'CSV + PNG'
  }[id];
  const detail = {
    config:flowLabel('显示的是当前表单，包括尚未运行的修改；切回实验设置后可继续编辑。','These are the current form values, including edits not yet run. Switch back to settings to edit them.'),
    reference:flowLabel('当前场景：','Current scene: ') + scenarioName(c.scenario,c.scenario),
    clock:flowLabel(`固定步长 ${c.dt_s} s；控制器与测距误差每步更新一次。`,`Fixed step ${c.dt_s} s; the controller and the range error update once per step.`),
    world:flowLabel('使用隔离编译的 esmini 3.8.1（含老师的 HiDriveController）一次运行完整场景。','Runs the whole scene in the isolated esmini 3.8.1 build containing the teacher’s HiDriveController.'),
    noise:exact ? flowLabel('当前不附加测距误差，控制器看到真实位置。','No range error is added; the controller sees true positions.') : flowLabel(`相关时间 τ = ${c.noise_correlation_time_s} s；${c.noise_zero_range_m} m 以内为零误差，在 ${c.noise_reference_range_m} m 处的平均绝对误差（MAE）为该距离的 ${c.noise_reference_mae_pct}%。随机种子为 ${c.seed}。`,`Correlation time τ = ${c.noise_correlation_time_s} s. Error is zero within ${c.noise_zero_range_m} m; mean absolute error (MAE) at ${c.noise_reference_range_m} m is ${c.noise_reference_mae_pct}% of that distance. Random seed: ${c.seed}.`),
    inject:noisy ? flowLabel('esmini 以 --perception_noise 加载误差文件，在自车控制器每步运行前挪动目标。','esmini loads the noise file with --perception_noise and moves targets before every ego controller step.') : flowLabel('无误差运行不传误差文件，接口不做任何事。','Ideal runs pass no noise file, so the hook does nothing.'),
    controller:flowLabel('控制器参数来自老师场景；源码保密，不在此展示。','Controller properties come from the teacher scene; the source is confidential and not shown.'),
    restore:noisy ? flowLabel('控制器每步运行后立即恢复目标真值。','Targets return to truth right after every controller step.') : flowLabel('无误差运行中目标从未被挪动。','In ideal runs targets are never moved.'),
    audit:flowLabel('记录首次触发、首次制动和首次碰撞；碰撞即结束，否则运行到场景停止条件。','Records the first trigger, first braking and first contact; contact ends the run, otherwise it runs to the scene stop condition.'),
    output:flowLabel(`自动保存 CSV 和曲线；当前${c.export_video ? '开启' : '关闭'} MP4 导出。`,`CSV files and plots are saved automatically. MP4 export is ${c.export_video ? 'enabled' : 'disabled'}.`)
  }[id];
  return {label,detail,bypassed:['inject','restore'].includes(id) && !noisy};
}
function paperCitation(paperId, location, label) {
  const paper = boot?.literature?.[paperId], target = paper?.locations?.[location];
  if (!paper || !target) return `<span>${escapeFlow(label)}</span>`;
  const href = `/paper?paper=${encodeURIComponent(paperId)}&page=${target.page}&focus=${encodeURIComponent(location)}`;
  return `<a class="paper-citation" href="${href}" target="_blank" rel="noopener" title="${escapeFlow(target.label[language === 'en' ? 1 : 0])}">${escapeFlow(label)} ↗</a>`;
}

function noiseReferencesHtml() {
  const referenceNotes = {
    'turlej-2023': ['用于：选择时间相关 OU 感知误差。借鉴其建模思想，未照搬多维状态模型或其标定参数。', 'Used for: the choice of temporally correlated OU perception errors. The multidimensional state model and calibration parameters are not copied.'],
    'yang-2020': ['用于：指数相关系数、离散递推、平稳方差归一化。自相关标定作为后续方法参考；本文的卡尔曼估计与融合未引入本项目。', 'Used for: exponential correlation, discrete transition and stationary variance normalization. Autocorrelation identification is a future calibration reference; its Kalman estimation and fusion are not used.'],
    'piazzoni-2024': ['用于：时间因素与验证边界的说明。文中的检测状态马尔可夫链属于扩展阅读，当前没有实现随机漏检。', 'Used for: temporal considerations and validation limitations. Its detection-state Markov chain is further reading; random missed detections are not implemented.'],
    'sensi-2025': ['扩展阅读：同课题组的时间一致性误差研究方向。海报不提供本模型的 τ 或误差幅度参数。', 'Further reading: temporal consistency research by the project group. The poster does not supply τ or amplitude parameters for this model.'],
  };
  const references = Object.entries(boot?.literature || {}).map(([id,paper],index) => {
    const [location,target] = Object.entries(paper.locations)[0];
    return `<article class="paper-reference"><span class="paper-number">[${index+1}]</span><div><h4>${escapeFlow(paper.title)}</h4><p class="paper-byline">${escapeFlow(paper.authors)} · ${escapeFlow(paper.publication)}</p><p>${escapeFlow(flowText(referenceNotes[id] || ['', '']))}</p><div class="paper-actions">${paperCitation(id,location,flowLabel('打开引用位置','Open cited location'))}<a href="${escapeFlow(paper.local_pdf)}#page=${target.page}" target="_blank" rel="noopener">${flowLabel('完整 PDF','Full PDF')}</a><a href="${escapeFlow(paper.local_pdf)}" download="${escapeFlow(id)}.pdf">${flowLabel('下载','Download')}</a><a href="${escapeFlow(paper.doi || paper.url)}" target="_blank" rel="noopener">${flowLabel('出版来源','Source')}</a></div></div></article>`;
  }).join('');
  return `<section class="flow-references" aria-labelledby="flowReferencesTitle">
    <h3 id="flowReferencesTitle">${flowLabel('参考资料','REFERENCES')}</h3>
    <p class="paper-reader-hint">${flowLabel('“打开引用位置”进入本地原文阅读器并高亮关键词；完整 PDF 已下载，可离线阅读。','Open cited location displays the local original page with highlighted keywords. Full PDFs are downloaded for offline reading.')}</p>
    <div class="paper-references">${references}</div>
  </section>`;
}

function noiseExplanationHtml() {
  const citeTemporal = paperCitation('turlej-2023','temporal','[1] Turlej · p. 6');
  const citeTransition = paperCitation('yang-2020','transition','[2] Yang et al. · (3), (4), (7)');
  const citeCalibration = paperCitation('yang-2020','calibration','[2] Yang et al. · III-A');
  const citeValidation = paperCitation('piazzoni-2024','temporal','[3] Piazzoni et al. · IV-B/C');
  return `<div class="noise-lesson">
    <h4>${flowLabel('误差状态具体怎样更新','How the error state is updated')}</h4>
    <p>${flowLabel('上面第 2 步用 OU 递推保留部分旧状态，再加入新的随机变化。时间相关感知误差的建模思路参考 Turlej；离散递推与方差归一化参考 Yang 等。','Step 2 uses an OU transition to retain part of the previous state and add a random innovation. Turlej motivates temporally correlated perception errors; Yang et al. provide the discrete transition and variance normalization.')} ${citeTemporal} ${citeTransition}</p>
    <div class="noise-equation" role="math" aria-label="OU transition">
      <div>ρ = exp(−Δt / τ)</div>
      <div>z<sub>k</sub> = ρ z<sub>k−1</sub> + √(1 − ρ²) ξ<sub>k</sub></div>
      <div>ξ<sub>k</sub> ∼ N(0, 1),　z<sub>0</sub> ∼ N(0, 1)</div>
      <div>e<sub>k</sub> = σ(r<sub>k</sub>) z<sub>k</sub>，　r̂<sub>k</sub> = r<sub>k</sub> + e<sub>k</sub></div>
    </div>
    <p>${flowLabel('依据 Yang 等的公式 (3)、(4)、(7)，将误差过程归一化为单位方差，再乘以本项目的距离幅度 σ(r)。系数 √(1−ρ²) 让新加入的随机变化与保留下来的旧状态刚好配平：ρ² + (1−ρ²) = 1，所以 z 的方差始终为 1，固定距离下的误差幅度不会随时间漂移。控制器每步看到的就是加上误差后的位置，中间不做滤波。','Using Yang et al. equations (3), (4), (7), the process is normalized to unit variance and then scaled by the project envelope σ(r). The factor √(1−ρ²) balances the new random change against the retained state: ρ² + (1−ρ²) = 1, so z keeps unit variance and the error amplitude at a fixed range does not drift over time. The controller sees the perturbed position at every step, without filtering.')} ${citeTransition}</p>
    <dl class="noise-parameters"><div><dt>τ</dt><dd>${flowLabel('相关时间，单位秒。经过 τ 秒，归一化误差相关性降至 1/e ≈ 0.368。','Correlation time in seconds. After τ seconds, normalized error correlation falls to 1/e ≈ 0.368.')}</dd></div><div><dt>Δt</dt><dd>${flowLabel('相邻两次更新的间隔，即仿真步长 dt。','Interval between successive updates, i.e. the simulation step dt.')}</dd></div><div><dt>ρ</dt><dd>${flowLabel('保留系数，介于 0 和 1 之间。Δt 越短或 τ 越长，ρ 越接近 1，上一次的误差保留得越多。','Retention factor between 0 and 1. A shorter Δt or longer τ brings ρ closer to 1, so more of the previous error is kept.')}</dd></div><div><dt>ξ</dt><dd>${flowLabel('每次更新新抽取的标准正态随机数，提供新的随机变化；每个目标使用自己的随机流。','A fresh standard normal draw at each update, supplying the new random change; each target uses its own random stream.')}</dd></div><div><dt>z / σ</dt><dd>${flowLabel('z 是无量纲的持续状态；σ(r) 是以米计的理论误差标准差。','z is a dimensionless persistent state; σ(r) is the theoretical error standard deviation in metres.')}</dd></div><div><dt>e / r̂</dt><dd>${flowLabel('e 是本次测距误差（米）；r 是真实距离，r̂ 是加上误差后的观测距离。','e is this observation’s range error in metres; r is the true range and r̂ the observed range after adding the error.')}</dd></div></dl>
    <h4>${flowLabel('距离怎样决定误差幅度','How range determines error amplitude')}</h4>
    <div class="noise-equation"><div>MAE(r) = r<sub>ref</sub> · p/100 · max(0, r − r<sub>0</sub>) / (r<sub>ref</sub> − r<sub>0</sub>)</div><div>σ(r) = MAE(r) · √(π/2)</div></div>
    <p>${flowLabel('r 是两车包围盒中心的水平距离；r₀ 是零误差边界，r_ref 是参考距离，p 是该距离处的平均绝对误差百分比。边界以内误差为零，之后线性增长，超过参考距离后仍沿用同一斜率。','r is horizontal distance between bounding-box centres; r₀ is the zero-error boundary, r_ref the reference range, and p the mean absolute error percentage there. Error is zero within the boundary, then grows linearly with the same slope beyond the reference range.')}</p>
    <p class="noise-assumption">${flowLabel('线性距离规律、近距离零误差区和默认 τ = 0.5 s 是本项目可调整的试验参数，用于比较不同误差条件下的控制表现，实测标定留待后续开展。上面的 MAE 换算对应固定距离下的平稳高斯抽样；仿真中的实际统计结果还取决于车辆距离变化和有效观测筛选。','The linear distance law, near-range zero-error zone and default τ = 0.5 s are adjustable project settings for comparing control under different error conditions; calibration against measurements is future work. The MAE conversion describes stationary Gaussian samples at fixed range. Simulation statistics also depend on changing vehicle distance and observation validity selection.')}</p>
    <h4>${flowLabel('无效测距怎样处理','Handling invalid ranges')}</h4>
    <p>${flowLabel('每个目标保存独立随机流和误差状态，首次从平稳分布开始，之后每个仿真步都更新，在零误差区内也继续演化。误差加在场景中的每个非自车对象上；控制器自己决定把谁当作前车。若生成的测距为负，原始数值保存在记录中并标为无效，该步把目标放到自车后方 1000 m，使控制器不会把它当作前车。','Each target has an independent random stream and error state, starting from the stationary distribution and updated at every simulation step, including inside the zero-error zone. The error is applied to every non-ego object; the controller decides which one is its lead vehicle. A negative range keeps its raw value in the records with an invalid flag, and for that step the target is placed 1000 m behind the ego so the controller cannot take it as the lead vehicle.')}</p>
    <h4>${flowLabel('记录如何支持后续分析','How the records support further analysis')}</h4>
    <p>${flowLabel('实测误差序列可用于按距离和场景估计误差幅度、分析自相关并拟合 τ。仿真 CSV 保存了归一化误差，便于比较相同时间间隔下的相关性；结合减速时刻、车间距和碰撞结果，可以分析测距误差对车辆控制的影响。','Measured error sequences can provide error amplitude by range and scene, autocorrelation estimates and fitted τ values. Simulation CSV files store normalized errors for correlation comparisons at matched time lags. Deceleration timing, vehicle gaps and collision outcomes help assess how range errors affect control.')} ${citeCalibration} ${citeValidation}</p>
  </div>`;
}

function noiseSettingsHtml(c) {
  if (!boot || flowExact(c)) return '';
  const dt = Number(c.dt_s), tau = Number(c.noise_correlation_time_s);
  const rho = Math.exp(-dt/tau), jumpRatio = Math.sqrt(1-rho);
  const referenceMae = Number(c.noise_reference_range_m)*Number(c.noise_reference_mae_pct)/100;
  const safeNumber = (value, digits=3) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  return `<p>${escapeFlow(flowLabel(`按当前仿真步长 Δt = ${dt} s 计算，ρ ≈ ${safeNumber(rho,4)}。在参考距离 ${c.noise_reference_range_m} m 处，理论 MAE 为 ${safeNumber(referenceMae,2)} m，标准差 σ ≈ ${safeNumber(referenceMae*Math.sqrt(Math.PI/2),2)} m。`,`At the current simulation step Δt = ${dt} s, ρ ≈ ${safeNumber(rho,4)}. At reference range ${c.noise_reference_range_m} m, theoretical MAE is ${safeNumber(referenceMae,2)} m and standard deviation σ ≈ ${safeNumber(referenceMae*Math.sqrt(Math.PI/2),2)} m.`))}</p>
    <p>${escapeFlow(flowLabel(`固定距离、平稳状态下，相邻误差变化的均方根约为独立抽样时的 ${safeNumber(jumpRatio*100,1)}%。这是长期统计关系；单次结果还受随机种子、距离变化和有效观测筛选影响。`,`At fixed range in stationarity, the root-mean-square change between samples is about ${safeNumber(jumpRatio*100,1)}% of independent sampling. This is a long-run statistic; individual results also depend on the seed, changing range and validity selection.`))}</p>`;
}

function overlayExplanationHtml() {
  const steps = [
    [flowLabel('读取观测中心，并配上同一时刻的车身属性','Read observed centres and matching vehicle attributes'), flowLabel('video.py 从 truth.csv 准备车辆状态；video_overlay.py 的 ObservationOverlay 读取 observations.csv 中的观测中心，按时间与目标 ID 配上长宽、朝向和高度。观测坐标已经是车身中心，不再叠加中心偏移。','video.py prepares vehicle states from truth.csv. ObservationOverlay in video_overlay.py reads observed centres from observations.csv and matches dimensions, heading and height by timestamp and target ID. The observed coordinates already describe the body centre, so no centre offset is added.')],
    [flowLabel('用中心、长宽和朝向构造矩形','Build a rectangle from centre, dimensions and heading'), flowLabel('PerceivedBox.corners() 取 (±长/2, ±宽/2) 四个局部角点，按航向角旋转，再平移到观测中心。由此得到世界坐标中的四个角。视频模型另用 ModelToBB 对齐物理包围盒。','PerceivedBox.corners() takes four local corners (±length/2, ±width/2), rotates them by heading and translates them to the observed centre, producing world coordinates. ModelToBB separately aligns the replay mesh with its physical bounding box.')],
    [flowLabel('为每个视频时刻选择最近的观测快照','Select the latest observation for each video frame'), flowLabel('ObservationOverlay.boxes_at() 用 bisect_right() 查找不晚于当前帧的最近观测，保持该次的位置、尺寸和朝向。新观测缺失或测距无效时，该目标从快照中清除，直到再次有有效观测。','ObservationOverlay.boxes_at() uses bisect_right() to find the latest observation at or before the video frame, holding that sample’s position, dimensions and heading. A missing or invalid new reading removes the target from the snapshot until a valid observation returns.')],
    [flowLabel('投影为像素，叠加后送入视频编码','Project to pixels and send the composited frame for encoding'), flowLabel('project_top_point() 将四个角投影到 esmini 俯视画面；ObservationOverlay.draw() 用 Pillow 先画 4 像素白边，再画 2 像素黑线。处理后的像素返回 video.py，再交给 FFmpeg 编码为 video.mp4。','project_top_point() projects the four corners into the esmini top-camera image. ObservationOverlay.draw() uses Pillow for a 4 px white stroke beneath a 2 px black stroke. The composited pixels return to video.py and then go to FFmpeg for encoding as video.mp4.')]
  ];
  return `<ol class="flow-steps">${steps.map(([title,body])=>`<li><h4>${escapeFlow(title)}</h4><p>${escapeFlow(body)}</p></li>`).join('')}</ol>`;
}
function flowCodeHtml(versions, id) {
  const sources = boot?.source_links?.[id] || [];
  const cursors = sources.map(() => 0);
  let sourceIndex = 0;
  return flowText(versions).split('\n').map(line => {
    const comment = /^\s*(#|\/\/)/.test(line);
    let number = '', title = '';
    if (line.trim() && !comment) {
      // Match in source order within the registered functions. Omitted code
      // produces gaps in numbering; teaching comments have no source number.
      for (let i = sourceIndex; i < sources.length; i++) {
        const source = sources[i];
        const index = source.lines?.findIndex((text, j) => j >= cursors[i] && text.trim() === line.trim()) ?? -1;
        if (index < 0) continue;
        number = source.line + index;
        title = `${source.path}:${number}`;
        cursors[i] = index + 1;
        sourceIndex = i;
        break;
      }
      if (!number) {
        number = '—';
        title = flowLabel('暂无对应源码行号','Source line unavailable');
      }
    }
    const escaped = escapeFlow(line);
    return `<span class="flow-code-line"><span class="flow-line-number" data-line="${number}" title="${escapeFlow(title)}" aria-hidden="true"></span><span class="flow-line-text${comment ? ' flow-code-comment' : ''}">${escaped}</span></span>`;
  }).join('\n');
}
function flowSourceLinks(id, fallback) {
  const sources = boot?.source_links?.[id];
  if (!sources?.length) {
    const hint = boot?.capabilities?.open_source
      ? flowLabel('源码入口暂不可用','Source location is currently unavailable')
      : flowLabel('重新启动后台服务后可在 VS Code 中打开源码','Restart the local service to open source in VS Code');
    return `<span title="${hint}">${escapeFlow(fallback)}</span>`;
  }
  return `<span class="flow-source-links">${sources.map(source => {
    const path = source.path.replace(/\\/g,'/').split('/').map((part,index) => index === 0 && /^[a-z]:$/i.test(part) ? part : encodeURIComponent(part)).join('/');
    const href = `vscode://file/${path}:${source.line}:1`;
    const title = `${flowLabel('在 VS Code 中打开','Open in VS Code')}\n${source.path}:${source.line}`;
    const label = `${flowLabel('在 VS Code 中打开','Open in VS Code')} ${source.label}, ${flowLabel('第','line')} ${source.line} ${flowLabel('行','')}`;
    return `<a class="flow-source-link" href="${escapeFlow(href)}" data-flow-source="${escapeFlow(source.id)}" title="${escapeFlow(title)}" aria-label="${escapeFlow(label.trim())}">${escapeFlow(source.label)} <span aria-hidden="true">↗</span></a>`;
  }).join('<span class="flow-source-separator" aria-hidden="true"> / </span>')}</span>`;
}
function renderFlowDetail() {
  const c = flowConfig(), node = flowNodes.find(n => n.id === flowSelection);
  const overview = {
    title:['从场景到一次制动','From scene to braking'],
    summary:['整场实验在一个 esmini 3.8.1 进程中完成：老师的 HiDrive AEB 控制自车，前车按原轨迹行驶。Studio 在控制器每步运行前把目标挪到“看到的位置”，运行后放回真值，因此测距误差只影响控制器的判断。','The whole experiment runs in one esmini 3.8.1 process: the teacher’s HiDrive AEB drives the ego while the lead vehicle follows its recorded trajectory. Before every controller step Studio moves targets to their perceived positions and restores truth afterwards, so range error affects only the controller’s decisions.'],
    input:['配置定义实验条件和误差参数，老师场景提供道路、车辆和控制器，时钟决定步长。','Settings define the experiment and error parameters; the teacher scene provides road, vehicles and controller; the clock sets the step.'],
    output:['esmini 输出每步真值、碰撞和控制器日志；Python 整理成 CSV、指标、曲线和视频。','esmini outputs per-step truth, collisions and the controller log; Python turns them into CSV files, metrics, plots and video.'],
    upstream:['config','reference','clock'],downstream:['audit','output'],file:'hidrive_runner.py · run_hidrive',
    code: [
      `# 有误差时准备每步的 OU 样本，交给 esmini 的误差接口
noisy = c['uncertainty'] == 'distance_ou_range'
if noisy:
    write_noise_file(out / 'perception_noise.tsv', c, ego_name,
                     [s for s in base.values() if s['name'] != ego_name],
                     out / 'perception_applied.tsv', duration)
    noise_args = ['--perception_noise', out / 'perception_noise.tsv']
# …省略 esmini 参数；整场一次运行，控制器在 esmini 内部
result = subprocess.run([str(a) for a in args], cwd=out, stdout=log, stderr=subprocess.STDOUT,
# 读回真值、控制器日志和实际加入的误差
frames = read_esmini_csv(out / 'hidrive_esmini.csv')
logged = read_log_values(out / 'esmini.log')
applied = read_perception_log(out / 'perception_applied.tsv') if noisy else {}`,
      `# With range error, prepare per-step OU samples for the esmini hook.
noisy = c['uncertainty'] == 'distance_ou_range'
if noisy:
    write_noise_file(out / 'perception_noise.tsv', c, ego_name,
                     [s for s in base.values() if s['name'] != ego_name],
                     out / 'perception_applied.tsv', duration)
    noise_args = ['--perception_noise', out / 'perception_noise.tsv']
# ... esmini arguments omitted; one run, controller inside esmini.
result = subprocess.run([str(a) for a in args], cwd=out, stdout=log, stderr=subprocess.STDOUT,
# Read back truth, the controller log and the applied error.
frames = read_esmini_csv(out / 'hidrive_esmini.csv')
logged = read_log_values(out / 'esmini.log')
applied = read_perception_log(out / 'perception_applied.tsv') if noisy else {}`
    ],
    note:['阅读方式：实线跟随主流程与每步反馈；虚线表示辅助关系。控制器源码保密，图中只展示它周围的公共代码。','Solid arrows show the main flow and per-step feedback; dashed arrows show supporting relations. The controller source is confidential; only the public code around it is shown.']
  };
  const id = node?.id || 'overview';
  const item = node || overview;
  const state = node ? flowState(node.id,c) : {
    label:flowLabel(`${flowNodes.length} 个模块 · 1 个闭环`,`${flowNodes.length} modules · 1 feedback loop`),
    detail:!boot ? flowLabel('正在读取实验配置。','Loading experiment settings.') : flowExact(c) ? flowLabel('当前为无误差运行：控制器看到真实位置。','Ideal run: the controller sees true positions.') : flowLabel('当前为 OU 误差运行：控制器每步看到带误差的目标位置，真实世界不受影响。','OU error run: the controller sees perturbed target positions every step; the true world is unaffected.')
  };
  const index = flowNodes.indexOf(node);
  $('flowDetailContent').innerHTML = `
    <div class="flow-detail-kicker"><span>${node ? `MODULE ${String(index+1).padStart(2,'0')} / ${flowNodes.length}` : 'THE BIG PICTURE'}</span><span>${escapeFlow(state.label)}</span></div>
    <h2 id="flowDetailTitle">${escapeFlow(flowText(item.title))}</h2>
    <div class="flow-summary">${flowText(item.summary).split('\n\n').map(paragraph => `<p>${escapeFlow(paragraph)}</p>`).join('')}</div>
    <section class="flow-io" aria-label="${flowLabel('输入与输出','Inputs and outputs')}"><div><h3>↘ ${flowLabel('输入 · 从哪里来','INPUT · RECEIVED FROM')}</h3><p>${escapeFlow(flowText(item.input))}</p></div>
    <div><h3>↗ ${flowLabel('输出 · 传到哪里','OUTPUT · PASSED TO')}</h3><p>${escapeFlow(flowText(item.output))}</p></div></section>
    ${flowImplementationHtml(item, id)}
    <section class="flow-code" aria-labelledby="flowCodeTitle"><div class="flow-code-head"><h3 id="flowCodeTitle">${flowLabel('关键源码 · 节选','KEY SOURCE · EXCERPT')}</h3>${flowSourceLinks(id,item.file)}</div><pre><code>${flowCodeHtml(item.code,id)}</code></pre></section>
    ${id === 'noise' ? noiseReferencesHtml() : ''}
    <section class="flow-current" aria-labelledby="flowCurrentTitle"><h3 id="flowCurrentTitle">${flowLabel('当前配置下','With the current settings')}</h3><p>${escapeFlow(state.detail)}</p>${id === 'noise' ? noiseSettingsHtml(c) : ''}</section>`;
  $('flowPosition').textContent = node ? `${String(index+1).padStart(2,'0')} / ${flowNodes.length}` : flowLabel('全局概览','Overview');
  $('flowPrevious').disabled = !node;
  $('flowNext').disabled = index === flowNodes.length - 1;
  $('flowPrevious').textContent = flowLabel('← 上一步','← Previous');
  $('flowNext').textContent = flowLabel('下一步 →','Next →');
}
function drawFlowEdges() {
  if (workspaceView !== 'flow') return;
  const map = $('flowMap'), bounds = map.getBoundingClientRect(), c = flowConfig();
  const boxes = Object.fromEntries(flowNodes.map(node => {
    const r = $(`flow-node-${node.id}`).getBoundingClientRect();
    return [node.id,{x:r.x-bounds.x,y:r.y-bounds.y,w:r.width,h:r.height}];
  }));
  const selected = flowNodes.find(n => n.id === flowSelection);
  let svg = '<defs>' + ['normal','side','active'].map(id => `<marker id="flow-arrow-${id}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" markerUnits="userSpaceOnUse" orient="auto"><path class="flow-arrow-${id}" d="M1 1 L9 5 L1 9 Z"/></marker>`).join('') + '</defs>';
  for (const [from,to,route,side] of flowEdges) {
    const a = boxes[from], b = boxes[to];
    let d;
    if (route === 'horizontal') {
      const forward = a.x < b.x, x1 = a.x+(forward?a.w:0), x2 = b.x+(forward?0:b.w), y1=a.y+a.h/2,y2=b.y+b.h/2;
      d=`M${x1},${y1} C${(x1+x2)/2},${y1} ${(x1+x2)/2},${y2} ${x2},${y2}`;
    } else if (route === 'vertical') {
      const x1=a.x+a.w/2,y1=a.y+a.h,x2=b.x+b.w/2,y2=b.y;
      d=`M${x1},${y1} C${x1},${(y1+y2)/2} ${x2},${(y1+y2)/2} ${x2},${y2}`;
    } else {
      const rail = route === 'feedback' ? 20 : 8;
      const y1=a.y+a.h*(route === 'feedback' ? .35 : .7), y2=b.y+b.h*(route === 'feedback' ? .35 : .5);
      d=`M${a.x},${y1} H${rail+5} Q${rail},${y1} ${rail},${y1+(y2>y1?5:-5)} V${y2+(y2>y1?-5:5)} Q${rail},${y2} ${rail+5},${y2} H${b.x}`;
    }
    const active = selected && (from === selected.id || to === selected.id);
    const bypass = boot && flowExact(c) && [from,to].some(id=>['inject','restore'].includes(id));
    const kind = active ? 'active' : side || bypass ? 'side' : 'normal';
    svg += `<path class="flow-edge ${side || bypass ? 'side' : ''} ${active ? 'active' : selected ? 'muted' : ''}" d="${d}" marker-end="url(#flow-arrow-${kind})"/>`;
  }
  $('flowEdges').innerHTML = svg;
}
function renderFlowSoftware() {
  $('flowSoftwareTitle').textContent=flowLabel('软件与版本','Software & versions');
  $('flowSoftwareScope').textContent=flowLabel('实验环境','Experiment environment');
  // Snapshot of the experiment environment; update when dependencies change.
  const packages={numpy:'2.4.6',matplotlib:'3.10.6',pillow:'12.3.0','imageio-ffmpeg':'0.6.0'};
  $('flowSoftwareScope').title=flowLabel('运行目录记录在每次实验的 versions.json','Each run records its runtime in versions.json');
  const software=[
    ['esmini', '3.8.1', flowLabel('HiDrive 控制器与仿真（隔离编译）','HiDrive controller & simulation (isolated build)')],
    ['esmini', '3.8.1', flowLabel('视频回放渲染','Video replay rendering')],
    ['Python', '3.11.9', flowLabel('测距误差序列与实验脚本','Range-error sequences & experiment scripts')],
    ['NumPy', packages.numpy, flowLabel('数值与绘图数据处理','Numerical & plot data processing')],
    ['Matplotlib', packages.matplotlib, flowLabel('实验结果绘图','Result plotting')],
    ['Pillow', packages.pillow, flowLabel('图像处理','Image processing')],
    ['imageio-ffmpeg', packages['imageio-ffmpeg'], flowLabel('MP4 视频导出接口','MP4 video export interface')],
  ];
  $('flowSoftwareVersions').innerHTML=software.map(([name,version,purpose])=>`<div title="${escapeFlow(purpose)}"><dt>${escapeFlow(name)}</dt><dd>${escapeFlow(version || flowLabel('未检测到','Unavailable'))}</dd></div>`).join('');
}
function refreshFlow() {
  $('flowTab').textContent = flowLabel('流程分析','Flow analysis');
  $('configTitle').textContent = flowLabel('实验','Experiment');
  $('flowPageTitle').textContent = flowLabel('代码与数据流','Code & data flow');
  document.querySelector('.workspace-tabs').setAttribute('aria-label',flowLabel('工作区视图','Workspace views'));
  if (workspaceView !== 'flow') return;
  const c=flowConfig();
  renderFlowSoftware();
  $('flowMapTitle').textContent=flowLabel('一场实验，如何发生','Inside an experiment');
  $('flowMapHint').textContent=flowLabel('沿箭头读数据，点击节点看实现。','Follow the data. Select a node to explore its implementation.');
  $('flowOverview').textContent=flowLabel('全局概览','Overview');
  $('flowOverview').setAttribute('aria-pressed',String(flowSelection === 'overview'));
  $('flowMap').setAttribute('aria-label',flowLabel('仿真流程图，使用方向键切换节点','Simulation flow diagram; use arrow keys to move between nodes'));
  document.querySelector('.flow-detail-nav').setAttribute('aria-label',flowLabel('流程导览','Flow walkthrough'));
  $('flowLaneInput').textContent=flowLabel('01 / 定义实验','01 / DEFINE');
  $('flowLaneLoop').textContent=flowLabel('02 / 每步：感知误差与控制','02 / EVERY STEP · PERCEIVE · ACT');
  $('flowLaneOutput').textContent=flowLabel('03 / 校验与结果','03 / EVALUATE');
  $('flowContext').innerHTML=boot ? [scenarioName(c.scenario,c.scenario),controllerName(c.controller),processingName(c)].map(label=>`<span>${escapeFlow(label)}</span>`).join('') : '';
  $('flowLegend').innerHTML=`<span><i></i>${flowLabel('数据与每步反馈','Data & per-step feedback')}</span><span><i class="branch"></i>${flowLabel('辅助 / 未启用','Support / inactive')}</span><span><i class="selection"></i>${flowLabel('选中模块','Selected module')}</span>`;
  $('flowFootnote').textContent=flowLabel('基于当前表单 · 非实时执行动画。无误差运行中，误差接口不起作用。','Based on the current form, not live execution. In ideal runs the noise hook is inactive.');
  const selected=flowNodes.find(n=>n.id===flowSelection);
  for (const [index,node] of flowNodes.entries()) {
    let button=$(`flow-node-${node.id}`);
    if (!button) {
      button=document.createElement('button'); button.type='button'; button.id=`flow-node-${node.id}`;
      button.className='flow-node'; button.dataset.flowNode=node.id; button.dataset.tone=node.tone;
      button.style.gridColumn=node.col; button.style.gridRow=node.row;
      button.setAttribute('aria-controls','flowDetailContent');
      $('flowMap').appendChild(button);
    }
    const state=flowState(node.id,c), related=selected && [...selected.upstream,...selected.downstream].includes(node.id);
    button.classList.toggle('related',Boolean(related));button.classList.toggle('bypassed',Boolean(state.bypassed));
    button.setAttribute('aria-pressed',String(flowSelection===node.id));
    button.innerHTML=`<span class="flow-node-top"><span class="flow-node-icon" aria-hidden="true">${node.icon}</span><span class="flow-node-index">${String(index+1).padStart(2,'0')}</span></span><strong>${escapeFlow(flowText(node.title))}</strong><small>${escapeFlow(state.label || flowText(node.sub))}</small>`;
  }
  renderFlowDetail();
  requestAnimationFrame(drawFlowEdges);
}
function selectFlowNode(id) {
  if (id !== 'overview' && !flowNodes.some(n=>n.id===id)) return;
  flowSelection=id; refreshFlow(); $('flowDetailContent').scrollTop=0;
}
function setWorkspaceView(view) {
  workspaceView=view;
  const analysis=view==='flow';
  document.querySelector('.workspace').classList.toggle('flow-mode',analysis);
  document.querySelector('.configuration').setAttribute('aria-labelledby',analysis?'flowTab':'configTitle');
  $('configForm').hidden=analysis; $('flowPanel').hidden=!analysis;
  $('flowDetail').hidden=!analysis; $('flowPageTitle').hidden=!analysis;
  document.querySelectorAll('[data-workspace-tab]').forEach(button=>{
    const selected=button.dataset.workspaceTab===view;
    button.setAttribute('aria-selected',String(selected));button.tabIndex=selected?0:-1;
  });
  for (const id of ['scenarioInfo','distanceNoiseInfo']) if ($(id).matches(':popover-open')) $(id).hidePopover();
  if (analysis) {
    $('videoView').pause();
    $('compareViewer').querySelectorAll('video').forEach(video => video.pause());
    refreshFlow();
  }
}
const workspaceTabs=[...document.querySelectorAll('[data-workspace-tab]')];
workspaceTabs.forEach(button=>{
  button.addEventListener('click',()=>setWorkspaceView(button.dataset.workspaceTab));
  button.addEventListener('keydown',event=>{
    if (!['ArrowLeft','ArrowRight','Home','End'].includes(event.key) || event.ctrlKey || event.metaKey || event.altKey) return;
    event.preventDefault();const index=event.key==='Home'?0:event.key==='End'?1:1-workspaceTabs.indexOf(button);
    workspaceTabs[index].focus();workspaceTabs[index].click();
  });
});
$('flowMap').addEventListener('click',event=>{
  const node=event.target.closest('[data-flow-node]');
  if (node) {
    selectFlowNode(node.dataset.flowNode);
    if (matchMedia('(max-width: 820px)').matches) $('flowDetail').scrollIntoView({block:'start'});
  }
});
$('flowMap').addEventListener('keydown',event=>{
  const button=event.target.closest('[data-flow-node]');
  if (!button || !['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','Home','End'].includes(event.key)) return;
  event.preventDefault();const index=flowNodes.findIndex(n=>n.id===button.dataset.flowNode);
  const next=event.key==='Home'?0:event.key==='End'?flowNodes.length-1:(index+(event.key==='ArrowLeft'||event.key==='ArrowUp'?-1:1)+flowNodes.length)%flowNodes.length;
  selectFlowNode(flowNodes[next].id);$(`flow-node-${flowNodes[next].id}`).focus();
});
let openingFlowSource = false;
$('flowDetailContent').addEventListener('click',async event=>{
  const source=event.target.closest('[data-flow-source]');
  if (source) {
    event.preventDefault();
    if (openingFlowSource) return;
    openingFlowSource=true;
    source.setAttribute('aria-busy','true');
    try { await api('/api/open-source',{source_id:source.dataset.flowSource}); }
    catch (error) { toast(`${flowLabel('无法打开源码：','Could not open source: ')}${error.message}`); }
    finally { openingFlowSource=false; source.removeAttribute('aria-busy'); }
    return;
  }
});
$('flowOverview').onclick=()=>selectFlowNode('overview');
$('flowPrevious').onclick=()=>{const index=flowNodes.findIndex(n=>n.id===flowSelection);selectFlowNode(index<=0?'overview':flowNodes[index-1].id);};
$('flowNext').onclick=()=>{const index=flowNodes.findIndex(n=>n.id===flowSelection);if(index<flowNodes.length-1)selectFlowNode(flowNodes[index+1].id);};
new ResizeObserver(()=>requestAnimationFrame(drawFlowEdges)).observe($('flowMap'));

$('themeToggle').onclick = () => setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');
document.querySelectorAll('[data-language]').forEach(button => button.onclick = () => setLanguage(button.dataset.language));
setLanguage(language);
if (location.hash === '#flow' || location.hash === '#flow/noise') {
  if (location.hash === '#flow/noise') flowSelection = 'noise';
  setWorkspaceView('flow');
}

(async () => {
  $('runButton').disabled = true;
  try {
    boot = await api('/api/bootstrap');
    const groups = {};
    for (const scenario of boot.scenarios) {
      const category = scenario.category || 'car';
      if (!groups[category]) {
        const group = document.createElement('optgroup');
        group.dataset.category = category;
        group.label = t(`category.${category}`);
        $('scenario').appendChild(group);
        groups[category] = group;
      }
      const option = document.createElement('option');
      option.value = scenario.id;
      option.textContent = scenarioName(scenario.id, scenario.title);
      groups[category].appendChild(option);
    }
    setConfig(boot.default);
    const runs = await refreshHistory();
    initialized = true;
    setBusy(Boolean(activeRun));
    if (runs.length) await selectRun(activeRun || runs[0].run_id);
  } catch (error) { setFormError(error.message, 'connectionFailed'); }
})();
