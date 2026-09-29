const $ = id => document.getElementById(id);
let boot, selectedRun = null, current = null, activeRun = null, tab = 'video', timer = null, toastTimer = null;
let language = 'zh', historyRuns = [], busy = false, serviceStopped = false, initialized = false, formError = null;
try { if (localStorage.getItem('simulation-studio.language') === 'en') language = 'en'; } catch { /* Storage may be unavailable in private browsing. */ }
const messages = {
  'configProcessingAuto': ['界面已按高斯误差模型自动启用 Kalman 距离估计。', 'The interface enabled Kalman range estimation for the Gaussian noise model.'],
  'rawProcessing': ['直接使用观测（未滤波）', 'Raw observations (unfiltered)'],
  'kalmanProcessing': ['卡尔曼滤波', 'Kalman filtering'],
  'exactProcessing': ['准确距离（无需滤波）', 'Exact range (filter bypassed)'],
  'rawProcessingHelp': ['本次未使用卡尔曼滤波；图中 estimated 与 observed 重合，表示直接采用原始观测。', 'This run used no Kalman filtering. The estimated and observed curves overlap because raw observations were used directly.'],
  'removedNoiseConfig': ['配置中的“固定参数高斯误差”已从界面移除，请选择现有误差模型后重新保存配置。', 'Fixed-parameter Gaussian error has been removed from the interface. Choose an available error model and save a new configuration.'],
  'removedControllerConfig': ['配置中的“无制动 · 恒速对照”已从界面移除，请选择现有控制器后重新保存配置。', 'No braking · constant speed has been removed from the interface. Choose an available controller and save a new configuration.'],
  'observation_period_s': ['观测更新周期', 'Observation interval'],
  'legacyClock': ['跟随仿真步长 · 旧版', 'Every simulation step · legacy'],
  'kalmanParameters': ['估计器参数', 'Estimator parameters'],
  'kalman_noise_mode': ['测距噪声假设', 'Range noise assumption'],
  'predictedNoiseOption': ['随预测距离调整', 'Adapt to predicted range'],
  'fixedNoiseOption': ['固定标准差 · 对照', 'Fixed standard deviation · baseline'],
  'kalman_range_sigma_m': ['固定标准差 / 初始不确定性下限', 'Fixed sigma / minimum initial uncertainty'],
  'kalman_accel_sigma_mps2': ['运动变化标准差', 'Motion acceleration standard deviation'],
  'kalman_warmup_s': ['跟踪建立时间', 'Track warmup time'],
  'kalmanHint': ['随距离调整模式使用预测距离和预测不确定性，沿用上方误差曲线，不读取真实距离；标准差至少 0.1 m。此处数值仅作为初始不确定性下限，固定模式则全程使用。目标速度、方位仍假定准确；负测距筛选偏差未消除。无误差模式直接使用准确距离。', 'Adaptive mode uses predicted range and uncertainty with the noise curve above, never true range; minimum sigma is 0.1 m. The value here bounds initial uncertainty only; fixed mode uses it throughout. Velocity and bearing remain ideal; rejection bias is not corrected. Noise-free modes bypass filtering.'],
  'aebFollowOption': ['AEB · 原始轨迹接管', 'AEB · take over original motion'],
  'recommendedAeb': ['推荐 AEB 设置', 'Recommended AEB setup'],
  'aeb_trigger_mode': ['触发方式', 'Trigger policy'],
  'confirmedOption': ['持续确认后制动', 'Confirm sustained risk'],
  'instantOption': ['单帧触发并保持 · 对照', 'Single-hit latch · baseline'],
  'aeb_confirmation_s': ['危险持续确认', 'Risk confirmation time'],
  'followHint': ['介入前保留原有减速；实际开始制动后，由 AEB 沿原路径更新车速，替代原驾驶行为。', 'Retain original deceleration until braking starts; AEB then replaces driver speed control along the original path.'],
  'egoFollowHint': ['轨迹接管使用原始初速和航向，实际制动前保留原驾驶过程。', 'Trajectory takeover uses original initial conditions and driver motion until AEB braking.'],
  'assessmentTitle': ['制动与感知诊断', 'Braking and perception diagnostics'],
  'brakeAdvance': ['相对理想感知提前制动', 'Braking advance vs ideal range'],
  'brakeRange': ['开始制动时真实中心距离', 'True centre range at brake onset'],
  'rangeRmse': ['同帧 RMSE：观测 → 估计', 'Matched RMSE: observed → estimated'],
  'confirmationCancelled': ['取消的待确认事件', 'Cancelled confirmation attempts'],
  'assessmentHelp': ['正值表示比独立的理想感知对照更早制动，负值表示更晚；提前制动不直接等于误制动。RMSE 仅比较同一批有效、跟踪已建立的时刻。', 'Positive advance means earlier braking than an independent ideal-range run; negative means later. Early braking is not automatically a false positive. RMSE uses matched valid frames after track warmup.'],
  'unmatchedBrake': ['本次制动，理想感知对照在时限内未制动；需结合场景判断原因。', 'This run braked while the ideal-range baseline did not within the horizon; interpret in context.'],
  'presetApplied': ['已应用设置；误差模型与参数保持当前选择。', 'Setup applied; the current noise model and parameters are retained.'],
  'language': ['界面语言', 'Interface language'],
  'preview': ['实验预览', 'Experiment preview'],
  'ready': ['就绪', 'Ready'],
  'metrics': ['本次实验指标', 'Current experiment metrics'],
  'collision': ['碰撞结果', 'Collision outcome'],
  'impact': ['碰撞时自车速度', 'Ego speed at impact'],
  'braking': ['AEB 开始制动', 'AEB braking onset'],
  'history': ['历史记录', 'History'],
  'refresh': ['刷新', 'Refresh'],
  'refreshHistory': ['刷新历史记录', 'Refresh history'],
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
  'emptyDescription': ['在实验设置中调整参数，点击「运行实验」。', 'Adjust the experiment settings, then select Run experiment.'],
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
  'headingHint': ['初速由轨迹前 0.20 秒估算，航向读取首帧（世界坐标）。', 'Initial speed is derived from the first 0.20 s; heading comes from the first frame in world coordinates.'],
  'egoReplayHint': ['原始回放使用文件初始状态；切换到“AEB · 恒速起步”后可调整。', 'Original replay uses the source initial state. Select AEB · constant-speed approach to edit it.'],
  'egoEditableHint': ['可调整初始状态；“AEB · 恒速起步”保持初始航向。', 'Initial state is editable. AEB · constant-speed approach holds this initial heading.'],
  'step3': ['感知条件', 'Perception'],
  'uncertainty': ['误差模型', 'Error model'],
  'idealOption': ['理想观测 · 无误差', 'Ideal · no range error'],
  'recordedOption': ['理想探测 · 不附加误差', 'Ideal detection · no added error'],
  'nativeSensor': ['esmini 理想传感器', 'esmini ideal sensor'],
  'sensorPipeline': ['esmini 理想传感器 → 误差模型 → 距离估计 → 控制器', 'esmini ideal sensor → uncertainty → range estimation → controller'],
  'noiseOption': ['固定参数高斯误差', 'Fixed-parameter Gaussian error'],
  'distanceNoiseOption': ['随距离增大的高斯误差', 'Distance-dependent Gaussian error'],
  'noise_zero_range_m': ['零误差距离', 'Zero-error distance'],
  'noise_reference_range_m': ['参考距离', 'Reference distance'],
  'noise_reference_mae_pct': ['参考处平均绝对误差', 'Mean absolute error at reference'],
  'distanceNoiseHint': ['≤ {zero} m 无误差；{reference} m 处平均绝对误差 {mae} m（σ ≈ {sigma} m），正负对称。', 'No error at ≤ {zero} m. At {reference} m: mean absolute error {mae} m (σ ≈ {sigma} m), symmetric about zero.'],
  'noiseAnchorInvalid': ['误差参考距离必须大于零误差距离。', 'Reference distance must exceed the zero-error distance.'],
  'range_bias_m': ['偏差 μ', 'Bias μ'],
  'range_sigma_m': ['标准差 σ', 'Std. deviation σ'],
  'sensor_range_m': ['感知范围', 'Sensor range'],
  'sensor_fov_deg': ['视场角', 'Field of view'],
  'seed': ['随机种子', 'Random seed'],
  'perceptionHelp': ['感知说明', 'Perception details'],
  'perceptionHelpText': ['所有模式先由 esmini 理想传感器探测：安装在自车中心、朝向车头，按目标参考点判断范围和水平视场。探测后转换为两车中心距离，再加入所选误差。距离相关 MAE 从零误差边界线性增长，超过参考距离继续增长。每个观测周期独立抽样；负测距保留在 CSV 中，但不送给 AEB。零误差模式也受探测范围限制。不模拟遮挡、随机漏检或速度误差。', 'All modes use the esmini ideal sensor, mounted at the ego centre facing forward. Native range and horizontal FOV gates test target reference points. Detected states are converted to centre-to-centre range before adding uncertainty. Distance-dependent MAE grows linearly beyond the zero-error boundary and reference distance. Each observation tick samples independently; negative ranges are logged but excluded from AEB. Noise-free mode also uses detection gates. No occlusion, random missed detections or velocity noise.'],
  'step4': ['控制策略', 'Control strategy'],
  'controller': ['控制器', 'Controller'],
  'replayOption': ['原始轨迹回放 · 默认', 'Original trajectory replay · default'],
  'replayHint': ['两辆车按原始轨迹回放，保留原有减速和方向变化；感知设置仅影响观测曲线。', 'Both vehicles follow their source trajectories, including deceleration and heading changes. Perception settings affect observation plots only.'],
  'aebOption': ['AEB · 恒速起步', 'AEB · constant-speed approach'],
  'constantOption': ['无制动 · 恒速对照', 'No braking · constant speed'],
  'ttc_threshold_s': ['触发阈值', 'Trigger threshold'],
  'brake_decel_mps2': ['减速度', 'Deceleration'],
  'reaction_delay_s': ['响应延迟', 'Response delay'],
  'constantHint': ['保持初始速度，用于对照制动效果。', 'Hold the initial speed as a baseline for comparison.'],
  'modelHelp': ['模型说明', 'Model details'],
  'modelHelpText': ['默认模式直接回放记录；文件引用的 HiDriveController 未提供实现。AEB 是未标定的规则原型。持续确认模式要求同一目标在新观测中持续危险；确认及响应期间可取消，实际制动后保持至停车。', 'Default mode replays the recorded motion; the referenced HiDriveController implementation is not supplied. AEB is an uncalibrated prototype. Sustained-risk mode confirms the same target on fresh observations and can cancel before braking; actual braking holds to stop.'],
  'step5': ['运行输出', 'Run & output'],
  'duration_s': ['数据记录时长', 'Recorded duration'],
  'dt_s': ['时间步长', 'Time step'],
  'exportVideo': ['导出 MP4 视频', 'Export MP4 video'],
  'outputHint': ['运行至数据最后一帧（9.98 秒），记录首次碰撞；曲线和 CSV 自动保存。', 'Run through the last recorded frame (9.98 s), recording the first collision. Plots and CSV files are saved automatically.'],
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
  'noiseShort': ['高斯误差', 'Gaussian noise'],
  'distanceNoiseShort': ['距离相关高斯误差', 'Distance-dependent Gaussian'],
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
function controllerName(value) { return value === 'original_trajectory' ? t('replayShort') : value === 'aeb_follow' ? t('aebFollowOption') : value === 'aeb' ? t('aebOption') : t('constantShort'); }
function perceptionName(value, result) {
  const native = result?.sensor?.backend === 'esmini_ideal_object_sensor';
  const label = t(value === 'recorded_truth' ? (native ? 'idealShort' : 'recordedShort') : value === 'distance_gaussian_range' ? 'distanceNoiseShort' : value === 'gaussian_range' ? 'noiseShort' : 'idealShort');
  return native ? `${t('nativeSensor')} · ${label}` : label;
}
function processingName(config) {
  const exact = ['recorded_truth','ideal'].includes(config.uncertainty) ||
    (config.uncertainty === 'distance_gaussian_range' && Number(config.noise_reference_mae_pct) === 0) ||
    (config.uncertainty === 'gaussian_range' && Number(config.range_sigma_m) === 0 && Number(config.range_bias_m) === 0);
  return t(exact ? 'exactProcessing' : config.observation_filter === 'kalman' ? 'kalmanProcessing' : 'rawProcessing');
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

function setLanguage(value) {
  language = value === 'en' ? 'en' : 'zh';
  try { localStorage.setItem('simulation-studio.language', language); } catch { /* The switch also works without storage. */ }
  document.documentElement.lang = language === 'en' ? 'en' : 'zh-CN';
  document.querySelectorAll('[data-i18n]').forEach(element => element.textContent = t(element.dataset.i18n));
  for (const attribute of ['aria-label', 'title', 'alt']) {
    document.querySelectorAll(`[data-i18n-${attribute}]`).forEach(element => element.setAttribute(attribute, t(element.getAttribute(`data-i18n-${attribute}`))));
  }
  document.querySelectorAll('[data-language]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.language === language)));
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
  if (!response.ok) throw Error(result.error || messages.requestFailed[0]);
  return result;
}

function getConfig() {
  const config = {};
  for (const [key, value] of Object.entries(boot.default)) {
    if (key === 'observation_filter') continue;
    config[key] = typeof value === 'boolean' ? $(key).checked : typeof value === 'number' ? Number($(key).value) : $(key).value;
  }
  config.observation_filter = config.uncertainty === 'distance_gaussian_range' ? 'kalman' : 'raw';
  return config;
}

function setConfig(config) {
  config = {kalman_noise_mode:'fixed', ...config};
  if (!boot.scenarios.some(s => s.id === config.scenario)) throw Error(messages.unknownScene[0]);
  if (config.uncertainty === 'gaussian_range') throw Error(messages.removedNoiseConfig[0]);
  if (config.controller === 'constant_speed') throw Error(messages.removedControllerConfig[0]);
  if (config.uncertainty === 'ideal') config = {...config, uncertainty:'recorded_truth'};
  const scene = boot.scenarios.find(s => s.id === config.scenario);
  config = {...config, ego_speed_kph:config.ego_speed_kph ?? scene.speed, ego_heading_deg:config.ego_heading_deg ?? scene.heading_deg};
  for (const key of Object.keys(boot.default)) {
    if (key === 'observation_filter') continue;
    if (key in config) {
      if (typeof boot.default[key] === 'boolean') $(key).checked = config[key];
      else $(key).value = config[key];
    }
  }
  updateControls();
  updateScenario(false);
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
  const fixedNoise = $('uncertainty').value === 'gaussian_range';
  const distanceNoise = $('uncertainty').value === 'distance_gaussian_range';
  const noisy = fixedNoise || distanceNoise;
  const follow = $('controller').value === 'aeb_follow';
  const aeb = $('controller').value === 'aeb' || follow;
  const replay = $('controller').value === 'original_trajectory';
  toggleFields(['range_bias_m', 'range_sigma_m'], fixedNoise);
  toggleFields(['noise_zero_range_m','noise_reference_range_m','noise_reference_mae_pct'], distanceNoise);
  toggleFields(['seed'], noisy);
  toggleFields(['sensor_range_m', 'sensor_fov_deg'], true);
  toggleFields(['ttc_threshold_s', 'brake_decel_mps2', 'reaction_delay_s','aeb_trigger_mode'], aeb);
  toggleFields(['aeb_confirmation_s'], aeb && $('aeb_trigger_mode').value === 'confirmed');
  toggleFields(['kalman_noise_mode','kalman_range_sigma_m','kalman_accel_sigma_mps2','kalman_warmup_s'],noisy);
  $('kalmanFields').hidden = !noisy;
  toggleFields(['observation_period_s'],true);
  $('observationPeriodField').hidden = false;
  $('confirmationField').hidden = $('aeb_trigger_mode').value !== 'confirmed';
  $('noiseFields').hidden = !fixedNoise;
  $('distanceNoiseFields').hidden = !distanceNoise;
  $('seedField').hidden = !noisy;
  $('aebFields').hidden = !aeb;
  $('constantSpeedHint').hidden = $('controller').value !== 'constant_speed';
  $('replayHint').hidden = !replay;
  $('followHint').hidden = !follow;
  $('ego_speed_kph').readOnly = replay || follow;
  $('ego_heading_deg').readOnly = replay || follow;
  $('egoModeHint').textContent = t(replay ? 'egoReplayHint' : follow ? 'egoFollowHint' : 'egoEditableHint');
  if (replay || follow) fillInitialState();
  updateNoiseProfile();
}

function updateNoiseProfile() {
  const zero = Number($('noise_zero_range_m').value);
  const reference = Number($('noise_reference_range_m').value);
  const pct = Number($('noise_reference_mae_pct').value);
  const enabled = $('uncertainty').value === 'distance_gaussian_range';
  $('noise_reference_range_m').setCustomValidity(enabled && reference <= zero ? t('noiseAnchorInvalid') : '');
  const mae = reference*pct/100;
  $('distanceNoiseHint').textContent = t('distanceNoiseHint', {zero,reference,mae:mae.toFixed(2),sigma:(mae*Math.sqrt(Math.PI/2)).toFixed(2)});
}

function fillInitialState() {
  const scenario = boot.scenarios.find(s => s.id === $('scenario').value);
  if (!scenario) return;
  $('ego_speed_kph').value = scenario.speed.toFixed(2);
  $('ego_heading_deg').value = scenario.heading_deg == null ? '' : scenario.heading_deg.toFixed(2);
  $('ego_heading_deg').required = scenario.heading_deg != null;
}

function updateScenario(resetSpeed) {
  const scenario = boot.scenarios.find(s => s.id === $('scenario').value);
  if (!scenario) return;
  $('duration_s').value = scenario.duration_s.toFixed(2);
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
}

function setBusy(isBusy) {
  busy = isBusy;
  $('runButton').disabled = busy || !initialized || serviceStopped;
  $('runButton').replaceChildren(document.createTextNode(serviceStopped ? t('serviceStopped') : busy ? t('runningButton') : t('run')));
  if (!busy && !serviceStopped) {
    const arrow = document.createElement('span');
    arrow.setAttribute('aria-hidden', 'true');
    arrow.textContent = '→';
    $('runButton').appendChild(arrow);
  }
  $('cancelButton').hidden = !busy;
  $('cancelButton').disabled = !activeRun;
  $('shutdown').disabled = serviceStopped;
}

function renderView() {
  const files = current?.files || [];
  const base = selectedRun ? `/runs/${selectedRun}/` : '';
  const wanted = tab === 'video' ? 'video.mp4' : `${tab}.png`;
  const video = $('videoView');
  video.hidden = true;
  $('plotView').hidden = true;
  $('emptyView').hidden = true;
  if (files.includes(wanted) && (tab !== 'video' || current.video_status === 'ready')) {
    if (tab === 'video') {
      if (video.getAttribute('src') !== base + wanted) {
        video.src = base + wanted;
        video.load();
      }
      video.hidden = false;
    } else {
      video.pause();
      $('plotView').src = base + wanted;
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
  const scenario = boot.scenarios.find(s => s.id === config.scenario);
  const complete = isComplete(result);
  $('resultTitle').textContent = scenarioName(config.scenario, scenario?.title || result.run_id);
  $('statusBadge').textContent = statusName(result.status);
  $('statusBadge').className = 'status ' + (isRunning(result) ? 'running' : ['failed','cancelled','interrupted'].includes(result.status) ? 'bad' : '');
  $('collisionMetric').textContent = complete ? t(result.collision ? 'hit' : 'safe') : '—';
  $('collisionCard').className = 'metric collision-card' + (complete ? (result.collision ? ' hit' : ' safe') : '');
  metric('impactMetric', result.collision_details ? result.collision_details.ego_speed_kph.toFixed(1) : '—', result.collision_details ? 'km/h' : '');
  metric('brakeMetric', config.controller === 'original_trajectory' && complete ? t('notApplicable') : result.first_brake_time_s != null ? result.first_brake_time_s.toFixed(2) : complete ? t('notTriggered') : '—', result.first_brake_time_s != null ? 's' : '');
  const assessment = result.assessment;
  $('assessmentPanel').hidden = !complete || !assessment;
  if (assessment) {
    const value = (n,unit='') => n == null ? '—' : `${Number(n).toFixed(2)}${unit}`;
    $('advanceMetric').textContent = value(assessment.brake_advance_vs_ideal_s,' s');
    $('brakeRangeMetric').textContent = value(assessment.true_center_range_at_brake_m,' m');
    $('rmseMetric').textContent = `${value(assessment.matched_observed_range_rmse_m)} → ${value(assessment.estimated_range_rmse_m)} m`;
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
  for (const [id, file] of [['csvLink','ego.csv'],['obsLink','observations.csv'],['truthLink','truth.csv'],['summaryLink','summary.json'],['videoLink','video.mp4']]) {
    const link = $(id);
    link.href = `/runs/${result.run_id}/${file}`;
    link.download = file;
    link.hidden = !(result.files || []).includes(file) || (file === 'video.mp4' && result.video_status !== 'ready');
  }
  renderView();
}

function highlightHistory() {
  document.querySelectorAll('.history-row').forEach(row => {
    const selected = row.dataset.id === selectedRun;
    row.classList.toggle('active', selected);
    row.setAttribute('aria-current', String(selected));
  });
}

async function selectRun(id) {
  selectedRun = id;
  $('videoView').pause();
  highlightHistory();
  try {
    const result = await api(`/api/run/${id}`);
    if (selectedRun === id) showResult(result);
  } catch (error) { toast(error.message); }
}

async function refreshHistory() {
  const runs = await api('/api/runs');
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

function renderHistory(runs) {
  const list = $('history');
  const scrollTop = list.scrollTop, scrollLeft = list.scrollLeft;
  list.replaceChildren();
  $('historyEmpty').hidden = runs.length > 0;
  $('historyCount').textContent = runs.length;
  for (const result of runs) {
    const button = document.createElement('button');
    button.className = 'history-row';
    button.dataset.id = result.run_id;
    const title = document.createElement('b');
    const meta = document.createElement('small');
    const bottom = document.createElement('div');
    const date = document.createElement('time');
    const outcome = document.createElement('span');
    title.textContent = scenarioName(result.config?.scenario, boot.scenarios.find(s => s.id === result.config?.scenario)?.title || result.run_id);
    meta.textContent = `${controllerName(result.config?.controller)} · ${perceptionName(result.config?.uncertainty,result)} · ${result.config?.observation_filter === 'kalman' ? 'Kalman · ' : ''}${result.config?.ego_speed_kph != null ? Number(result.config.ego_speed_kph).toFixed(2) : '—'} km/h`;
    if (result.created_utc) {
      const created = new Date(result.created_utc);
      date.dateTime = result.created_utc;
      date.textContent = created.toLocaleString(language === 'en' ? 'en-GB' : 'zh-CN', {month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',hour12:false});
      button.title = `${result.run_id}\n${created.toLocaleString(language === 'en' ? 'en-GB' : 'zh-CN', {hour12:false})}`;
    }
    outcome.className = 'outcome' + (isComplete(result) ? (result.collision ? ' hit' : '') : ' pending');
    outcome.textContent = isComplete(result) ? t(result.collision ? 'hit' : 'safe') : statusName(result.status);
    bottom.className = 'history-bottom';
    bottom.append(date, outcome);
    button.append(title, meta, bottom);
    button.onclick = () => selectRun(result.run_id);
    list.appendChild(button);
  }
  highlightHistory();
  list.scrollTop = scrollTop;
  list.scrollLeft = scrollLeft;
}

function startPolling() {
  if (timer) return;
  timer = setInterval(async () => {
    if (!activeRun) { clearInterval(timer); timer = null; return; }
    try {
      const result = await api(`/api/run/${activeRun}`);
      if (selectedRun === activeRun) showResult(result);
      if (!isRunning(result)) {
        activeRun = null;
        setBusy(false);
        clearInterval(timer);
        timer = null;
        await refreshHistory();
      }
    } catch (error) { /* A newly launched worker may not have created its directory yet. */ }
  }, 800);
}

$('configForm').onsubmit = async event => {
  event.preventDefault();
  if ($('runButton').disabled) return;
  setFormError();
  setBusy(true);
  try {
    const config = getConfig();
    const result = await api('/api/run', config);
    activeRun = result.run_id;
    selectedRun = result.run_id;
    showResult({run_id:result.run_id, status:'starting', config, files:[], video_status:config.export_video ? 'pending' : 'not_requested'});
    setBusy(true);
    highlightHistory();
    startPolling();
    setTimeout(() => refreshHistory().catch(() => {}), 1000);
  } catch (error) {
    setFormError(error.message);
    setBusy(Boolean(activeRun));
  }
};

$('scenario').onchange = () => updateScenario(true);
$('uncertainty').onchange = updateControls;
for (const id of ['noise_zero_range_m','noise_reference_range_m','noise_reference_mae_pct']) $(id).oninput = updateNoiseProfile;
$('controller').onchange = updateControls;
$('aeb_trigger_mode').onchange = updateControls;
$('recommendedAeb').onclick = () => {
  setConfig({...getConfig(),controller:'aeb_follow',observation_period_s:.05,aeb_trigger_mode:'confirmed',aeb_confirmation_s:.15,kalman_warmup_s:.15});
  toast(t('presetApplied'));
};
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
    for (const key of Object.keys(config)) if (!(key in boot.default)) throw Error(`不支持的配置项：${key}`);
    const scene = boot.scenarios.find(s => s.id === (config.scenario || boot.default.scenario));
    setConfig({...boot.default, observation_filter:'raw', observation_period_s:0,aeb_trigger_mode:'instant',kalman_noise_mode:'fixed',ego_speed_kph:scene?.speed, ego_heading_deg:scene?.heading_deg, ...config});
    const processingChanged = getConfig().uncertainty === 'distance_gaussian_range' && config.observation_filter !== 'kalman';
    toast(t('configLoaded') + (processingChanged ? ` ${t('configProcessingAuto')}` : ''));
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
function positionScenarioInfo() {
  const popup = $('scenarioInfo');
  if (!popup.matches(':popover-open')) return;
  const anchor = $('scenarioInfoButton').getBoundingClientRect();
  const bounds = popup.getBoundingClientRect();
  const inset = 16;
  popup.style.left = `${Math.max(inset, Math.min(anchor.right - bounds.width, innerWidth - bounds.width - inset))}px`;
  const below = anchor.bottom + 8;
  popup.style.top = `${below + bounds.height <= innerHeight - inset ? below : Math.max(inset, anchor.top - bounds.height - 8)}px`;
}

$('scenarioInfo').addEventListener('toggle', () => {
  $('scenarioInfoButton').setAttribute('aria-expanded', String($('scenarioInfo').matches(':popover-open')));
  positionScenarioInfo();
});
function closeScenarioInfo() {
  if ($('scenarioInfo').matches(':popover-open')) $('scenarioInfo').hidePopover();
}
$('configForm').addEventListener('scroll', closeScenarioInfo, {passive:true});
window.addEventListener('scroll', closeScenarioInfo, {passive:true});
window.addEventListener('resize', positionScenarioInfo);

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

document.querySelectorAll('[data-language]').forEach(button => button.onclick = () => setLanguage(button.dataset.language));
setLanguage(language);

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
