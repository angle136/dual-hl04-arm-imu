function arm_imu_raw_pose
%ARM_IMU_RAW_POSE Display two HL04 poses from J-Link RTT Channel 0.

if ~ispc
    error('arm_imu_live_pose:WindowsOnly', ...
        '当前 J-Link 一键启动仅支持 Windows。');
end

settings.device = 'STM32F407IG';
settings.interface = 'SWD';
settings.speedKHz = 4000;
settings.poseScale = 1000000;
settings.upperLinkLength = 0.204;
settings.forearmLinkLength = 0.120;
settings.noPoseTimeoutSeconds = 15;

assertProbeIsFree();
loggerExe = findRttLogger();
logPath = [tempname, '.log'];
logStream = [];
logReader = [];
figureHandle = [];

loggerArgs = sprintf('-Device %s -If %s -Speed %d -RTTChannel 0 "%s"', ...
    settings.device, settings.interface, settings.speedKHz, logPath);
startInfo = System.Diagnostics.ProcessStartInfo();
startInfo.FileName = loggerExe;
startInfo.Arguments = loggerArgs;
startInfo.UseShellExecute = false;
startInfo.CreateNoWindow = true;
% JLinkRTTLogger 会持续输出状态；不重定向，避免管道写满后停止读 RTT。
startInfo.RedirectStandardOutput = false;
startInfo.RedirectStandardError = false;
loggerProcess = System.Diagnostics.Process.Start(startInfo);

try
    waitStart = tic;
    openMessage = 'RTT 输出文件尚未创建。';
    while toc(waitStart) < 10
        if loggerProcess.HasExited
            error('arm_imu_live_pose:LoggerExited', ...
                'JLinkRTTLogger 连接前已退出，请检查 J-Link 占用和板子供电。');
        end
        if isfile(logPath)
            try
                logStream = System.IO.FileStream(logPath, ...
                    System.IO.FileMode.Open, System.IO.FileAccess.Read, ...
                    System.IO.FileShare.ReadWrite);
                logReader = System.IO.StreamReader(logStream);
                break;
            catch openException
                openMessage = openException.message;
            end
        end
        pause(0.05);
    end
    if isempty(logReader)
        error('arm_imu_live_pose:OpenTimeout', ...
            '10 秒内未能打开 RTT Channel 0 输出：%s', openMessage);
    end

    [figureHandle, scene] = createPoseScene( ...
        settings.upperLinkLength, settings.forearmLinkLength);
    setappdata(figureHandle, 'StopRequested', false);
    set(figureHandle, 'CloseRequestFcn', ...
        @(source, ~) setappdata(source, 'StopRequested', true));

    receivedPose = false;
    waitForPoseClock = tic;
    lineBuffer = '';

    while isgraphics(figureHandle) && ...
            ~getappdata(figureHandle, 'StopRequested')
        if loggerProcess.HasExited
            error('arm_imu_live_pose:LoggerStopped', ...
                'JLinkRTTLogger 已停止，请检查 J-Link 连接和板子供电。');
        end

        chunk = char(logReader.ReadToEnd());
        if ~isempty(chunk)
            lineBuffer = [lineBuffer, chunk]; %#ok<AGROW>
        end

        [latestSample, lineBuffer] = parseLatestPose(lineBuffer, settings.poseScale);
        if ~isempty(latestSample)
            receivedPose = true;
            updatePoseScene(scene, latestSample, ...
                settings.upperLinkLength, settings.forearmLinkLength);
            drawnow limitrate;
        else
            drawnow;
            pause(0.01);
        end

        if ~receivedPose && toc(waitForPoseClock) > settings.noPoseTimeoutSeconds
            error('arm_imu_live_pose:NoPose', ...
                ['RTT Channel 0 已连接，但未收到 ARMPOSE。' ...
                 '请确认板子运行 arm_imu，且 ARM_IMU_DEBUG_ENABLE=1。']);
        end
    end

    cleanupResources(loggerProcess, logReader, logStream, figureHandle, logPath);
catch exception
    cleanupResources(loggerProcess, logReader, logStream, figureHandle, logPath);
    rethrow(exception);
end
end

function [latestSample, remainingText] = parseLatestPose(text, scale)
latestSample = [];
lineEnds = find(text == newline);
lineStart = 1;
for index = 1:numel(lineEnds)
    line = text(lineStart:lineEnds(index) - 1);
    lineStart = lineEnds(index) + 1;
    sample = parsePoseLine(line, scale);
    if ~isempty(sample)
        latestSample = sample;
    end
end

if isempty(lineEnds)
    remainingText = text;
else
    remainingText = text(lineStart:end);
end
if numel(remainingText) > 4096
    remainingText = remainingText(end - 4095:end);
end
end

function sample = parsePoseLine(line, scale)
sample = [];
marker = 'ARMPOSE,';
markerIndex = strfind(line, marker);
if isempty(markerIndex)
    return;
end

payload = line(markerIndex(end) + length(marker):end);
parts = regexp(strtrim(payload), ',', 'split');
if numel(parts) < 13
    return;
end
values = str2double(parts(1:13));
if any(~isfinite(values))
    return;
end

sample.sequence = values(1);
sample.upperValid = logical(values(2));
sample.forearmValid = logical(values(3));
sample.calibrated = logical(values(4));
sample.zeroCount = values(5);
sample.upperQuaternion = normalizeQuaternion(values(6:9) ./ scale);
sample.forearmQuaternion = normalizeQuaternion(values(10:13) ./ scale);
end

function quaternion = normalizeQuaternion(quaternion)
normValue = norm(quaternion);
if ~isfinite(normValue) || normValue < 1e-6
    quaternion = [1, 0, 0, 0];
else
    quaternion = quaternion ./ normValue;
end
end

function rotation = quaternionToRotation(quaternion)
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);
rotation = [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y); ...
    2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x); ...
    2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)
];
end

function [figureHandle, scene] = createPoseScene(upperLinkLength, forearmLinkLength)
figureHandle = figure( ...
    'Name', '双 HL04 原坐标姿态 - RTT Channel 0', ...
    'NumberTitle', 'off', ...
    'Color', [0.96, 0.97, 0.98], ...
    'Position', [120, 80, 1120, 780]);
axesHandle = axes('Parent', figureHandle, ...
    'Position', [0.07, 0.08, 0.88, 0.86], ...
    'Color', [0.99, 0.99, 0.99]);
hold(axesHandle, 'on');
grid(axesHandle, 'on');
axis(axesHandle, 'equal');
axis(axesHandle, 'vis3d');
xlabel(axesHandle, 'X / m');
ylabel(axesHandle, 'Y / m');
zlabel(axesHandle, 'Z / m');
titleHandle = title(axesHandle, '等待 ARMPOSE ...');

limit = upperLinkLength + forearmLinkLength + 0.08;
xlim(axesHandle, [-limit, limit]);
ylim(axesHandle, [-limit, limit]);
zlim(axesHandle, [-limit, limit]);

worldScale = 0.085;
drawStaticFrame(axesHandle, [0, 0, 0], eye(3), worldScale, ':', 1.8);
text(axesHandle, worldScale, 0, 0, ' X', 'Color', [0.82, 0.12, 0.12]);
text(axesHandle, 0, worldScale, 0, ' Y', 'Color', [0.08, 0.52, 0.20]);
text(axesHandle, 0, 0, worldScale, ' Z', 'Color', [0.08, 0.28, 0.78]);

[unitX, unitY, unitZ] = sphere(24);
sphereRadius = 0.014;
scene.unitSphere = {unitX * sphereRadius, unitY * sphereRadius, unitZ * sphereRadius};
scene.baseSphere = surf(axesHandle, scene.unitSphere{1}, scene.unitSphere{2}, ...
    scene.unitSphere{3}, 'FaceColor', [0.36, 0.39, 0.43], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.95);
scene.elbowSphere = surf(axesHandle, scene.unitSphere{1}, scene.unitSphere{2}, ...
    scene.unitSphere{3}, 'FaceColor', [0.10, 0.55, 0.72], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.92, 'Visible', 'off');
scene.endSphere = surf(axesHandle, scene.unitSphere{1}, scene.unitSphere{2}, ...
    scene.unitSphere{3}, 'FaceColor', [0.93, 0.62, 0.12], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.92, 'Visible', 'off');
scene.upperLink = plot3(axesHandle, [0, 0], [0, 0], [0, 0], ...
    'Color', [0.20, 0.22, 0.25], 'LineWidth', 7, 'Visible', 'off');
scene.forearmLink = plot3(axesHandle, [0, 0], [0, 0], [0, 0], ...
    'Color', [0.20, 0.22, 0.25], 'LineWidth', 7, 'Visible', 'off');

scene.frameScale = 0.055;
scene.upperFrame = createDynamicFrame(axesHandle);
scene.forearmFrame = createDynamicFrame(axesHandle);
scene.baseLabel = text(axesHandle, 0, 0, 0.025, '基座 / 肩点', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold');
scene.elbowLabel = text(axesHandle, 0, 0, 0.025, '肘点 / 大臂 HL04', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Visible', 'off');
scene.endLabel = text(axesHandle, 0, 0, 0.025, '末端 / 前臂 HL04', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Visible', 'off');
scene.title = titleHandle;

lighting(axesHandle, 'gouraud');
light(axesHandle, 'Style', 'infinite', 'Position', [1, 1, 1]);
light(axesHandle, 'Style', 'infinite', 'Position', [-1, -0.4, 0.5], ...
    'Color', [0.45, 0.45, 0.45]);

setappdata(axesHandle, 'ArmWorkspaceRadius', limit);
drawnow;
enable_arm_camera_controls(figureHandle, axesHandle, 5);
end

function updatePoseScene(scene, sample, upperLinkLength, forearmLinkLength)
basePosition = [0, 0, 0];
upperRotation = quaternionToRotation(sample.upperQuaternion);
forearmRotation = quaternionToRotation(sample.forearmQuaternion);
elbowPosition = (upperRotation * [upperLinkLength; 0; 0])';
endPosition = elbowPosition + ...
    (forearmRotation * [forearmLinkLength; 0; 0])';

setObjectVisible(scene.baseSphere, true);
setObjectVisible(scene.baseLabel, true);

if sample.upperValid
    set(scene.upperLink, 'XData', [basePosition(1), elbowPosition(1)], ...
        'YData', [basePosition(2), elbowPosition(2)], ...
        'ZData', [basePosition(3), elbowPosition(3)], 'Visible', 'on');
    moveSphere(scene.elbowSphere, scene.unitSphere, elbowPosition);
    set(scene.elbowSphere, 'Visible', 'on');
    set(scene.elbowLabel, 'Position', elbowPosition + [0, 0, 0.025], ...
        'Visible', 'on');
else
    set(scene.upperLink, 'Visible', 'off');
    set(scene.elbowSphere, 'Visible', 'off');
    set(scene.elbowLabel, 'Visible', 'off');
end
updateFrame(scene.upperFrame, elbowPosition, upperRotation, ...
    scene.frameScale, sample.upperValid);

bothValid = sample.upperValid && sample.forearmValid;
if bothValid
    set(scene.forearmLink, 'XData', [elbowPosition(1), endPosition(1)], ...
        'YData', [elbowPosition(2), endPosition(2)], ...
        'ZData', [elbowPosition(3), endPosition(3)], 'Visible', 'on');
    moveSphere(scene.endSphere, scene.unitSphere, endPosition);
    set(scene.endSphere, 'Visible', 'on');
    updateFrame(scene.forearmFrame, endPosition, forearmRotation, ...
        scene.frameScale, true);
    set(scene.endLabel, 'Position', endPosition + [0, 0, 0.025], ...
        'Visible', 'on');
else
    set(scene.forearmLink, 'Visible', 'off');
    set(scene.endSphere, 'Visible', 'off');
    updateFrame(scene.forearmFrame, endPosition, eye(3), ...
        scene.frameScale, false);
    set(scene.endLabel, 'Visible', 'off');
end

calibrationText = '未标零';
if sample.calibrated
    calibrationText = sprintf('已标零 #%d', round(sample.zeroCount));
end
scene.title.String = sprintf( ...
    '双 HL04 原坐标姿态 | SEQ %d | %s', ...
    round(sample.sequence), calibrationText);
end

function frameLines = createDynamicFrame(parentAxes)
colors = [0.88, 0.12, 0.12; 0.08, 0.62, 0.22; 0.08, 0.30, 0.88];
frameLines = gobjects(3, 1);
for axisIndex = 1:3
    frameLines(axisIndex) = plot3(parentAxes, [0, 0], [0, 0], [0, 0], ...
        'Color', colors(axisIndex, :), 'LineWidth', 3);
end
end

function drawStaticFrame(parentAxes, origin, rotation, scale, lineStyle, width)
colors = [0.88, 0.12, 0.12; 0.08, 0.62, 0.22; 0.08, 0.30, 0.88];
for axisIndex = 1:3
    endpoint = origin(:) + scale * rotation(:, axisIndex);
    plot3(parentAxes, [origin(1), endpoint(1)], ...
        [origin(2), endpoint(2)], [origin(3), endpoint(3)], ...
        'Color', colors(axisIndex, :), 'LineStyle', lineStyle, ...
        'LineWidth', width);
end
end

function updateFrame(frameLines, origin, rotation, scale, visible)
visibility = onOff(visible);
for axisIndex = 1:3
    endpoint = origin(:) + scale * rotation(:, axisIndex);
    set(frameLines(axisIndex), ...
        'XData', [origin(1), endpoint(1)], ...
        'YData', [origin(2), endpoint(2)], ...
        'ZData', [origin(3), endpoint(3)], ...
        'Visible', visibility);
end
end

function moveSphere(surfaceHandle, unitSphere, position)
set(surfaceHandle, ...
    'XData', unitSphere{1} + position(1), ...
    'YData', unitSphere{2} + position(2), ...
    'ZData', unitSphere{3} + position(3));
end

function setObjectVisible(objectHandle, visible)
set(objectHandle, 'Visible', onOff(visible));
end

function value = onOff(condition)
if condition
    value = 'on';
else
    value = 'off';
end
end

function cleanupResources(loggerProcess, logReader, logStream, figureHandle, logPath)
if ~isempty(logReader)
    try
        logReader.Dispose();
    catch
    end
elseif ~isempty(logStream)
    try
        logStream.Dispose();
    catch
    end
end
if ~isempty(loggerProcess)
    try
        if ~loggerProcess.HasExited
            loggerProcess.Kill();
            loggerProcess.WaitForExit(2000);
        end
        loggerProcess.Dispose();
    catch
    end
end
if isgraphics(figureHandle)
    delete(figureHandle);
end
if isfile(logPath)
    delete(logPath);
end
end

function loggerExe = findRttLogger
candidates = {
    getenv('JLINK_RTT_LOGGER'), ...
    'C:\Code for environment\JLink_V798d\JLinkRTTLogger.exe'
};

for index = 1:numel(candidates)
    if ~isempty(candidates{index}) && isfile(candidates{index})
        loggerExe = candidates{index};
        return;
    end
end

[status, output] = system('where JLinkRTTLogger.exe');
if status == 0
    paths = splitlines(strtrim(output));
    if ~isempty(paths) && isfile(paths{1})
        loggerExe = char(paths{1});
        return;
    end
end

error('arm_imu_live_pose:LoggerNotFound', ...
    ['找不到 JLinkRTTLogger.exe。请安装 SEGGER J-Link Software Pack，' ...
     '或设置环境变量 JLINK_RTT_LOGGER。']);
end

function assertProbeIsFree
processNames = {'JLinkGUIServer.exe', 'JLinkGDBServer.exe', ...
    'JLinkGDBServerCL.exe', 'JLinkRTTLogger.exe', 'openocd.exe'};
for index = 1:numel(processNames)
    command = sprintf('tasklist /NH /FI "IMAGENAME eq %s"', processNames{index});
    [~, output] = system(command);
    if contains(lower(output), lower(processNames{index}))
        error('arm_imu_live_pose:ProbeBusy', ...
            '%s 正在或可能正在占用 J-Link，请先关闭它。', ...
            processNames{index});
    end
end
end
