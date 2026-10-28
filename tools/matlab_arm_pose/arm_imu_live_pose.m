function arm_imu_live_pose
%ARM_IMU_LIVE_POSE Display arm IMU orientations received through J-Link RTT.

if ~ispc
    error('This launcher currently supports Windows only.');
end

settings.device = 'STM32F407IG';
settings.interface = 'SWD';
settings.speedKHz = 4000;
settings.historySeconds = 20;
settings.poseScale = 1000000;
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
loggerProcess = System.Diagnostics.Process.Start(startInfo);

try
waitStart = tic;
while toc(waitStart) < 10
    if loggerProcess.HasExited
        error(['JLinkRTTLogger exited before connecting. Close Ozone, Cortex-Debug, ', ...
               'OpenOCD, and any other program using the J-Link.']);
    end
    if isfile(logPath)
        try
            logStream = System.IO.FileStream(logPath, System.IO.FileMode.Open, ...
                System.IO.FileAccess.Read, System.IO.FileShare.ReadWrite);
            logReader = System.IO.StreamReader(logStream);
            break;
        catch openException
            openMessage = openException.message;
        end
    else
        openMessage = 'RTT output file has not been created.';
    end
    pause(0.1);
end
if isempty(logReader)
    error('Could not open the RTT stream: %s', openMessage);
end

[figureHandle, plotState] = createPlotWindow();
setappdata(figureHandle, 'StopRequested', false);
set(figureHandle, 'CloseRequestFcn', @(source, ~) setappdata(source, 'StopRequested', true));

startTime = tic;
lastPoseTime = tic;
receivedPose = false;
previousUpperQuaternion = [];
previousForearmQuaternion = [];
upperTimeHistory = [];
upperEulerHistory = zeros(0, 3);
forearmTimeHistory = [];
forearmEulerHistory = zeros(0, 3);
lineBuffer = '';

while isgraphics(figureHandle) && ~getappdata(figureHandle, 'StopRequested')
    if loggerProcess.HasExited
        error('JLinkRTTLogger stopped. Check the J-Link connection and target power.');
    end

    parsedThisPass = false;
    chunk = char(logReader.ReadToEnd());
    if ~isempty(chunk)
        lineBuffer = [lineBuffer, chunk]; %#ok<AGROW>
    end
    lineEnds = find(lineBuffer == newline);
    lineStart = 1;
    for lineIndex = 1:numel(lineEnds)
        line = lineBuffer(lineStart:lineEnds(lineIndex) - 1);
        lineStart = lineEnds(lineIndex) + 1;

        sample = parsePoseLine(line, settings.poseScale);
        if isempty(sample)
            continue;
        end

        receivedPose = true;
        parsedThisPass = true;
        lastPoseTime = tic;
        sampleTime = toc(startTime);

        if sample.upperValid
            sample.upperQuaternion = makeQuaternionContinuous(sample.upperQuaternion, previousUpperQuaternion);
            previousUpperQuaternion = sample.upperQuaternion;
            upperRotation = quaternionToRotation(sample.upperQuaternion);
            upperEuler = rotationToEulerZYX(upperRotation);
            updateFrame(plotState.upperFrame, [0, 0, 0], upperRotation, true);

            upperTimeHistory(end + 1, 1) = sampleTime; %#ok<AGROW>
            upperEulerHistory(end + 1, :) = upperEuler; %#ok<AGROW>
            keep = upperTimeHistory >= sampleTime - settings.historySeconds;
            upperTimeHistory = upperTimeHistory(keep);
            upperEulerHistory = upperEulerHistory(keep, :);
            set(plotState.upperRollLine, 'XData', upperTimeHistory, 'YData', upperEulerHistory(:, 1));
            set(plotState.upperPitchLine, 'XData', upperTimeHistory, 'YData', upperEulerHistory(:, 2));
            set(plotState.upperYawLine, 'XData', upperTimeHistory, 'YData', upperEulerHistory(:, 3));
            xlim(plotState.upperEulerAxes, [max(0, sampleTime - settings.historySeconds), max(settings.historySeconds, sampleTime)]);
        else
            upperEuler = [NaN, NaN, NaN];
            updateFrame(plotState.upperFrame, [0, 0, 0], eye(3), false);
        end

        if sample.forearmValid
            sample.forearmQuaternion = makeQuaternionContinuous(sample.forearmQuaternion, previousForearmQuaternion);
            previousForearmQuaternion = sample.forearmQuaternion;
            forearmRotation = quaternionToRotation(sample.forearmQuaternion);
            forearmEuler = rotationToEulerZYX(forearmRotation);
            updateFrame(plotState.forearmFrame, [1.8, 0, 0], forearmRotation, true);

            forearmTimeHistory(end + 1, 1) = sampleTime; %#ok<AGROW>
            forearmEulerHistory(end + 1, :) = forearmEuler; %#ok<AGROW>
            keep = forearmTimeHistory >= sampleTime - settings.historySeconds;
            forearmTimeHistory = forearmTimeHistory(keep);
            forearmEulerHistory = forearmEulerHistory(keep, :);
            set(plotState.forearmRollLine, 'XData', forearmTimeHistory, 'YData', forearmEulerHistory(:, 1));
            set(plotState.forearmPitchLine, 'XData', forearmTimeHistory, 'YData', forearmEulerHistory(:, 2));
            set(plotState.forearmYawLine, 'XData', forearmTimeHistory, 'YData', forearmEulerHistory(:, 3));
            xlim(plotState.forearmEulerAxes, [max(0, sampleTime - settings.historySeconds), max(settings.historySeconds, sampleTime)]);
        else
            forearmEuler = [NaN, NaN, NaN];
            updateFrame(plotState.forearmFrame, [1.8, 0, 0], eye(3), false);
        end

        set(plotState.statusText, 'String', sprintf([ ...
            'Sequence: %.0f\nUpper valid: %d\nForearm valid: %d\nCalibrated: %d\nZero count: %.0f\n\n', ...
            'Upper quaternion [w x y z]\n[% .6f  % .6f  % .6f  % .6f]\n\n', ...
            'Upper ZYX Euler [deg]\nRoll  % 8.3f\nPitch % 8.3f\nYaw   % 8.3f\n\n', ...
            'Forearm quaternion [w x y z]\n[% .6f  % .6f  % .6f  % .6f]\n\n', ...
            'Forearm ZYX Euler [deg]\nRoll  % 8.3f\nPitch % 8.3f\nYaw   % 8.3f'], ...
            sample.sequence, sample.upperValid, sample.forearmValid, sample.calibrated, sample.zeroCount, ...
            sample.upperQuaternion(1), sample.upperQuaternion(2), sample.upperQuaternion(3), sample.upperQuaternion(4), ...
            upperEuler(1), upperEuler(2), upperEuler(3), ...
            sample.forearmQuaternion(1), sample.forearmQuaternion(2), sample.forearmQuaternion(3), sample.forearmQuaternion(4), ...
            forearmEuler(1), forearmEuler(2), forearmEuler(3)));
    end
    if ~isempty(lineEnds)
        lineBuffer = lineBuffer(lineStart:end);
    end

    if ~receivedPose && toc(lastPoseTime) > settings.noPoseTimeoutSeconds
        error(['RTT is connected, but no ARMPOSE records were received. ', ...
               'Build and flash the arm_imu firmware containing the MATLAB telemetry output.']);
    end
    if parsedThisPass
        drawnow limitrate;
    else
        drawnow;
        pause(0.01);
    end
end

if isgraphics(figureHandle)
    delete(figureHandle);
end
catch exception
    cleanupResources(loggerProcess, logReader, logStream, figureHandle, logPath);
    rethrow(exception);
end
cleanupResources(loggerProcess, logReader, logStream, figureHandle, logPath);
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
        loggerExe = paths{1};
        return;
    end
end

error(['JLinkRTTLogger.exe was not found. Install the SEGGER J-Link Software Pack ', ...
       'or set the JLINK_RTT_LOGGER environment variable.']);
end

function assertProbeIsFree
processNames = {'JLinkGUIServer.exe', 'JLinkGDBServer.exe', 'JLinkGDBServerCL.exe', 'JLinkRTTLogger.exe', 'openocd.exe'};
for index = 1:numel(processNames)
    command = sprintf('tasklist /NH /FI "IMAGENAME eq %s"', processNames{index});
    [~, output] = system(command);
    if contains(lower(output), lower(processNames{index}))
        error('%s is already using or may be using the debug probe. Close it first.', processNames{index});
    end
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
if numel(parts) < 11
    return;
end
fieldCount = 11;
if numel(parts) >= 13
    fieldCount = 13;
end
values = str2double(parts(1:fieldCount));
if any(~isfinite(values))
    return;
end

sample.sequence = values(1);
sample.upperValid = logical(values(2));
sample.forearmValid = logical(values(3));
if fieldCount == 13
    sample.calibrated = logical(values(4));
    sample.zeroCount = values(5);
    sample.upperQuaternion = values(6:9) ./ scale;
    sample.forearmQuaternion = values(10:13) ./ scale;
else
    sample.calibrated = false;
    sample.zeroCount = 0;
    sample.upperQuaternion = values(4:7) ./ scale;
    sample.forearmQuaternion = values(8:11) ./ scale;
end
end

function quaternion = makeQuaternionContinuous(quaternion, previousQuaternion)
normValue = norm(quaternion);
if normValue < 1e-6
    quaternion = [1, 0, 0, 0];
    return;
end
quaternion = quaternion ./ normValue;
if ~isempty(previousQuaternion) && dot(quaternion, previousQuaternion) < 0
    quaternion = -quaternion;
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

function eulerDegrees = rotationToEulerZYX(rotation)
pitch = asin(max(-1, min(1, -rotation(3, 1))));
roll = atan2(rotation(3, 2), rotation(3, 3));
yaw = atan2(rotation(2, 1), rotation(1, 1));
eulerDegrees = [roll, pitch, yaw] .* (180 / pi);
end

function [figureHandle, plotState] = createPlotWindow
figureHandle = figure('Name', 'HL04 Live Pose', 'NumberTitle', 'off', 'Color', 'w', ...
    'Position', [60, 80, 1500, 760]);
layout = tiledlayout(figureHandle, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

poseAxes = nexttile(layout, 1, [2, 1]);
hold(poseAxes, 'on');
grid(poseAxes, 'on');
axis(poseAxes, 'equal');
view(poseAxes, 35, 24);
xlim(poseAxes, [-1.2, 3.0]);
ylim(poseAxes, [-1.3, 1.3]);
zlim(poseAxes, [-1.3, 1.3]);
xlabel(poseAxes, 'World X');
ylabel(poseAxes, 'World Y');
zlabel(poseAxes, 'World Z');
title(poseAxes, 'Upper and Forearm Sensor Frames');
plot3(poseAxes, [-1, 1], [0, 0], [0, 0], ':', 'Color', [0.65, 0.65, 0.65]);
plot3(poseAxes, [0, 0], [-1, 1], [0, 0], ':', 'Color', [0.65, 0.65, 0.65]);
plot3(poseAxes, [0, 0], [0, 0], [-1, 1], ':', 'Color', [0.65, 0.65, 0.65]);
text(poseAxes, 0, 0, -1.15, 'Upper', 'HorizontalAlignment', 'center');
text(poseAxes, 1.8, 0, -1.15, 'Forearm', 'HorizontalAlignment', 'center');
plotState.upperFrame = createFrame(poseAxes);
plotState.forearmFrame = createFrame(poseAxes);

[plotState.upperEulerAxes, plotState.upperRollLine, plotState.upperPitchLine, plotState.upperYawLine] = ...
    createEulerPlot(layout, 2, 'Upper IMU ZYX Euler Angles');
[plotState.forearmEulerAxes, plotState.forearmRollLine, plotState.forearmPitchLine, plotState.forearmYawLine] = ...
    createEulerPlot(layout, 5, 'Forearm IMU ZYX Euler Angles');

statusAxes = nexttile(layout, 3, [2, 1]);
axis(statusAxes, 'off');
plotState.statusText = text(statusAxes, 0.02, 0.96, 'Waiting for ARMPOSE data...', ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 10);
end

function [axesHandle, rollLine, pitchLine, yawLine] = createEulerPlot(layout, tileIndex, titleText)
axesHandle = nexttile(layout, tileIndex);
hold(axesHandle, 'on');
grid(axesHandle, 'on');
rollLine = plot(axesHandle, NaN, NaN, 'Color', [0.82, 0.18, 0.18], 'LineWidth', 1.3);
pitchLine = plot(axesHandle, NaN, NaN, 'Color', [0.12, 0.55, 0.24], 'LineWidth', 1.3);
yawLine = plot(axesHandle, NaN, NaN, 'Color', [0.12, 0.32, 0.78], 'LineWidth', 1.3);
xlabel(axesHandle, 'Time [s]');
ylabel(axesHandle, 'Angle [deg]');
title(axesHandle, titleText);
legend(axesHandle, {'Roll', 'Pitch', 'Yaw'}, 'Location', 'best');
end

function frameLines = createFrame(parentAxes)
colors = [0.86, 0.16, 0.16; 0.12, 0.62, 0.24; 0.12, 0.34, 0.88];
frameLines = gobjects(3, 1);
for axisIndex = 1:3
    frameLines(axisIndex) = plot3(parentAxes, [0, 0], [0, 0], [0, 0], ...
        'Color', colors(axisIndex, :), 'LineWidth', 3);
end
end

function updateFrame(frameLines, origin, rotation, visible)
visibility = 'off';
if visible
    visibility = 'on';
end
for axisIndex = 1:3
    endpoint = origin(:) + rotation(:, axisIndex);
    set(frameLines(axisIndex), 'XData', [origin(1), endpoint(1)], ...
        'YData', [origin(2), endpoint(2)], 'ZData', [origin(3), endpoint(3)], ...
        'Visible', visibility);
end
end
