function enable_arm_camera_controls(fig, ax, defaultView)
%ENABLE_ARM_CAMERA_CONTROLS 为机械臂图窗启用编号视图和相机操作。
%
% 操作方式：
%   V 键               - 按编号循环 10 个预设视图。
%   数字键 1~9、0     - 直接选择视图 01~09、10。
%   F 键               - 保持当前方向，重新完整取景。
%   R 键               - 强制使用最近一帧关节角刷新模型。
%   H 键或 Home 键     - 一键回到固定主视图 05。
%   按住鼠标左键拖动 - 自由旋转视角。
%   按住鼠标中键拖动 - 仅绕基座竖直轴水平旋转。
%   滚动鼠标滚轮     - 放大或缩小。
%
% 仅当鼠标位于三维坐标轴区域内时响应，不影响右侧串口状态面板。

if nargin < 3 || isempty(defaultView)
    defaultView = 5;
end
validateattributes(defaultView, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', 10}, ...
    mfilename, 'defaultView');

viewAzimuth = [0, 180, 90, -90, 90, 0, 45, 135, -135, -45];
viewElevation = [0, 0, 0, 0, 0, -90, 30, 30, 30, 30];
viewNames = { ...
    '正视', '后视', '左视', '右视', '主视图 Z上 X前', '仰视', ...
    '等轴 前右', '等轴 前左', '等轴 后左', '等轴 后右'};

activeDragMode = '';
lastMousePoint = [0, 0];
currentViewIndex = double(defaultView);
% 相机位于 -X 后方略偏 -Y/+Z；+Z 向上，+X 从观察者朝前延伸。
mainCameraDirection = [-1, -0.30, 0.25];
mainCameraDirection = mainCameraDirection / norm(mainCameraDirection);
mainCameraUp = [0, 0, 1];
mainHorizontalDirection = mainCameraDirection;
mainHorizontalDirection(3) = 0;
mainHorizontalDirection = mainHorizontalDirection / norm(mainHorizontalDirection);

% 光源只有正反两个固定方位。相机跨过两个水平半球的边界时才切换一次，
% 不使用随相机逐帧移动的 camlight，避免拖动时产生额外的连续光照更新。
fixedLights = gobjects(1, 0);
currentLightSide = 0;

% 标签属于 figure，不会被 rigidBodyTree.show(...,'PreservePlot',false) 删除。
axesPosition = get(ax, 'Position');
viewLabel = annotation(fig, 'textbox', ...
    [axesPosition(1) + 0.012, axesPosition(2) + axesPosition(4) - 0.055, 0.18, 0.04], ...
    'String', '', ...
    'Interpreter', 'none', ...
    'FontName', 'Microsoft YaHei UI', ...
    'FontSize', 10, ...
    'FontWeight', 'bold', ...
    'Color', [0.10, 0.10, 0.10], ...
    'BackgroundColor', [1.00, 1.00, 1.00], ...
    'EdgeColor', [0.45, 0.45, 0.45], ...
    'Margin', 4, ...
    'FitBoxToText', 'on');
if isprop(viewLabel, 'HitTest')
    viewLabel.HitTest = 'off';
end
if isprop(viewLabel, 'PickableParts')
    viewLabel.PickableParts = 'none';
end

set(fig, ...
    'WindowButtonDownFcn', @onMouseDown, ...
    'WindowButtonMotionFcn', @onMouseMove, ...
    'WindowButtonUpFcn', @onMouseUp, ...
    'WindowScrollWheelFcn', @onMouseWheel, ...
    'WindowKeyPressFcn', @onKeyPress);
setappdata(fig, 'ArmCameraInteracting', false);
setappdata(fig, 'ArmRefreshLighting', @refreshFixedLights);
initializeFixedLights();
applyNumberedView(currentViewIndex);

    function onKeyPress(~, event)
        if ~isgraphics(ax)
            return;
        end

        requestedView = [];
        if strcmpi(event.Key, 'v')
            requestedView = mod(currentViewIndex, 10) + 1;
        elseif strcmpi(event.Key, 'f')
            fitCameraToWorkspace();
            return;
        elseif strcmpi(event.Key, 'r')
            if isappdata(fig, 'ArmForceRefresh')
                refreshFunction = getappdata(fig, 'ArmForceRefresh');
                if isa(refreshFunction, 'function_handle')
                    refreshFunction();
                end
            end
            return;
        elseif strcmpi(event.Key, 'h') || strcmpi(event.Key, 'home')
            applyNumberedView(5);
            return;
        elseif ~isempty(event.Character)
            keyCharacter = char(event.Character);
            if isscalar(keyCharacter) && keyCharacter >= '1' && keyCharacter <= '9'
                requestedView = double(keyCharacter - '0');
            elseif isequal(keyCharacter, '0')
                requestedView = 10;
            end
        end

        if ~isempty(requestedView)
            applyNumberedView(requestedView);
        end
    end

    function applyNumberedView(requestedView)
        previousViewIndex = currentViewIndex;
        try
            setappdata(fig, 'ArmCameraInteracting', true);
            limits = [ax.XLim; ax.YLim; ax.ZLim];
            target = mean(limits, 2)';
            currentViewIndex = requestedView;
            if currentViewIndex == 5
                % 使用显式方向向量，不依赖不同 MATLAB 版本的 azimuth 解释。
                workspaceRadius = double(getappdata(ax, 'ArmWorkspaceRadius'));
                camtarget(ax, target);
                campos(ax, target + 4 * workspaceRadius * mainCameraDirection);
                camup(ax, mainCameraUp);
            else
                view(ax, viewAzimuth(currentViewIndex), viewElevation(currentViewIndex));
                if all(isfinite(target))
                    camtarget(ax, target);
                end
            end
            fitCameraToWorkspace();
            updateLightSideIfNeeded();
            if isgraphics(viewLabel)
                viewLabel.String = sprintf('视图 %02d | %s', ...
                    currentViewIndex, viewNames{currentViewIndex});
            end
            drawnow limitrate;
        catch cameraError
            currentViewIndex = previousViewIndex;
            if isgraphics(fig)
                warning('enable_arm_camera_controls:ViewChangeFailed', ...
                    '编号视图切换失败，已保留当前视角：%s', cameraError.message);
            end
        end
        if isgraphics(fig)
            setappdata(fig, 'ArmCameraInteracting', false);
        end
    end

    function fitCameraToWorkspace()
        % axis vis3d 会冻结旧缩放。这里让 MATLAB 针对已经锁定的固定边界
        % 自动求一次视角，再加 12%% 画面留白；串口刷新不会进入本函数。
        target = mean([ax.XLim; ax.YLim; ax.ZLim], 2)';
        ax.Projection = 'orthographic';
        camtarget(ax, target);
        ax.CameraViewAngleMode = 'auto';
        drawnow limitrate;
        fittedViewAngle = ax.CameraViewAngle;
        ax.CameraViewAngleMode = 'manual';
        ax.CameraViewAngle = min(160, 1.12 * fittedViewAngle);
        drawnow limitrate;
    end

    function onMouseDown(~, ~)
        if ~isPointerInsideAxes()
            return;
        end

        % MATLAB：左键为 normal，中键为 extend。
        if strcmp(fig.SelectionType, 'normal')
            activeDragMode = 'free';
        elseif strcmp(fig.SelectionType, 'extend')
            activeDragMode = 'horizontal';
        else
            return;
        end

        setappdata(fig, 'ArmCameraInteracting', true);
        lastMousePoint = fig.CurrentPoint;
        fig.Pointer = 'fleur';
    end

    function onMouseMove(~, ~)
        if isempty(activeDragMode) || ~isgraphics(ax)
            return;
        end

        currentMousePoint = fig.CurrentPoint;
        mouseDelta = currentMousePoint - lastMousePoint;
        lastMousePoint = currentMousePoint;
        if strcmp(activeDragMode, 'free')
            % 左键维持原来的自由轨道旋转。
            camorbit(ax, -0.30 * mouseDelta(1), ...
                         -0.30 * mouseDelta(2), 'camera');
        else
            % 中键只绕世界坐标 Z 轴旋转。忽略鼠标竖直位移，保持当前俯仰高度。
            rotateCameraAroundBaseAxis(-0.30 * mouseDelta(1));
        end
        updateLightSideIfNeeded();
        drawnow limitrate;
    end

    function onMouseUp(~, ~)
        if ~isempty(activeDragMode)
            activeDragMode = '';
            if isgraphics(fig)
                setappdata(fig, 'ArmCameraInteracting', false);
                fig.Pointer = 'arrow';
            end
        end
    end

    function onMouseWheel(~, event)
        if ~isgraphics(ax) || ~isPointerInsideAxes()
            return;
        end

        zoomFactor = 1.12 ^ (-event.VerticalScrollCount);
        camzoom(ax, zoomFactor);
        drawnow limitrate;
    end

    function rotateCameraAroundBaseAxis(angleDegrees)
        target = ax.CameraTarget;
        offset = ax.CameraPosition - target;
        angleRadians = deg2rad(angleDegrees);
        cosineValue = cos(angleRadians);
        sineValue = sin(angleRadians);

        % 绕 Z 轴的显式旋转不依赖 camorbit 对坐标系参数的版本差异。
        rotatedOffset = [ ...
            cosineValue * offset(1) - sineValue * offset(2), ...
            sineValue * offset(1) + cosineValue * offset(2), ...
            offset(3)];
        campos(ax, target + rotatedOffset);
        camup(ax, mainCameraUp);
    end

    function initializeFixedLights()
        if ~isgraphics(ax)
            return;
        end

        % 移除 rigidBodyTree.show 自动建立的光源，改由本控制器统一管理。
        oldLights = findall(ax, 'Type', 'light');
        if ~isempty(oldLights)
            delete(oldLights);
        end

        positiveDirection = mainHorizontalDirection + 0.55 * mainCameraUp;
        negativeDirection = -mainHorizontalDirection + 0.55 * mainCameraUp;
        positiveDirection = positiveDirection / norm(positiveDirection);
        negativeDirection = negativeDirection / norm(negativeDirection);

        fixedLights = gobjects(1, 2);
        fixedLights(1) = light(ax, ...
            'Style', 'infinite', ...
            'Position', positiveDirection, ...
            'Color', [1.00, 1.00, 1.00], ...
            'Visible', 'off', ...
            'Tag', 'ArmFixedPositiveLight');
        fixedLights(2) = light(ax, ...
            'Style', 'infinite', ...
            'Position', negativeDirection, ...
            'Color', [1.00, 1.00, 1.00], ...
            'Visible', 'off', ...
            'Tag', 'ArmFixedNegativeLight');
        if isprop(ax, 'AmbientLightColor')
            ax.AmbientLightColor = [0.28, 0.28, 0.28];
        end
        currentLightSide = 0;
        updateLightSideIfNeeded();
    end

    function refreshFixedLights()
        % FastUpdate 正常情况下会保留光源；若某次 show 重建了图元，才重建光源。
        if numel(fixedLights) ~= 2 || ~all(isgraphics(fixedLights, 'light'))
            initializeFixedLights();
            return;
        end

        % 防止 show 额外创建自动光源，保留且只保留两个固定状态光源。
        allLights = findall(ax, 'Type', 'light');
        for lightIndex = 1:numel(allLights)
            if ~any(allLights(lightIndex) == fixedLights)
                delete(allLights(lightIndex));
            end
        end
        updateLightSideIfNeeded();
    end

    function updateLightSideIfNeeded()
        if numel(fixedLights) ~= 2 || ~all(isgraphics(fixedLights, 'light'))
            return;
        end

        cameraDirection = ax.CameraPosition - ax.CameraTarget;
        cameraDirection(3) = 0;
        horizontalNorm = norm(cameraDirection);
        if ~isfinite(horizontalNorm) || horizontalNorm < eps
            return;
        end
        cameraDirection = cameraDirection / horizontalNorm;
        sideScore = dot(cameraDirection, mainHorizontalDirection);

        % 0.05 的滞回区避免相机恰在分界面附近时来回切换。
        requestedSide = currentLightSide;
        if currentLightSide == 0
            requestedSide = 1;
            if sideScore < 0
                requestedSide = -1;
            end
        elseif currentLightSide > 0 && sideScore < -0.05
            requestedSide = -1;
        elseif currentLightSide < 0 && sideScore > 0.05
            requestedSide = 1;
        end

        if requestedSide == currentLightSide
            return;
        end
        currentLightSide = requestedSide;
        if currentLightSide > 0
            set(fixedLights(1), 'Visible', 'on');
            set(fixedLights(2), 'Visible', 'off');
        else
            set(fixedLights(1), 'Visible', 'off');
            set(fixedLights(2), 'Visible', 'on');
        end
    end

    function inside = isPointerInsideAxes()
        if ~isgraphics(fig) || ~isgraphics(ax)
            inside = false;
            return;
        end

        pointerPosition = fig.CurrentPoint;
        axesPixelPosition = getpixelposition(ax, true);
        inside = pointerPosition(1) >= axesPixelPosition(1) && ...
                 pointerPosition(1) <= axesPixelPosition(1) + axesPixelPosition(3) && ...
                 pointerPosition(2) >= axesPixelPosition(2) && ...
                 pointerPosition(2) <= axesPixelPosition(2) + axesPixelPosition(4);
    end
end
