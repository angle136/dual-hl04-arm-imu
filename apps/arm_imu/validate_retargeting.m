function results = validate_retargeting()
%VALIDATE_RETARGETING Compare straight-rod poses against the real arm URDF.

root = fileparts(mfilename('fullpath'));
urdf = fullfile(root, '..', '..', '..', 'arm_small_machine', ...
    'matlab_arm_visualizer', 'model', 'robot_dynamics.urdf');
robot = importrobot(urdf, 'DataFormat', 'row');

direction = [-1 -1 -1 -1 -1 1 1];
zeroOffset = [0 -1.278566 -2.918680 3.110406 -0.031736 -0.000452 -1.1];
qReference = [0 1.66 0.25 -0.23 -0.16 0.23];
qMin = [-2*pi -1.9 -3.2 -2*pi -2.1 -2*pi];
qMax = [ 2*pi  1.8  3.0  2*pi  2.7  2*pi];

humanToRobot = [0 1 0; 0 0 1; 1 0 0];
upperLength = 0.204;
forearmLength = 0.120;
humanReach = upperLength + forearmLength;

TReference = armTransform(robot, qReference, direction, zeroOffset);
TShoulder = getTransform(robot, protocolToUrdf([qReference 0], direction, zeroOffset), ...
    'link_002', robot.BaseName);
robotShoulder = TShoulder(1:3, 4);
robotReference = TReference(1:3, 4);
robotReachVector = robotReference - robotShoulder;
positionScale = norm(robotReachVector) / humanReach;

mappedNeutral = humanToRobot * [1; 0; 0];
positionAlignment = vectorAlignment(mappedNeutral, robotReachVector / norm(robotReachVector));
positionMap = positionAlignment * humanToRobot;

shoulderYaw = deg2rad([-60 0 60]);
shoulderPitch = deg2rad([-45 0 45]);
shoulderRoll = deg2rad([-45 0 45]);
elbowBend = deg2rad([0 45 90]);
forearmTwist = deg2rad([-60 0 60]);

seeds = [qReference; ...
         0 0.15 1.20 -0.70 0 0; ...
         0 1.20 0.65 -0.55 0 0; ...
        -1.40 0.50 1.55 -0.90 0 0];
seeds = min(max(seeds, qMin), qMax);
options = optimoptions('lsqnonlin', 'Display', 'off', ...
    'MaxIterations', 100, 'MaxFunctionEvaluations', 3000, ...
    'FunctionTolerance', 1e-10, 'StepTolerance', 1e-10);

rows = [];
poseIndex = 0;
previous = qReference;
previousApp = qReference;
for yaw = shoulderYaw
    for pitch = shoulderPitch
        for roll = shoulderRoll
            RUpper = rotz(yaw) * roty(pitch) * rotx(roll);
            for elbow = elbowBend
                for twist = forearmTwist
                    poseIndex = poseIndex + 1;
                    RForearm = RUpper * rotz(elbow) * rotx(twist);
                    humanWrist = RUpper * [upperLength; 0; 0] + ...
                        RForearm * [forearmLength; 0; 0];
                    targetPosition = robotShoulder + positionScale * positionMap * humanWrist;
                    mappedRotation = humanToRobot * RForearm / humanToRobot;
                    targetRotation = mappedRotation * TReference(1:3, 1:3);

                    trialSeeds = [previous; seeds];
                    bestCost = inf;
                    bestQ = qReference;
                    bestPosError = inf;
                    bestRotError = inf;
                    for seedIndex = 1:size(trialSeeds, 1)
                        objective = @(q) poseResidual(robot, q, direction, zeroOffset, ...
                            targetPosition, targetRotation);
                        [candidate, cost] = lsqnonlin(objective, trialSeeds(seedIndex, :), ...
                            qMin, qMax, options);
                        [positionError, rotationError] = poseError(robot, candidate, ...
                            direction, zeroOffset, targetPosition, targetRotation);
                        score = positionError / 0.015 + rotationError / 0.025;
                        if score < bestCost
                            bestCost = score;
                            bestQ = candidate;
                            bestPosError = positionError;
                            bestRotError = rotationError;
                        end
                    end

                    accepted = bestPosError < 0.015 && bestRotError < 0.025;
                    if accepted
                        previous = bestQ;
                    end

                    appSeeds = [previousApp; seeds];
                    [appQ, appPosError, appRotError, appAccepted, appAttempt, appIterations] = ...
                        solveAppDls(robot, appSeeds, direction, zeroOffset, qMin, qMax, ...
                        targetPosition, targetRotation);
                    if appAccepted
                        previousApp = appQ;
                    end
                    [coldQ, coldPosError, coldRotError, coldAccepted, coldAttempt, coldIterations] = ...
                        solveAppDls(robot, seeds, direction, zeroOffset, qMin, qMax, ...
                        targetPosition, targetRotation);
                    rows(end + 1, :) = [poseIndex rad2deg([yaw pitch roll elbow twist]), ...
                        targetPosition.', bestQ, bestPosError, bestRotError, accepted, ...
                        appQ, appPosError, appRotError, appAccepted, appAttempt, appIterations, ...
                        coldQ, coldPosError, coldRotError, coldAccepted, coldAttempt, coldIterations]; %#ok<AGROW>
                end
            end
        end
    end
end

names = {'pose','yaw_deg','pitch_deg','roll_deg','elbow_deg','twist_deg', ...
    'target_x','target_y','target_z','j1','j2','j3','j4','j5','j6', ...
    'position_error_m','rotation_error_rad','accepted', ...
    'app_j1','app_j2','app_j3','app_j4','app_j5','app_j6', ...
    'app_position_error_m','app_rotation_error_rad','app_accepted', ...
    'app_attempt','app_iterations', ...
    'cold_j1','cold_j2','cold_j3','cold_j4','cold_j5','cold_j6', ...
    'cold_position_error_m','cold_rotation_error_rad','cold_accepted', ...
    'cold_attempt','cold_iterations'};
results = array2table(rows, 'VariableNames', names);
output = fullfile(root, 'retargeting_validation.csv');
writetable(results, output);

acceptedCount = nnz(results.accepted);
appAcceptedCount = nnz(results.app_accepted);
coldAcceptedCount = nnz(results.cold_accepted);
[worstPosition, worstPositionIndex] = max(results.position_error_m);
[worstRotation, worstRotationIndex] = max(results.rotation_error_rad);
fprintf('Retarget scale: %.8f (robot %.6f m / human %.6f m)\n', ...
    positionScale, norm(robotReachVector), humanReach);
fprintf('Accepted: %d/%d (%.1f%%)\n', acceptedCount, height(results), ...
    100 * acceptedCount / height(results));
fprintf('APP DLS accepted: %d/%d (%.1f%%)\n', appAcceptedCount, height(results), ...
    100 * appAcceptedCount / height(results));
fprintf('Cold multiseed accepted: %d/%d (%.1f%%)\n', coldAcceptedCount, height(results), ...
    100 * coldAcceptedCount / height(results));
fprintf('Worst position: %.6f m at pose %d\n', worstPosition, worstPositionIndex);
fprintf('Worst rotation: %.6f rad at pose %d\n', worstRotation, worstRotationIndex);
fprintf('CSV: %s\n', output);
end

function [bestQ, bestPositionError, bestRotationError, accepted, bestAttempt, bestIterations] = ...
    solveAppDls(robot, seeds, direction, zeroOffset, qMin, qMax, targetPosition, targetRotation)
bestQ = seeds(1, :);
bestPositionError = inf;
bestRotationError = inf;
bestAttempt = 0;
bestIterations = 0;
accepted = false;
for attempt = 1:size(seeds, 1)
    q = min(max(seeds(attempt, :), qMin), qMax);
    for iteration = 0:20
        T = armTransform(robot, q, direction, zeroOffset);
        positionVector = targetPosition - T(1:3, 4);
        rotationVector = rotationLog(targetRotation * T(1:3, 1:3).');
        positionError = norm(positionVector);
        rotationError = norm(rotationVector);
        score = positionError / 0.015 + rotationError / 0.025;
        bestScore = bestPositionError / 0.015 + bestRotationError / 0.025;
        if score < bestScore
            bestQ = q;
            bestPositionError = positionError;
            bestRotationError = rotationError;
            bestAttempt = attempt;
            bestIterations = iteration;
        end
        if positionError < 0.015 && rotationError < 0.025
            accepted = true;
            return;
        end
        if iteration == 20
            break;
        end

        qUrdf = protocolToUrdf([q 0], direction, zeroOffset);
        geometric = geometricJacobian(robot, qUrdf, 'link_007');
        angular = geometric(1:3, 1:6);
        linear = geometric(4:6, 1:6);
        jacobian = [linear; angular] .* direction(1:6);
        error = [positionVector; rotationVector];
        step = (jacobian.' * jacobian + 0.025^2 * eye(6)) \ ...
            (jacobian.' * error);
        step = max(-0.20, min(0.20, step.'));
        q = min(max(q + step, qMin), qMax);
    end
end
end

function T = armTransform(robot, protocolQ, direction, zeroOffset)
q = protocolToUrdf([protocolQ 0], direction, zeroOffset);
T = getTransform(robot, q, 'link_007', robot.BaseName);
end

function q = protocolToUrdf(protocolQ, direction, zeroOffset)
q = direction .* protocolQ + zeroOffset;
end

function residual = poseResidual(robot, protocolQ, direction, zeroOffset, targetPosition, targetRotation)
[positionError, rotationVector] = poseErrorVector(robot, protocolQ, direction, ...
    zeroOffset, targetPosition, targetRotation);
residual = [positionError; 0.15 * rotationVector];
end

function [positionNorm, rotationNorm] = poseError(robot, protocolQ, direction, zeroOffset, targetPosition, targetRotation)
[positionError, rotationVector] = poseErrorVector(robot, protocolQ, direction, ...
    zeroOffset, targetPosition, targetRotation);
positionNorm = norm(positionError);
rotationNorm = norm(rotationVector);
end

function [positionError, rotationVector] = poseErrorVector(robot, protocolQ, direction, zeroOffset, targetPosition, targetRotation)
T = armTransform(robot, protocolQ, direction, zeroOffset);
positionError = T(1:3, 4) - targetPosition;
delta = targetRotation * T(1:3, 1:3).';
rotationVector = rotationLog(delta);
end

function vector = rotationLog(rotation)
cosine = max(-1, min(1, (trace(rotation) - 1) / 2));
angle = acos(cosine);
if angle < 1e-8
    vector = 0.5 * [rotation(3,2)-rotation(2,3); ...
        rotation(1,3)-rotation(3,1); rotation(2,1)-rotation(1,2)];
else
    vector = angle / (2 * sin(angle)) * [rotation(3,2)-rotation(2,3); ...
        rotation(1,3)-rotation(3,1); rotation(2,1)-rotation(1,2)];
end
end

function rotation = vectorAlignment(from, to)
crossVector = cross(from, to);
sineSquared = dot(crossVector, crossVector);
cosine = max(-1, min(1, dot(from, to)));
if sineSquared < 1e-12
    rotation = eye(3);
    return;
end
skew = [0 -crossVector(3) crossVector(2); crossVector(3) 0 -crossVector(1); ...
    -crossVector(2) crossVector(1) 0];
rotation = eye(3) + skew + skew * skew * ((1 - cosine) / sineSquared);
end
