# ARM IMU Live Pose

1. Build and flash the `ROBOT=arm_imu`, `BOARD=single` firmware.
2. Disconnect Ozone, Cortex-Debug, OpenOCD, and other J-Link clients.
3. Double-click `start_arm_pose.bat`.
4. Close the MATLAB figure to stop `JLinkRTTLogger` and release the probe.

The MATLAB program starts the RTT logger automatically. It only reads the RTT
buffer; it does not flash, reset, or write target memory.

Firmware RTT telemetry is isolated in `apps/arm_imu/single_board/imu_debug.c`.
Set `ARM_IMU_DEBUG_ENABLE` to `0` in `apps/arm_imu/robot.cmake` to remove the
debug thread and all `ARMPOSE` output from the firmware.

Press the DJI C board's `KEY` button (PA0) to capture the current valid upper-arm,
forearm, and hand orientations as the new software zero pose. The request remains
pending until all three IMU streams are valid. `Zero count` increments after the
new zero pose is actually captured.

The upper-arm, forearm, and hand frames are drawn side by side. Their ZYX Euler
curves and `[w,x,y,z]` quaternions are relative to the latest software zero pose
after calibration. New 18-field `ARMPOSE` records carry the hand BMI088 data;
the parser still accepts the previous 11- and 13-field records. The raw sensor
values remain available in the firmware's Ozone watch variables.
