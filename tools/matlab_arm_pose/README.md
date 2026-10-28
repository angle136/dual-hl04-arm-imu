# ARM IMU Live Pose

1. Build and flash the `ROBOT=arm_imu`, `BOARD=single` firmware.
2. Disconnect Ozone, Cortex-Debug, OpenOCD, and other J-Link clients.
3. Double-click `start_arm_pose.bat`.
4. Close the MATLAB figure to stop `JLinkRTTLogger` and release the probe.

The MATLAB program starts the RTT logger automatically. It only reads the RTT
buffer; it does not flash, reset, or write target memory.

Firmware RTT telemetry is isolated in `apps/arm_imu/single_board/arm_imu_debug.c`.
Set `ARM_IMU_DEBUG_ENABLE` to `0` in `apps/arm_imu/robot.cmake` to remove the
debug thread and all `ARMPOSE` output from the firmware.

Press the DJI C board's `KEY` button (PA0) to capture the current valid upper-arm
and forearm orientations as the new software zero pose. The request remains
pending until both IMU streams are valid. `Zero count` increments after the new
zero pose is actually captured.

The upper-arm frame is drawn at the origin. The forearm frame is drawn to its
right when valid CAN2 HL04 data is available. After calibration, the frames,
quaternions, and ZYX Euler angles are relative to the latest software zero pose.
The raw sensor values remain available in the firmware's Ozone watch variables.
