0.14.2 20260825
1. Add RGB DTOF filter
2. Fix USB disconnect deadlock issue
3. Add RGB parameter configuration

0.14.1 20260809
1. Fixed the SDK's USB memory leak issue
2. Fixed the problem of 'transfer no mem' caused by USB buffer full during machine transfer

0.13.1 20260724
1.modify frame_id for image topic
2.fix tf transform fail in ROS2 Jazzy
3.Fix the issue where the driver's initial configuration could not read the calibration file

0.13.0 20260709
1.add large file transfer 

v0.11.0 2026_0618
Required Minimum Firmware Version：0.12.0
1.Reduce cloud slam latency
2.Improve get mapping result 

v0.10.5 2026_0525
1.modify recorddata format，add device_id、algorithm_version key
2.fix download map fail when in slam mode(USB2.0)
3.fix upload map fail when in relocalization mode(USB2.0)

v0.10.4 2026_0522
1.fix usb2.0 heartBeat timeout to cause soft detaching
2.add custom_init_pose_search_radius and custom_init_pose_max_rot_deg in control_command.yaml