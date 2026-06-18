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