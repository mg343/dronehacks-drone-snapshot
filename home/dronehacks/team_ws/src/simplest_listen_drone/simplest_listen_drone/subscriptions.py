"""Every topic we listen to, one per line. The node subscribes to each line that is not commented out.

Names only: the node asks the live ROS graph for each topic's message type, so a wrong guess about
a type can never silently receive nothing. A PX4 name like /fmu/out/vehicle_status also matches
its versioned spelling (/fmu/out/vehicle_status_v1 on PX4 v1.16).

Sources for the list: the kit's README topic table and competitor guide (drone-snapshot/home/
dronehacks/skybeta), PX4 v1.16's dds_topics.yaml as modelled in DroneHacks sitl/dronesim/px4/uxrce.py,
and the drone's own bench check (27 topics under /fmu/out).
"""

TOPICS = [
    # ---- the camera: OAK-D S2 through depthai-ros (30 Hz stereo, 200 Hz IMU) ----
    ('/oak/left/image_raw',          'left stereo image, bgr8 1024x640, 30 Hz'),
    ('/oak/right/image_raw',         'right stereo image, bgr8 1024x640, 30 Hz'),
    ('/oak/left/image_mono',         'left image as mono8 (what cuVSLAM reads)'),
    ('/oak/right/image_mono',        'right image as mono8 (what cuVSLAM reads)'),
    ('/oak/left/camera_info',        'left lens calibration'),
    ('/oak/right/camera_info',       'right lens calibration'),
    ('/oak/imu/data',                'camera IMU (BNO086), ~200 Hz'),

    # ---- cuVSLAM: where the camera thinks it is ----
    ('/visual_slam/tracking/odometry',           'camera pose + velocity, ENU odom, 30 Hz'),
    ('/visual_slam/tracking/vo_pose',            'camera pose only'),
    ('/visual_slam/tracking/vo_pose_covariance', 'camera pose with covariance'),
    ('/visual_slam/tracking/vo_path',            'camera path so far'),
    ('/visual_slam/tracking/slam_path',          'SLAM path so far (loop-closed)'),
    ('/visual_slam/status',                      'tracking state: vo_state 1 tracking / 2 lost'),

    # ---- the kit's flight interface ----
    ('/skybeta/state',               'PX4 summary as JSON, 5 Hz'),
    ('/skybeta/setpoint/pose',       'pose setpoints someone is sending the kit'),
    ('/skybeta/setpoint/twist',      'velocity setpoints someone is sending the kit'),

    # ---- what the kit sends PX4 (reading is harmless; never publish here) ----
    ('/fmu/in/vehicle_visual_odometry', 'camera pose as PX4 receives it (the bridge)'),
    ('/fmu/in/trajectory_setpoint',     'setpoint as PX4 receives it'),
    ('/fmu/in/offboard_control_mode',   'offboard heartbeat, 10 Hz'),
    ('/fmu/in/vehicle_command',         'commands to PX4'),

    # ---- PX4, the flight controller (/fmu/out, best effort) ----
    ('/fmu/out/vehicle_status',          'arming, flight mode, failsafe'),
    ('/fmu/out/vehicle_local_position',  'EKF2 fused position + velocity; dist_bottom = rangefinder'),
    ('/fmu/out/vehicle_attitude',        'EKF2 attitude quaternion (heading)'),
    ('/fmu/out/vehicle_odometry',        'EKF2 pose + velocity in one message'),
    ('/fmu/out/vehicle_land_detected',   'landed / maybe landed / ground contact'),
    ('/fmu/out/vehicle_control_mode',    'which controllers are active'),
    ('/fmu/out/estimator_status_flags',  'what EKF2 is fusing: vision pos/yaw, rangefinder height'),
    ('/fmu/out/failsafe_flags',          'every failsafe condition'),
    ('/fmu/out/battery_status',          'battery voltage, current, remaining'),
    ('/fmu/out/sensor_combined',         'Pixhawk IMU: gyro + accel'),
    ('/fmu/out/timesync_status',         'Jetson <-> PX4 clock sync'),
    ('/fmu/out/manual_control_setpoint', 'the pilot\'s sticks'),
    ('/fmu/out/home_position',           'where PX4 armed'),
    ('/fmu/out/vehicle_command_ack',     'PX4\'s answers to commands'),
    ('/fmu/out/position_setpoint_triplet', 'PX4\'s current navigation setpoints'),
    ('/fmu/out/vehicle_global_position', 'global position (no GPS here: invalid)'),
    ('/fmu/out/vehicle_gps_position',    'GPS (none fitted; banned by rule 6.1)'),

    # ---- ROS plumbing ----
    ('/tf',                          'moving transforms'),
    ('/tf_static',                   'fixed transforms: base_link -> camera'),
    ('/diagnostics',                 'node health reports'),
    ('/rosout',                      'every node\'s log'),
]

# Anything else on the live graph is also heard (rate only), except these: image_transport's
# encoded copies (/compressed, /compressedDepth, /theora) are only produced while someone listens,
# so listening would make the Jetson encode video next to cuVSLAM.
CATCH_ALL = True
NEVER = ('/compressed', '/compressedDepth', '/theora', '/parameter_events', '/simplest_listen_drone/')
