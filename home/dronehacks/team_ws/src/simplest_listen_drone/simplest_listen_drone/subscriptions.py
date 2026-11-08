"""Every topic we listen to, one per line. The node subscribes to each line that is not commented out.

Names only: the node asks the live ROS graph for each topic's message type, so a wrong guess about
a type can never silently receive nothing. A PX4 name like /fmu/out/vehicle_status also matches
its versioned spelling (/fmu/out/vehicle_status_v1 on PX4 v1.16).

Sources for the list: the kit's README topic table and competitor guide (drone-snapshot/home/
dronehacks/skybeta), PX4 v1.16's dds_topics.yaml as modelled in DroneHacks sitl/dronesim/px4/uxrce.py,
and the drone's own bench check (27 topics under /fmu/out).

=====================================================================================================
WHAT DO WE ACTUALLY NEED TO HEAR?  A thread of reasoning, from the problem down to the topics.
=====================================================================================================

The problem (field guide, dronehacks-field-guide.html). Fly a 6.1 m netted course on our own, no
GPS, no pilot: take off from the launch mark, through the maze (CP1, CP2 Gate A), the dark tunnel
(CP3, 10 points, the biggest), the chicane through Gate B or the 0.91 m window (+5), onto the pad
(land and disarm inside the ring). Checkpoints count only in order. Stay under the 1.73 m walls.
+5 s per touch; a crash, a ground touch or 3 minutes ends the run. Every finish beats every
non-finish, so: finish first, then go faster. The narrowest gap leaves +-0.18 m for all error.
The layout "may change slightly", and generality is 15 points: plan from the map, not hard-coded
coordinates.

So the drone has to answer four questions, all the time:
  1. Where am I?               (x, y, z, heading, in the course's frame)
  2. Can I trust that?         (is the answer fresh, consistent, actually being used by PX4)
  3. What is around me?        (walls, the tunnel, the window, boulders, the pad)
  4. What is PX4 doing?        (armed, Offboard, landed, did the kit clamp my command)
And the course adds a fifth that is ours, not a sensor's: which checkpoint is next (an ordered
list from the map; a sensor only tells us when we have passed one).

1. WHERE AM I.  Three independent sources, each with a known weakness:
   - VIO: cuVSLAM on the stereo pair -> /visual_slam/tracking/odometry, 30 Hz, x/y/z/heading from
     the power-on pose. Drifts slowly; is CONFIDENTLY WRONG on a blank view (field guide: a grey
     wall reports "standing still" with hundreds of features); goes blind in the dark.
   - PX4's EKF2 -> /fmu/out/vehicle_local_position (and _odometry, _attitude). It fuses VIO with
     the Pixhawk's own IMU and the downward rangefinder, so it coasts through short vision drops
     and its height does not drift. It is mostly VIO underneath, so it shares VIO's long-term drift.
   - The rangefinder (VL53L1X, down, 0-4 m, narrow beam). It is NOT bridged as its own topic; it
     arrives inside vehicle_local_position as dist_bottom (+ dist_bottom_valid). Height above
     whatever is directly below: the floor, or a 0.53 m boulder.
   The idea that falls out: use VIO (what the proven square flies on) for position, use EKF2 and
   the rangefinder to CHECK it. Over the course the map can correct both (match what the camera
   sees to the walls the map says are there; the field guide's "map correction"), but that is
   later.

2. CAN I TRUST IT.  Trust is fusion's first and cheapest job: comparing sources that fail
   differently. Signals:
   - /visual_slam/status: vo_state 1 tracking, 2 lost. Direct, but misses the blank-view case.
   - age of the last odometry message: stale = no pose.
   - VIO vs EKF2 horizontal position: they diverge when PX4 stops accepting vision.
   - VIO heading vs EKF2 heading (/fmu/out/vehicle_attitude): the field guide's heading gate
     ("start the route only when PX4's heading agrees with vision's within 2 deg"; a wrong
     start-up yaw bias flew a turned route).
   - /fmu/out/estimator_status_flags: cs_ev_pos / cs_ev_yaw say whether EKF2 is fusing vision
     at all, cs_rng_hgt whether it uses the rangefinder for height.
   - the image itself (/oak/left/image_mono): a blank or dark frame is exactly when VIO lies.
     Texture or brightness is the honest signal for "the camera can't see" (the field guide's
     guard withholds frames under 1% texture).

3. WHAT IS AROUND ME.  The course is mostly known (the map); what we must sense is where we are
   in it and what changed:
   - walls / corridors: the map, plus our position (1). The camera could see them (stereo depth),
     but depth costs CPU next to cuVSLAM and saw through the net in simulation: later, if at all.
   - obstacles below: the rangefinder. Camera height minus dist_bottom ~ 0.5 m means a boulder
     is under us. Free obstacle sensing, no new topic needed.
   - the tunnel: from the inside the image goes dark except the exit. Image brightness tells us
     we are in it; heading along its axis keeps the exit in view.
   - the window / gates / pad: positions come from the map; the camera could confirm them
     (a bright square opening, a ring on the floor), later.
   - ordered checkpoints: a list from the map plus position (1). No topic.

4. WHAT IS PX4 DOING.  /skybeta/state already carries armed, flight mode, offboard_active,
   landed, PX4's local position and the kit's clamps, at 5 Hz. The raw /fmu/out versions
   (vehicle_status, vehicle_land_detected, vehicle_control_mode) say the same thing in more
   detail. One source is enough: /skybeta/state, because it is the kit's contract.

What we can drop, and why:
   - /oak/right/*, the bgr8 image_raw pair: cuVSLAM already uses both eyes; for brightness and
     texture one mono image is enough, and the bgr8 copies are 3x the bytes.
   - /oak/imu/data, /fmu/out/sensor_combined: PX4 already fuses the IMU (the field guide: stereo-only
     cuVSLAM + EKF2's inertial fusion flew better than cuVSLAM's inertial mode).
   - vo_pose, vo_pose_covariance, vo_path, slam_path: repeats of odometry; its covariance is not
     an honest uncertainty (field guide).
   - /skybeta/setpoint/*, /fmu/in/*: our own commands echoed back. Useful for debugging, not for
     deciding.
   - vehicle_status, land_detected, control_mode, home_position, command_ack,
     position_setpoint_triplet: covered by /skybeta/state.
   - vehicle_odometry: same as local_position + attitude.
   - failsafe_flags, battery_status: the organisers' failsafes act on these (rule 8.5); a 3-minute
     run will not drain a 5000 mAh pack.
   - timesync_status: matters (a stepped clock once made vision stale), but it is a pre-flight
     check, not an input. Back on if fusion starts seeing old poses.
   - manual_control_setpoint: the pilot's sticks; any human input other than abort ends the run.
   - global / GPS position: no GPS, and GPS is banned (rule 6.1).
   - /tf, /tf_static, camera_info: needed once we turn pixels into directions (the camera mount
     and lens). Left camera_info stays on now (tiny); tf_static comes back with that step.
   - /diagnostics, /rosout, and the catch-all: for humans watching, not for the drone.

THE RELEVANT SUBSET (what the first sensor-fusion rung reads):
RELEVANT = ['/visual_slam/tracking/odometry', '/visual_slam/status', '/skybeta/state', '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude', '/fmu/out/estimator_status_flags', '/oak/left/image_mono', '/oak/left/camera_info']
"""

TOPICS = [
    # ---- the camera: OAK-D S2 through depthai-ros (30 Hz stereo, 200 Hz IMU) ----
    # ('/oak/left/image_raw',          'left stereo image, bgr8 1024x640, 30 Hz'),
    # ('/oak/right/image_raw',         'right stereo image, bgr8 1024x640, 30 Hz'),
    ('/oak/left/image_mono',         'left image as mono8 (what cuVSLAM reads)'),
    # ('/oak/right/image_mono',        'right image as mono8 (what cuVSLAM reads)'),
    ('/oak/left/camera_info',        'left lens calibration'),
    # ('/oak/right/camera_info',       'right lens calibration'),
    # ('/oak/imu/data',                'camera IMU (BNO086), ~200 Hz'),

    # ---- cuVSLAM: where the camera thinks it is ----
    ('/visual_slam/tracking/odometry',           'camera pose + velocity, ENU odom, 30 Hz'),
    # ('/visual_slam/tracking/vo_pose',            'camera pose only'),
    # ('/visual_slam/tracking/vo_pose_covariance', 'camera pose with covariance'),
    # ('/visual_slam/tracking/vo_path',            'camera path so far'),
    # ('/visual_slam/tracking/slam_path',          'SLAM path so far (loop-closed)'),
    ('/visual_slam/status',                      'tracking state: vo_state 1 tracking / 2 lost'),

    # ---- the kit's flight interface ----
    ('/skybeta/state',               'PX4 summary as JSON, 5 Hz'),
    # ('/skybeta/setpoint/pose',       'pose setpoints someone is sending the kit'),
    # ('/skybeta/setpoint/twist',      'velocity setpoints someone is sending the kit'),

    # ---- what the kit sends PX4 (reading is harmless; never publish here) ----
    # ('/fmu/in/vehicle_visual_odometry', 'camera pose as PX4 receives it (the bridge)'),
    # ('/fmu/in/trajectory_setpoint',     'setpoint as PX4 receives it'),
    # ('/fmu/in/offboard_control_mode',   'offboard heartbeat, 10 Hz'),
    # ('/fmu/in/vehicle_command',         'commands to PX4'),

    # ---- PX4, the flight controller (/fmu/out, best effort) ----
    # ('/fmu/out/vehicle_status',          'arming, flight mode, failsafe'),
    ('/fmu/out/vehicle_local_position',  'EKF2 fused position + velocity; dist_bottom = rangefinder'),
    ('/fmu/out/vehicle_attitude',        'EKF2 attitude quaternion (heading)'),
    # ('/fmu/out/vehicle_odometry',        'EKF2 pose + velocity in one message'),
    # ('/fmu/out/vehicle_land_detected',   'landed / maybe landed / ground contact'),
    # ('/fmu/out/vehicle_control_mode',    'which controllers are active'),
    ('/fmu/out/estimator_status_flags',  'what EKF2 is fusing: vision pos/yaw, rangefinder height'),
    # ('/fmu/out/failsafe_flags',          'every failsafe condition'),
    # ('/fmu/out/battery_status',          'battery voltage, current, remaining'),
    # ('/fmu/out/sensor_combined',         'Pixhawk IMU: gyro + accel'),
    # ('/fmu/out/timesync_status',         'Jetson <-> PX4 clock sync'),
    # ('/fmu/out/manual_control_setpoint', 'the pilot\'s sticks'),
    # ('/fmu/out/home_position',           'where PX4 armed'),
    # ('/fmu/out/vehicle_command_ack',     'PX4\'s answers to commands'),
    # ('/fmu/out/position_setpoint_triplet', 'PX4\'s current navigation setpoints'),
    # ('/fmu/out/vehicle_global_position', 'global position (no GPS here: invalid)'),
    # ('/fmu/out/vehicle_gps_position',    'GPS (none fitted; banned by rule 6.1)'),

    # ---- ROS plumbing ----
    # ('/tf',                          'moving transforms'),
    # ('/tf_static',                   'fixed transforms: base_link -> camera'),
    # ('/diagnostics',                 'node health reports'),
    # ('/rosout',                      'every node\'s log'),
]

# Anything else on the live graph is also heard (rate only) while CATCH_ALL is True, except NEVER:
# image_transport's encoded copies (/compressed, /compressedDepth, /theora) are only produced while
# someone listens, so listening would make the Jetson encode video next to cuVSLAM.
CATCH_ALL = False          # was True while we surveyed everything; the subset above is the decision
NEVER = ('/compressed', '/compressedDepth', '/theora', '/parameter_events', '/simplest_listen_drone/')

# The subset, as data, for the next rung to import (must match the docstring's line and TOPICS).
RELEVANT = ['/visual_slam/tracking/odometry', '/visual_slam/status', '/skybeta/state',
            '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude',
            '/fmu/out/estimator_status_flags', '/oak/left/image_mono', '/oak/left/camera_info']
