# tr2_essentials
This is a ROS package that is filled with everything you need to get started with your TR2. Work-In-Progress packages are denoted by the **[WIP]** tag. This package includes...
 - tr2_description - *XACRO and URDF description of the robot for simulations and visualizations*
 - tr2py - *ROS-agnostic Python library for controlling the TR2 and receiving robot state*
 - tr2_moveit - *Integration and config for controlling the TR2 using MoveIt, an inverse kinematics package*
 - tr2_jetson_setup - *Includes the install scripts and config for setting up TR2 on a brand-new Jetson board*
 - tr2_gazebo - *Integration and config for simulating the TR2 in Gazebo 1.6*
 - tr2_navigation **[WIP]** - *Integration and config for the standard ROS navigation stack*
 - tr2cpp **[WIP]** - *ROS-agnostic C++ library for controlling the TR2 and receiving robot state*
 - tr2_hardware_interface **[WIP]** - *Config and hardware interface for controlling the robot with ros_control*

## Getting Started
To get started controlling your TR2, simply open up your bash terminal on your TR2 and execute
```sh
roslaunch tr2py tr2_node.launch
```
This will launch a node in ROS which provides an interface for controlling the TR2. Execute `rostopic list` in a separate bash terminal to view a list of all available rostopics for the TR2. All TR2 rostopics are prefixed with ``/tr2/``.

In order to view the state of the robot and its joint positions, enter the following into a separate bash terminal:
```sh
rostopic echo /tr2/state
```
This will echo a `JointState` message filled with joint positions every time the computer receives updated state from the actuator network controller.

### Actuator Modes and Emergency Stop
In order to begin publishing commands to the robot, you must run a few commands to get the TR2 ready.

The first command puts all actuators on the TR2 into Servo Mode. This is a mode that accepts position data and actively controls the actuators to reach their goal positions. There are two other modes: Rotate Mode (`/tr2/mode/rotate`) and Backdrive Mode (`/tr2/mode/backdrive`).
```sh
rostopic pub -1 /tr2/mode/servo std_msgs/Bool 1
```

You begin to see the actuators adjust slightly as they attempt to maintain their positions. If you try to physically move an actuator, it will resist you -- applying power to the motor in the opposite direction.

If you do not see any of these signs, your TR2 might be flagged for emergency stop, which will tells the actuators to ignore all commands and not move. In order to "release" the actuators, simply send the following command:
```sh
rostopic pub -1 /tr2/stop std_msgs/Bool 0
```

Note that you can set the TR2 back into emergency stop by setting the `/tr2/stop` rostopic to TRUE:
```sh
rostopic pub -1 /tr2/stop std_msgs/Bool 1
```

### Publishing Positions
Now that your TR2 is set to Servo Mode, and it is "released" from emergency stop, you can set positions of your actuators by sending position data to `/tr2/joints/[ACTUATOR_ID]/control/position`. Effort commands can be sent to `/tr2/joints/[ACTUATOR_ID]/control/effort`, but the robot must placed into Rotate Mode by sending a TRUE value to `/tr2/mode/rotate`.

First, let's grab the current position of the `a4` actuator so that we can have a reference point for future commands:
```sh
rostopic echo /tr2/joints/a4/state
```

This will echo a Float value of the position as data comes in from the actuator network controller. The values represent the angle of the actuator in radians. For those so inclined, 0.0 to 6.28 radians is roughly equivalent to 0 to 360 degrees.

Let's assume the value that we receive is roughly `5.5` radians, and we want to rotate the angle of `a4` 0.5 radians. In order to do this, we simply add 0.5 radians to 5.5 radians and publish that position on `/tr2/joints/a4/control/position`.
```sh
rostopic pub -1 /tr2/joints/a4/control/position std_msgs/Float64 6.0
```

You should now see the TR2's a4 actuator move 0.5 radians to a final value of 6.0. You can double-check that this has executed properly by checking the position data on `/tr2/joints/a4/state` as mentioned earlier.
