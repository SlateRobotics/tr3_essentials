# tr3_essentials
This is a ROS package that is filled with everything you need to get started with your TR3. Work-In-Progress packages are denoted by the **[WIP]** tag. This package includes...
 - tr3_description - *XACRO and URDF description of the robot for simulations and visualizations*
 - tr3py - *ROS-agnostic Python library for controlling the TR3 and receiving robot state*
 - tr3_os - *GUI for controlling the robot using its embedded touch screen (or on your phone!)*
 - tr3_moveit - *Integration and config for controlling the TR3 using MoveIt, an inverse kinematics package*
 - tr3_setup - *Includes the install scripts and config for setting up TR3 on a brand-new Jetson board or desktop*
 - tr3_gazebo - *Integration and config for simulating the TR3 in Gazebo 1.6*
 - tr3_navigation **[WIP]** - *Integration and config for the standard ROS navigation stack*

## Getting Started
To get started controlling your TR3, simply open up your bash terminal on your TR3 and execute
```sh
roslaunch tr3py tr3_node.launch
```
This will launch a node in ROS which provides an interface for controlling the TR3. Execute `rostopic list` in a separate bash terminal to view a list of all available rostopics for the TR3. All TR3 rostopics are prefixed with ``/tr3/``.

In order to view the state of the robot and its joint positions, enter the following into a separate bash terminal:
```sh
rostopic echo /tr3/state
```
This will echo a `JointState` message filled with joint positions every time the computer receives updated state from the actuator network controller.

### Actuator Modes and Emergency Stop
In order to begin publishing commands to the robot, you must run a few commands to get the TR3 ready.

The first command puts all actuators on the TR3 into Servo Mode. This is a mode that accepts position data and actively controls the actuators to reach their goal positions. There are two other modes: Rotate Mode (`/tr3/mode/rotate`) and Backdrive Mode (`/tr3/mode/backdrive`).
```sh
rostopic pub -1 /tr3/mode/servo std_msgs/Bool 1
```

You begin to see the actuators adjust slightly as they attempt to maintain their positions. If you try to physically move an actuator, it will resist you -- applying power to the motor in the opposite direction.

If you do not see any of these signs, your TR3 might be flagged for emergency stop, which will tells the actuators to ignore all commands and not move. In order to "release" the actuators, simply send the following command:
```sh
rostopic pub -1 /tr3/stop std_msgs/Bool 0
```

Note that you can set the TR3 back into emergency stop by setting the `/tr3/stop` rostopic to TRUE:
```sh
rostopic pub -1 /tr3/stop std_msgs/Bool 1
```

### Publishing Positions
Now that your TR3 is set to Servo Mode, and it is "released" from emergency stop, you can set positions of your actuators by sending position data to `/tr3/joints/[ACTUATOR_ID]/control/position`. Effort commands can be sent to `/tr3/joints/[ACTUATOR_ID]/control/effort`, but the robot must placed into Rotate Mode by sending a TRUE value to `/tr3/mode/rotate`.

First, let's grab the current position of the `a4` actuator so that we can have a reference point for future commands:
```sh
rostopic echo /tr3/joints/a4/state
```

This will echo a Float value of the position as data comes in from the actuator network controller. The values represent the angle of the actuator in radians. For those so inclined, 0.0 to 6.28 radians is roughly equivalent to 0 to 360 degrees.

Let's assume the value that we receive is roughly `5.5` radians, and we want to rotate the angle of `a4` 0.5 radians. In order to do this, we simply add 0.5 radians to 5.5 radians and publish that position on `/tr3/joints/a4/control/position`.
```sh
rostopic pub -1 /tr3/joints/a4/control/position std_msgs/Float64 6.0
```

You should now see the TR3's a4 actuator move 0.5 radians to a final value of 6.0. You can double-check that this has executed properly by checking the position data on `/tr3/joints/a4/state` as mentioned earlier.
