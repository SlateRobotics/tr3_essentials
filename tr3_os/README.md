# tr-os [WIP]
GUI for controlling TR3 robots on its embedded screen

The primary goal of this project is to provide a quick and easy way for users to do stuff with their robot. We are providing a type of "operating system" that will house the apps that run on the robot, will manage state, commands and other data with the robot hardware, and will enable a multitude of apps to be installed and executed on the robot.

Eventually, we will have a type of "App Store" so that users can find applications that are most useful or interesting to them. This is one of those ideas that sound really sexy to investors -- but our primary purpose with this is to best serve our customers. We envision a future where, if you need help setting the table for dinner, you simply download an app that will do it for you.

This is a logical progression of the mission of the company. We eventually want users who know nothing about ROS or software development, and they need to be able to do stuff on the robot without the learning curve. This is the solution for that problem.

# To Do
  - FRV
    - Navigation
      - Obstacle avoidance
      - Update speeds (max wheel vel is ~1.0)
      - Odom drift
    - 3D Render
      - Options:
        - Display LiDAR, Point Cloud, Map
    - Arm control
      - Goal position coordinate frame: robot vs. world
      - Gripper control
    - Console
  - Control Panel
    - Waypoint record and playback
  - MoveIt Tuning/Config
  - Other
    - Inverse Kinematics
  - Hardware
    - Actuator acceleration limits
    - Actuator IMU state -> ROS
