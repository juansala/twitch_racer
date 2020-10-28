# twitch_racer

Remote-controlled/autonomous rover with Pi B2, Teensy 3.2, Pololu DRV8835 motor driver, and A1M8 RPLidar. I initially created this robot to solely move based on Twitch chat commands. After getting that running, I decided to add in basic wall following, localization, and path-planning into the mix.

Here is a full list of ROS packages, both self-made and taken from public repositories:

rplidar_ros
key_teleop
serial_node
wall_follower

raspicam_node (coming soon)
robot_localization (coming soon)
