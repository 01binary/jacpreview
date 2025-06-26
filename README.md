# Jacobian Preview

React application that can preview Jacobian and End-Effector matrices from MoveIt 2 KDL plugin.

## Install Packages

```
npm install
sudo apt install ros-humble-rosbridge-server
source ~/.bashrc
```

## Launch ROS2 Bridge

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Launch Tool

```
npm run dev
```