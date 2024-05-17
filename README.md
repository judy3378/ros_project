in an empty catkin_ws:
```
mkdir src
cd src
catkin_create_pkg projet rospy urdf message_runtime
```

go to projet (from catkin_ws):
```
cd src/projet
```

pull from current repo or download as zip:
copy the files into catkin_ws/src/projet


back in catkin_ws:
```
catkin_make

source ~/.bashrc
```

## Launch Challenges
To launch the challenge 1 and 2:
```
roslaunch projet challenge1and2.launch
```
To launch the challenge 3:
```
roslaunch projet challenge3.launch
```
To launch all of the challenges : 
```
roslaunch projet challenge_all.launch
```



### Structure
```
.
└── catkin_ws/
    ├── build
    ├── devel
    └── src/
        └── projet/
            ├── config/
            │   └── color_ranges.yaml
            ├── launch/
            │   ├── challenge1and2.launch
            │   └── challenge3.launch
            ├── msg/
            │   └── LinePosition.msg
            ├── rviz/
            │   ├── config.rviz
            │   └── config_cam.rviz
            ├── src
            ├── srv/
            │   └── StartCorridor.srv
            ├── urdf
            ├── worlds
            ├── CMakeLists.txt
            └── package.xml
```
