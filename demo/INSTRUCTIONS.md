# Demo

## 0. Preparation

* start ROS core

        roscore

* put on glove
* connect USB
* start parser

        cat /dev/ttyACM0 | rosrun imutype serial_parser.py _from_network:=false

* reconnect / reset / start parser cycle until it works
* connect webcam, start mpv

        mpv tv://

* start keyboard listener

        sudo chmod a+r /dev/input/event2
        rosrun imutype keyboard_listener.py _input_device:=/dev/input/event2

* start visualization extraction node

        rosparam set /config_file $HOME/uni/bachelor/ros/src/imutype/experiments/2017-04-12-TGBYHNUJM_/config.yaml
        rosrun imutype convert_imu_events.py

* reset IMU headings

## 1. Show hardware

* photos on slides
* rubber band attachment
* show 6 imus
* connections, wiring
* EEPROM
* USB/Battery
* processor on wrist due to weight

## 2. RViz

* open rviz

        rviz

* show MPV webcam window now
* move hand around
* shake hand to show it has minimal effect
* rotate hand to show the quaternions are relative
* positions of fingers are just for visualization, not actually measured

## 3. PlotJuggler

* open it

        set_ros_workspace $HOME/rosbuild
        rosrun plotjuggler PlotJuggler

* start streaming `/angles` topic
* set duration to 10 seconds
* add `angles.4` and `angles.6` (index & middle finger pitch)
* bend fingers separately, in opposite directions, in same direction
* turn hand upside down, repeat

## 4. Actual typing

* run apply_net in a terminal with large font

        rosrun imutype apply_net.py 2017-05-13-demo-1

* take out "paper keyboard"
* still using webcam
* Caro speaks out what to type, Paul types, everyone sees it
* make it interactive - have people ask a few questions here

## 5. Demo 2

Finish off with this nice little gem :)

        rosrun imutype apply_net.py 2017-05-13-demo-2
