## PREREQ ##
[ROS Groovy+Turtlebot installation](http://ros.org/wiki/turtlebot/Tutorials/groovy/Installation)

## INSTALL ##

In the project source folder run
```
make
```

## RUN ##

```
cd bin
rosrun turtlebot_move turtlebot_tf_broadcaster
```
This runs a broadcaster for broadcasting the transform between _/world_ and _/turtlebot_.

```
cd bin
rosrun turtlebot_move turtlebot_move
```
This starts listening for move commands on the topic `/turtlebot_move_commands`.

A sample move command:
```
rostopic pub /turtlebot_move_commands turtlebot_move/Moves “moves: [{type: linear, value: 0.5}, {type: angular, value: 90}]”
```