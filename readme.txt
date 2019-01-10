# MicroControllers
This repository contains 2 folders. The first one ROS_MSP432_Red contains a project static library "RosMspHwdLib",
two subscriber projects and one publisher project.  In addition all the metadata to run Code Composer Studio ccsv8
on a Ubuntu 16.04 PC is included.  The compilers that were used is TI's TI v18.1.2.LTS or v18.1.4.LTS.

The static library contains the necessay hardware for a MSP432P401R, clocks and a serial io interface and is linked to the
subscriber and publisher project.

1. Subscriber "Joystick" connects to the publisher "joy", ($ rosrun joy joy-topic). It will display axis 1 & 2 values
in the CCSV8 Console Window.  It will also flash turn the Red LED on and off by pressing button 1/3.

To test:
Terminal 1: $ roscore
Terminal 2: $ rosrun joy joy_topic
Terminal 3: $ rosrun rosserial_python serial_topic.py _port:=/dev/ttyACM0

2. Subscriber "SubscriberBlinky" subscriber topic "msp432_sub" blinks the red LED when a std_msg/String is sent.

To test:
Terminal 1: $ roscore
Terminal 2: $ rosrun rosserial_python serial_topic.py _port:=/dev/ttyACM0
Terminal 3: $ rostopic pub -r .5 /msp432_sub std_msgs/String
  
3. Publisher "PublisherHelloWorld" pulishes a "Hello World!!!" message every 1 second on topic "smp432".

To test:
Terminal 1: $ roscore
Terminal 2: $ rosrun rosserial_python serial_topic.py _port:=/dev/ttyACM0
Terminal 3: $ rostopic echo msp432
  
4. The other project "RosInterface" is a publisher and subscriber.  The publisher topic is "msp432_pub and the
subscriber topic "qt_pub" which connects to a "Qt Example" project not yet uploaded. The interface uses the
std_msgs/String's in the form of "<XX,parameter?" commands and responses. The "<" and ">" are used as delimeters for the acii
messages.  These messages are easily viewed on a terminal.  The XX part of the message is a 2 character command or response formed as follows (('X'<8)|'X') into a integer value. This value then can be easily decoded through "C++" switch
statements.

To test:
Terminal 1: $ roscore
Terminal 2: $rosrun rosserial_python serial_topic.py _port:=/dev/ttyACM0
Terminal 3: $ rostopic pub -r .5 /qt_pub std_msgs/String "<GS>" # Means get string
Terminal 4. $ rostopuc echo msp432_pub
  
5. Refer to "setup.txt" in this repository for setting the environment/paths correctly.

