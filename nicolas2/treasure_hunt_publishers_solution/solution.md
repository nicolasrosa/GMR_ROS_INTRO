# [treasure_hunt_publishers] Solution

## Step 0

```shell
$ rostopic echo /000start_here000
data: "Step -1!# Next step contains a std_msgs/Int64MultiArray message. Use $rostopic echo\
  \ /step0. The message is coded (just cast each number to char)! Hint: You can always\
  \ use Internet... maybe an online int to char converters here? Hint 2: just in case\
  \ you forgot, be aware that topics start are referred to with an / and terminal\
  \ commands with $."

$ rostopic echo /step0
layout: 
  dim: []
  data_offset: 0
data: [70, 111, 111, 108, 115, 33, 32, 73, 116, 32, 119, 97, 115, 32, 116, 117, 116, 111, 114, 105, 97, 108, 32, 116, 105, 108, 108, 32, 110, 111, 119, 33, 32, 35, 83, 116, 101, 112, 32, 48, 33, 35, 32, 74, 117, 115, 116, 32, 47, 115, 116, 97, 114, 116, 95, 97, 103, 97, 105, 110, 32, 120, 80, 0]
```

**Decoded Message:** Fools! It was tutorial till now! #Step 0!# Just /start_again xP.



## Step 1

```shell
$ rostopic echo /start_again
data: "#Step1!# Use $rostopic info to find which topic has the control_msgs/JointTolerance\
  \ message. Hint: do you know about the control_msgs/JointTolerance? Google it! Doesn't\
  \ it remind you of physics?"

$ rostopic find control_msgs/JointTolerance
/some_wrong_physics

$ rosmsg info control_msgs/JointTolerance
string name
float64 position
float64 velocity
float64 acceleration
```



## Step 2

```shell
$ rostopic echo /some_wrong_physics
name: "#Step2!# Learn more about /more_numbers! Do you know what type of message it uses?\
  \ Then you can subscribe to it! The message is coded (You must add velocity to each\
  \ number, divide by acceleration and finally subtract position). Don't be lazy and\
  \ make a new package: On src directory, catkin_create_pkg <package_name> <dependency_0>\
  \ ... <dependency_n>"
position: 14.0
velocity: 18.0
acceleration: 2.0

$ rostopic info /more_numbers
Type: std_msgs/Int16MultiArray

Publishers: 
 * /follow_the_hints (http://nicolas-X51-Ubuntu:46819/)

Subscribers: None

$ rosrun sub_more_numbers sub_more_numbers_node
data_decoded: [35, 83, 116, 101, 112, 51, 33, 35, 32, 68, 111, 32, 121, 111, 117, 32, 107, 110, 111, 119, 32, 119, 104, 97, 116, 39, 115, 32, 116, 104, 101, 32, 97, 118, 101, 114, 97, 103, 101, 32, 112, 117, 98, 108, 105, 115, 104, 105, 110, 103, 32, 114, 97, 116, 101, 32, 111, 102, 32, 47, 101, 118, 101, 110, 95, 109, 111, 114, 101, 95, 110, 117, 109, 98, 101, 114, 115, 63, 32, 84, 104, 101, 32, 99, 108, 111, 115, 101, 115, 116, 32, 105, 110, 116, 101, 103, 101, 114, 32, 116, 111, 32, 116, 104, 97, 116, 32, 114, 97, 116, 101, 32, 109, 117, 108, 116, 105, 112, 108, 105, 101, 115, 32, 101, 118, 101, 114, 121, 32, 110, 117, 109, 98, 101, 114, 32, 111, 102, 32, 47, 101, 118, 101, 110, 95, 109, 111, 114, 101, 95, 110, 117, 109, 98, 101, 114, 115, 46, 46, 46, 0, ]
```

**Decoded Message:** #Step3!# Do you know what's the average publishing rate of /even_more_numbers? The closest integer to that rate multiplies every number of /even_more_numbers...



## Step 3

```shell
$ rostopic rate /even_more_numbers
average rate: 49.980
	min: 0.018s max: 0.022s std dev: 0.00048s window: 48
average rate: 49.990
	min: 0.018s max: 0.022s std dev: 0.00037s window: 98

$ rostopic info /even_more_numbers
Type: std_msgs/Int16MultiArray

Publishers: 
 * /follow_the_hints (http://nicolas-X51-Ubuntu:46819/)

Subscribers: None
$ rosrun sub_even_more_numbers sub_even_more_numbers_node
data_decoded: [35, 83, 116, 101, 112, 52, 33, 35, 32, 68, 111, 110, 39, 116, 32, 102, 111, 114, 103, 101, 116, 32, 116, 111, 32, 112, 117, 98, 108, 105, 115, 104, 32, 121, 111, 117, 114, 32, 115, 116, 97, 116, 117, 115, 33, 32, 83, 101, 114, 105, 111, 117, 115, 108, 121, 44, 32, 105, 116, 39, 115, 32, 114, 101, 97, 108, 108, 121, 32, 105, 109, 112, 111, 114, 116, 97, 110, 116, 46, 32, 84, 104, 101, 121, 39, 114, 101, 32, 115, 111, 32, 105, 109, 112, 111, 114, 116, 97, 110, 116, 32, 116, 104, 97, 116, 32, 111, 110, 101, 32, 111, 102, 32, 116, 104, 101, 32, 115, 116, 100, 95, 109, 115, 103, 115, 47, 73, 110, 116, 54, 52, 32, 116, 111, 112, 105, 99, 115, 32, 104, 97, 115, 32, 97, 32, 104, 105, 110, 116, 46, 32, 84, 114, 97, 110, 115, 108, 97, 116, 101, 32, 105, 116, 32, 116, 111, 32, 99, 104, 111, 110, 97, 32, 40, 97, 32, 114, 97, 110, 100, 111, 109, 32, 108, 97, 110, 103, 117, 97, 103, 101, 32, 105, 110, 32, 71, 111, 111, 103, 108, 101, 32, 84, 114, 97, 110, 115, 108, 97, 116, 101, 41, 32, 97, 110, 100, 32, 121, 111, 117, 39, 108, 108, 32, 107, 110, 111, 119, 32, 116, 104, 101, 32, 110, 97, 109, 101, 32, 111, 102, 32, 110, 101, 120, 116, 32, 116, 111, 112, 105, 99, 46, 0, ]
```

**Decoded Message:** #Step4!# Don't forget to publish your status! Seriously, it's really important. They're so important that one of the std_msgs/Int64 topics has a hint. Translate it to chona (a random language in Google Translate) and you'll know the name of next topic.



## Step 4

```sh
$ rostopic find std_msgs/Int64
/hint_here

$ rosmsg info std_msgs/Int64
int64 data

$ rostopic echo /hint_here
data: 300
```

**Answer:** Chona(300) = mazana matatu



## Step 5

```shell
$ rostopic info /mazana_matatu
Type: diagnostic_msgs/DiagnosticStatus

Publishers: 
 * /follow_the_hints (http://nicolas-X51-Ubuntu:46819/)

Subscribers: None

$ rostopic echo /mazana_matatu
level: 5
name: "Message fields are important. For example, level gives your current step (just in\
  \ case you forgot) and hardware_id has the hint for next step!"
message: ''
hardware_id: "Sometimes you see a /not_a_selfie topic but you don't care about the array fields.\
  \ Extra hint: $rostopic echo <topic> can have additional arguments!"
values: []

$ rostopic info /not_a_selfie
Type: sensor_msgs/Image

Publishers: 
 * /follow_the_hints (http://nicolas-X51-Ubuntu:46819/)

Subscribers: None

$ rosmsg info sensor_msgs/Image
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```



## Step 6

```shell
$ rostopic echo /not_a_selfie --noarr
"#Step6!# I hope you used $rostopic echo /not_a_selfie --noarr to get here =] Just\
  \ in case, try again with /not_a_selfie_for_sure."

$ rostopic echo /not_a_selfie_for_sure --noarr
rosrun turtlesim turtlesim_node  __name:=my_crush turtle1/cmd_vel:=move_crush
```

**Answer:** Maori(Square) = tapawha



## Step 7

```shell
$ rostopic echo /tapawha
"I bet you forgot to publish #Step7!# status. Anyway, now you need to publish #Step8!#\
  \ Are you still changing the code and doing catkin_make every time? Why don't you\
  \ try using arguments when you call $rosrun? You may have noticed the line int main(int\
  \ argc, char* argv[]) in your nodes. argc gives you the number of arguments and\
  \ argv gives you the arguments as strings. argv[0] gives you the path to the package.\
  \ If you call $rosrun publish_my_status publish_my_status_node 8, then argv[1] =\
  \ '8'. The function atoi conveniently converts string to int and you can publish\
  \ it! ... ... ... It's not over yet! You'll learn something even more useful in\
  \ /learn_parameters"
```



## Step 8

```shell
(After creating the publish_my_status_node, run the following line)
$ rosrun publish_my_status publish_my_status_node 8

$ rosrun echo /my_status
level: 8
name: ''
message: ''
hardware_id: ''
values: []
```



## Step 9

```shell
$ rostopic echo /learn_parameters
"#Step9!# Another way of changing variables inside node is ROS parameter. Explore\
  \ $rosparam commands starting with /tutorial# params. Start with $ rosparam list."
  
$ rosparam list
/tutorial1
/tutorial2
/tutorial3

$ rosparam get /tutorial1
If there's a param, you can get it inside node using nodeHandle.getParam(<param_name>,
  var_to_store_param). Make a node to print /tutorial1 with ROS_INFO_STREAM
 
(After creating the print_param_tutorial1, run the following line)
$ rosrun print_param_tutorial1 print_param_tutorial1_node
[ INFO] [1588255313.088628585]: If there's a param, you can get it inside node using nodeHandle.getParam(<param_name>, var_to_store_param). Make a node to print /tutorial1 with ROS_INFO_STREAM

$ rosparam get /tutorial2

$ rosparam set /nicolas_status 9

$ rosparam get /myname_status
9

$ rosparam get /tutorial3
'Or you can set the param inside your node: nodeHandle.setParam(<param_name>, new_value).
  Did you know isipho is gift in zulu? No? Do not waste more time! Collect your four
  isiphos disguised as params!'

$ rosparam list | grep "isipho"
/red_line/small_isipho
/red_line/medium_isipho
/red_line/big_isipho
/red_line/monstrous_isipho

$ rosparam get /red_line/small_isipho
Have you checked all tutorial params? Anyway, you just found an empty /key to unlock
  the publisher of a certain node. It may be obvious but just in case you don't find
  the node, search for std_msgs/Int32MultiArray messages. Keep echoing the topic until
  you set the /key =]
  
$ rostopic find std_msgs/Int32MultiArray
/unlock_me_if_you_can

$ rosparam get /red_line/medium_isipho
You must find the IP of the machine that's publishing all this stuff... rosnode may
  help you in your journey. Or you can find where's the master
  
$ rosparam get /red_line/big_isipho 
Your /key to unlock the publisher shall be the master's IP!!!
```
**Answer:** nicolas-X51-Ubuntu

```shell
(Let this topic echoing)
$ rostopic echo /unlock_me_if_you_can

(In another terminal session run)
$ rosparam set /key nicolas-X51-Ubuntu

(/unlock_me_if_you_can output)
layout: 
  dim: []
  data_offset: 0
data: [2003, 2028, 2076, 2109, 2094, 2105, 2042, 2041, 2026, 2028, 2025, 2060, 2104, 2103, 2096, 2107, 2090, 2109, 2108, 2026, 2025, 2071, 2104, 2112, 2025, 2114, 2104, 2110, 2025, 2097, 2090, 2111, 2094, 2025, 2090, 2025, 2096, 2107, 2090, 2108, 2105, 2025, 2090, 2091, 2104, 2110, 2109, 2025, 2107, 2104, 2108, 2105, 2090, 2107, 2090, 2102, 2039, 2025, 2077, 2097, 2094, 2025, 2095, 2098, 2103, 2090, 2101, 2025, 2109, 2104, 2105, 2098, 2092, 2025, 2098, 2108, 2025, 2040, 2107, 2094, 2093, 2088, 2101, 2098, 2103, 2094, 2039, 2025, 2075, 2110, 2103, 2025, 2029, 2025, 2107, 2106, 2109, 2088, 2098, 2102, 2090, 2096, 2094, 2088, 2111, 2098, 2094, 2112, 2025, 2040, 2107, 2094, 2093, 2088, 2101, 2098, 2103, 2094, 2025, 1993]
---
```

```shell
(Checking the last gift...)
$ rosparam get /red_line/monstrous_isipho
Remove a certain number from each element of the array and you will be able to cast
  it to char. Do you know about ros services? They are useful to execute request/reply
  interactions. Explore $ rosservice to find the genie that will grant your desire
  when called
  
$ rosservice call /genie_the_server
success: True
message: "Your magic number is 1993. Use it wisely"

(In different terminal sessions run)
$ rostopic echo /unlock_me_if_you_can
$ rosrun sub_unlock_me_if_you_can sub_unlock_me_if_you_can_node
Then...
$ rosparam set /key nicolas-X51-Ubuntu

(sub_Unlock_me_if_you_can_node output)
data_decoded: [10, 35, 83, 116, 101, 112, 49, 48, 33, 35, 32, 67, 111, 110, 103, 114, 97, 116, 115, 33, 32, 78, 111, 119, 32, 121, 111, 117, 32, 104, 97, 118, 101, 32, 97, 32, 103, 114, 97, 115, 112, 32, 97, 98, 111, 117, 116, 32, 114, 111, 115, 112, 97, 114, 97, 109, 46, 32, 84, 104, 101, 32, 102, 105, 110, 97, 108, 32, 116, 111, 112, 105, 99, 32, 105, 115, 32, 47, 114, 101, 100, 95, 108, 105, 110, 101, 46, 32, 82, 117, 110, 32, 36, 32, 114, 113, 116, 95, 105, 109, 97, 103, 101, 95, 118, 105, 101, 119, 32, 47, 114, 101, 100, 95, 108, 105, 110, 101, 32, 0, ]
```

**Decoded Message:** #Step10!# Congrats! Now you have a grasp about rosparam. The final topic is /red_line. Run $ rqt_image_view /red_line



# Step 10

```shell
$ rostopic info /red_line
Type: sensor_msgs/Image

Publishers: 
 * /follow_the_hints (http://nicolas-X51-Ubuntu:40165/)

Subscribers: None

$ rostopic echo /red_line --noarr
header: 
  seq: 1825
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
height: 739
width: 529
encoding: "bgr8"
is_bigendian: 0
step: 1587
data: "<array type: uint8, length: 1172793>"

$ rqt_image_view /red_line
```

![](./.end.png)