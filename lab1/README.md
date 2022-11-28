# Lab 1

Filozofia ROS'a i podstawowe koncepty

## Slajdy z prezentacji

[Do pobrania tutaj.](https://github.com/aghbit/BIT_ROS/raw/main/lab1/slides.pdf)

## Środowisko

Nowy pusty ROSject na platformie [The Construct](https://www.theconstructsim.com/), z uruchomioną symulacją Turtlebot 3.

## Pojęcia

**Część teoretyczna:**

- [Filozofia Robot Operating System](https://www.oreilly.com/library/view/programming-robots-with/9781449325480/ch01.html)
- [Client libraries](http://wiki.ros.org/Client%20Libraries)
- [ROS 1 vs ROS 2](http://design.ros2.org/articles/changes.html)
- [Gazebo](https://gazebosim.org/home)
- Roboty mobilne
  - [Turtlebot 3](https://www.turtlebot.com/)
  - [ROSbot 2R](https://husarion.com/manuals/rosbot/)

**Część praktyczna:**

- [Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- [Catkin](http://wiki.ros.org/catkin/conceptual_overview)
- [Package](http://wiki.ros.org/Packages)
  - [roscd](http://wiki.ros.org/rosbash#roscd)
- [Node](http://wiki.ros.org/Nodes)
  - [rosnode](http://wiki.ros.org/rosnode)
  - example node

    ```py
      #!/usr/bin/env python3
      import rospy

      rospy.init_node("bit_ros")
      r = rospy.Rate(10) # 10 hz
      while not rospy.is_shutdown():
        # . . .
        r.sleep()
    ```
  - [rospy.Rate(hz) vs rospy.sleep(duration)](http://wiki.ros.org/rospy/Overview/Time)
- [Master](http://wiki.ros.org/Master)
- [Topic](http://wiki.ros.org/Topics)
  - [Message](https://wiki.ros.org/Messages)
  - [rostopic](http://wiki.ros.org/rostopic)
  - example publisher
    ```py
      #!/usr/bin/env python3
      import rospy
      from std_msgs.msg import String

      rospy.init_node("publisher")
      publisher = rospy.Publisher("/hello_world", String, queue_size=10)
      r = rospy.Rate(10)
      while not rospy.is_shutdown():
        publisher.publish("Hello World")
        r.sleep()
    ```
  - example subscriber
    ```py
      #!/usr/bin/env python3
      import rospy
      from std_msgs.msg import String

      def callback(msg):
        print(msg)

      rospy.init_node("subscriber")
      subscriber = rospy.Subscriber("/hello_world", String, callback)
      rospy.spin()
    ```

## Skrypty

W katalogu `/scripts`.
