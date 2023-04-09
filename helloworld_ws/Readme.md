
[link](https://blog.csdn.net/qq_44732054/article/details/122719315?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~Rate-1-122719315-blog-126857610.235%5Ev28%5Epc_relevant_default_base1&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~Rate-1-122719315-blog-126857610.235%5Ev28%5Epc_relevant_default_base1&utm_relevant_index=2)


Python实现Hello world
在ROS中编写Python程序与编写C++程序大同小异。

step1.创建工作空间
Ctrl+Alt+T打开终端分别输入：【在主目录下创建工作空间】
```
mkdir -p helloworld_ws/src
cd helloworld_ws
catkin_make
```
其中helloworld_ws为我们命名的工作空间名称，catkin_make是一个编译命令，该编译命令会帮我们创建好我们所需的文件并输出若干日志，成功结果如下：



step2.创建功能包
继续在该终端输入：【在工作空间的src文件中创建功能包】
```
cd src
catkin_create_pkg helloworld roscpp rospy std_msgs
```
其中helloworld为我们的功能包名称，roscpp 、rospy和std_msgs为添加的依赖，这些依赖是我们在ROS中运行程序最基本的一些依赖，成功结果如下：【低版本的ROS中message依赖可能为ros_msgs】



step3.编辑源文件
 在功能包中新建名称为【scripts】的文件夹与【src】文件夹同级



在终端中输入：【创建源文件】
```
cd ~/helloworld_ws/src/helloworld/scripts
gedit  talker.py
```
在空白文档中输入：【Hello world!】

```
#!/home/lg/anaconda3/bin/python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

```
gedit listener.py
```

```
#!/home/lg/anaconda3/bin/python


import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

step4.编辑配置文件
打开helloworld功能包中的CMakeLists.txt【helloworld_ws/src/helloword/CMakeLists.txt】

```
catkin_install_python(PROGRAMS
   scripts/talker.py
   scripts/listener.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
```

step5.编译并执行
新打开一个输入：【启动ros核心】
```
roscore
```

重新打开一个终端输入：【在工作空间中编译并执行】
```
cd ~/helloworld_ws
catkin_make
source ./devel/setup.bash
rosrun helloworld talker.py
```

再重新打开一个终端输入：【在工作空间中编译并执行】
```
cd ~/helloworld_ws
catkin_make
source ./devel/setup.bash
rosrun helloworld listener.py
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/1f67463662c14038a6e6b509847be15b.png#pic_center)


