[代码在git](https://github.com/luogantt/ros_project/tree/master/helloworld_ws_cpp)
[link](https://blog.csdn.net/qq_44732054/article/details/122719315?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~Rate-1-122719315-blog-126857610.235%5Ev28%5Epc_relevant_default_base1&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~Rate-1-122719315-blog-126857610.235%5Ev28%5Epc_relevant_default_base1&utm_relevant_index=2)
Hello world实现流程
无论是C++还是Python其他编程语言，实现流程都是大同小异：

创建工作空间 → 创建功能包 → 编辑源文件 → 编辑配置文件 → 编译并执行

#### step1.创建工作空间
#### Ctrl+Alt+T打开终端分别输入：【在主目录下创建工作空间】
```
mkdir -p helloworld_ws/src
cd helloworld_ws
catkin_make
```
其中helloworld_ws为我们命名的工作空间名称，catkin_make是一个编译命令，该编译命令会帮我们创建好我们所需的文件并输出若干日志，成功结果如下：



#### step2.创建功能包
#### 继续在该终端输入：【在工作空间的src文件中创建功能包】
```
cd src
catkin_create_pkg helloworld roscpp rospy std_msgs
```
其中helloworld为我们的功能包名称，roscpp 、rospy和std_msgs为添加的依赖，这些依赖是我们在ROS中运行程序最基本的一些依赖，成功结果如下：【低版本的ROS中message依赖可能为ros_msgs】



#### step3.编辑源文件
继续在终端输入：【在功能包中创建源文件】
```
cd helloworld/src
gedit Hello_pub.cpp
```
在空白文档中输入：【Hello world!】
```
/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)

         PS: 二者需要设置相同的话题


    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号

    实现流程:
        1.包含头文件 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 发布者 对象
        5.组织被发布的数据，并编写逻辑发布数据

*/
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>

int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std_msgs::String msg;
    // msg.data = "你好啊！！！";
    std::string msg_front = "Hello 你好！"; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(1);

    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //暂无应用
        ros::spinOnce();
    }


    return 0;
}

```

```
cd helloworld/src
gedit Hello_sub.cpp
```




```
/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)


    消息订阅方:
        订阅话题并打印接收到的消息

    实现流程:
        1.包含头文件 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 订阅者 对象
        5.处理订阅的消息(回调函数)
        6.设置循环调用回调函数

*/
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h"

void doMsg(const std_msgs::String::ConstPtr& msg_p){
    ROS_INFO("我听见:%s",msg_p->data.c_str());
    // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"listener");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

    //4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter",10,doMsg);
    //5.处理订阅的消息(回调函数)

    //     6.设置循环调用回调函数
    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}

```

##### step4.编辑配置文件
打开helloworld功能包中的CMakeLists.txt【helloworld_ws/src/helloword/CMakeLists.txt】

```
add_executable(Hello_pub
  src/Hello_pub.cpp
)
add_executable(Hello_sub
  src/Hello_sub.cpp
)

target_link_libraries(Hello_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(Hello_sub
  ${catkin_LIBRARIES}
)
```





#### step5.编译并执行
新打开一个输入：【启动ros核心】
```
roscore
```
重新打开另一个终端输入：【在工作空间中编译并执行】
```
cd ~/helloworld_ws
catkin_make
source ./devel/setup.bash
rosrun helloworld Hello_pub 
```
其中，source命令为修改环境变量，helloworld为功能包名称，hello为映射变量名称，成功结果如下：


再重新打开另一个终端输入：【在工作空间中编译并执行】
```
cd ~/helloworld_ws
catkin_make
source ./devel/setup.bash
rosrun helloworld Hello_sub 
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/e3d30755497945e79e9b1259305c355b.png#pic_center)

