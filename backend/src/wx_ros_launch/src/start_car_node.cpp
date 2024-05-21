#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib> // For system()
#include <signal.h> // For signal()

// 全局变量，用于标记是否接收到退出信号
bool received_exit_signal = false;

// 回调函数，处理启动机器人的命令
void startRobotCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received start robot command: [%s]", msg->data.c_str());

    // 启动 husky_gazebo.launch 文件，并在后台启动 RViz
    system("roslaunch husky_gazebo husky_playpen.launch");
    
    // 使用 system 调用直接启动 Python 脚本
    // 确保给出了正确的脚本路径和所需的任何命令行参数
    system("python /home/wzq/my_ws/src/display_tran/scriptsimage_trans.py");
}

// 回调函数，处理退出信号
void exitSignalHandler(int sig)
{
    ROS_INFO("Received exit signal, shutting down...");

    // 标记接收到退出信号
    received_exit_signal = true;

    // 在退出节点时关闭 launch 文件和 RViz
    system("rosnode kill /rviz");
    system("pkill -f 'rosrun rviz'");
    system("pkill -f 'roslaunch husky_gazebo'");
    system("pkill -f '/path/to/display_trans/image_trans.py'");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_robot_node");
    ros::NodeHandle nh;

    // 订阅启动机器人的话题
    ros::Subscriber sub = nh.subscribe("start_robot_topic", 1, startRobotCallback);

    // 注册退出信号处理函数
    signal(SIGINT, exitSignalHandler);

    // 循环等待退出信号或收到退出指令
    while (ros::ok() && !received_exit_signal)
    {
        ros::spinOnce();
    }

    ROS_INFO("Shutting down...");

    return 0;
}

