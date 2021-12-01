#include <sys/fcntl.h>
// #include <unistd.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "segway_rmp/VelocityStatus.h"
#include "segway_rmp/SegwayStatusStamped.h"
#include "segway_rmp/AccelCmd.h"

#include "serialPathConfig.h" // SERIAL_PATH を定義


class A {
public:
    A() {
        n = new ros::NodeHandle("~");
        this->accel_pub = this->n->advertise<segway_rmp::AccelCmd>("accel", 100);
        this->VelocityStatus_sub = this->n->subscribe("/segway_rmp_node/vel", 100, &A::velocity_status_callback, this);
        this->SegwayStatus_sub = this->n->subscribe("/segway_rmp_node/segway_status", 1000, &A::segway_status_callback, this);
    }
    void segway_status_callback(const segway_rmp::SegwayStatusStamped &sss_msg) {
        std::stringstream ss;
        ss << "sgss";
        // ss << "ptich angle: " << sss_msg.segway.pitch_angle << '?';
        // ss << "ptich rate: " << sss_msg.segway.pitch_rate << '?';
        // ss << "roll angle: " << sss_msg.segway.roll_angle << '?';
        // ss << "roll rate: " << sss_msg.segway.roll_rate << '?';
        ss << "左車輪の速度: " << sss_msg.segway.left_wheel_velocity << "(m/s)?";
        ss << "右車輪の速度: " << sss_msg.segway.right_wheel_velocity << "(m/s)?";
        ss << "加速度: " << sss_msg.segway.accel << "(m/s^2)\n";
        // ss << "yaw rate: " << sss_msg.segway.yaw_rate << '?';
        // ss << "servo frames: " << sss_msg.segway.servo_frames << '?';
        // ss << "left wheel displacement: " << sss_msg.segway.left_wheel_displacement << '?';
        // ss << "right wheel velocity: " << sss_msg.segway.right_wheel_displacement << '?';
        // ss << "forward displacement: " << sss_msg.segway.forward_displacement << '?';
        // ss << "yaw displacement: " << sss_msg.segway.yaw_displacement << '?';
        // ss << "left motor torque: " << sss_msg.segway.left_motor_torque << '?';
        // ss << "right motor torque: " << sss_msg.segway.right_motor_torque << '?';
        // ss << "operation mode: " << sss_msg.segway.operation_mode << '?';
        // ss << "gain schedule: " << sss_msg.segway.gain_schedule << '?';
        // ss << "ui battery: " << sss_msg.segway.ui_battery << '?';
        // ss << "powerbase battery: " << sss_msg.segway.powerbase_battery << '?';
        // ss << "motors enabled: " << sss_msg.segway.motors_enabled << '\n';

        std::string str = ss.str();
        char buf_ptr[500];
        for (int i = 0; i < str.length(); i++) {
            buf_ptr[i] = str.at(i);
        }
        buf_ptr[str.length()] = '\0';
        write(this->fd_write, buf_ptr, str.length());
        std::stringstream ss2;
        ss2 << "geve";
        ss2 << std::fixed << std::setprecision(10) << sss_msg.segway.ros_time << ',' << sss_msg.segway.left_wheel_velocity << ',' << sss_msg.segway.right_wheel_velocity << ',' << (sss_msg.segway.left_wheel_velocity + sss_msg.segway.right_wheel_velocity) / 2.0 << '\n';
        std::string str2 = ss2.str();
        char buf_ptr2[100];
        for (int i = 0; i < str2.length(); i++) {
            buf_ptr2[i] = str2.at(i);
        }
        buf_ptr2[str2.length()] = '\0';
        write(this->fd_write, buf_ptr2, str2.length());
    }

    void velocity_status_callback(const segway_rmp::VelocityStatus& vs) {
        // int fd_write = open("/home/ojima/segway/serial_out", O_WRONLY);
        // this->fd_write = open("/home/ojima/segway/serial_out", O_WRONLY);
        // ROS_INFO("write");
        std::stringstream ss;
        ss << "sgvs";
        // ss << "section: " << vs.section << '?';
        // ss << "x: " << vs.x << '?';
        ss << "移動距離: " << vs.x << "(m)?";
        ss << "経過時間: " << vs.t << " (秒)?";
        ss << "所要時間: " << vs.total_time << " (秒)?";
        // ss << "total_time: " << vs.total_time << '?';
        ss << "速度: " << vs.velocity << " (m/s)?";
        ss << "最大速度: " << vs.vm << " (m/s)?";
        ss << "理想加速度: " << vs.am << " (m/s^2)\n";
        // ss << "T1: " << vs.T1 << '?';
        // ss << "T2: " << vs.T2 << '?';
        // ss << "T3: " << vs.T3 << '\n';

        std::string str = ss.str();
        char buf_ptr[255];
        for (int i = 0; i < str.length(); i++) {
            buf_ptr[i] = str.at(i);
        }
        buf_ptr[str.length()] = '\0';
        write(this->fd_write, buf_ptr, str.length());

        std::stringstream ss2;
        ss2 << "seve";
        ss2 << std::fixed << std::setprecision(10) << ros::Time::now() << ',' << vs.velocity << '\n';
        std::string str2 = ss2.str();
        char buf_ptr2[50];
        for (int i = 0; i < str2.length(); i++) {
            buf_ptr2[i] = str2.at(i);
        }
        buf_ptr2[str2.length()] = '\0';
        write(this->fd_write, buf_ptr2, str2.length());
        // close(fd_write);
    }

    void keepAliveCallback(const ros::TimerEvent& e) {
        // int fd_read = open("/home/ojima/segway/serial_out", O_RDONLY);
        // this->fd_read = open("/home/ojima/segway/serial_out", O_RDONLY);
        int req_size = 50;
        int count = 0;
        char buf_ptr[50];

        count++;
        int read_size = 0;
        read_size = read(this->fd_read, buf_ptr, req_size);

        if (read_size < 1) {
            return;
        }
        buf_ptr[read_size] = '\0';
        std::string str = std::string(buf_ptr);
        std::cout << "\nread_size: " << read_size << " message: " << str << '\n';
        if (read_size == 4 && str == "quit") {
            std::cout << "accel_cmd を終了\n";
            close(this->fd_write);
            close(this->fd_read);
            exit(0);
        }
        if (read_size > 4 && str.substr(0, 4) == "acce") {
            segway_rmp::AccelCmd msg;
            str = str.substr(4, str.size());
            int i = str.find('+');
            msg.accel = std::stod(str.substr(0, i));
            msg.x = std::stod(str.substr(i, str.size()));
            this->accel_pub.publish(msg);
        }
        // close(fd_read);
    }

    int run(void) {
        while (ros::ok()) {
            this->fd_read = open(SERIAL_PATH, O_RDONLY); // SERIAL_PATH は serialPathConfig.h.in にて定義されている。
            this->fd_write = open(SERIAL_PATH, O_WRONLY);
            if (this->fd_read < 0 && this->fd_write < 0) {
                std::cout << "./momo.run を実行してください。";
            }
            else {
                std::cout << "momo のシリアルポート /home/tristar/segway/serial_out を発見\n";
                break;
            }
            ros::Duration(3).sleep();
        }
        this->keep_alive_timer = this->n->createTimer(ros::Duration(3), &A::keepAliveCallback, this);
        ros::AsyncSpinner spinner(2);
        spinner.start();
        while (ros::ok()) {
            // int fd_write = open("/home/ojima/segway/serial_out", O_WRONLY);
            // ROS_INFO("hello");
            // char buf_ptr[50] = "hello\n";
            // write(fd_write, buf_ptr, 50);
            // close(fd_write);
            ros::Duration(1).sleep();
        }
        return 0;
    }
private:
    ros::NodeHandle* n;
    ros::Publisher accel_pub;
    ros::Subscriber VelocityStatus_sub, SegwayStatus_sub;
    ros::Timer keep_alive_timer;

    int fd_read, fd_write;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "accel_cmd");
    A a;
    a.run();
    // int fd_write = open("/home/ojima/segway/serial_out", O_WRONLY);
    // char buf_ptr[50] = "ojimayukiya\n";
    // // buf_ptr[str.length()] = '\0';
    // write(fd_write, buf_ptr, 50);
    // close(fd_write);
    return 0;
}
