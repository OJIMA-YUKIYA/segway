#include <sys/fcntl.h>
// #include <unistd.h>
#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "segway_rmp/VelocityStatus.h"

#include "serialPathConfig.h" // SERIAL_PATH を定義


class A {
public:
    A() {
        n = new ros::NodeHandle("~");
        this->accel_pub = this->n->advertise<std_msgs::Float64>("accel", 100);
        this->VelocityStatus_sub = this->n->subscribe("velocity_status", 100, &A::velocity_status_callback, this);
    }
    void velocity_status_callback(const segway_rmp::VelocityStatus& vs) {
        // int fd_write = open("/home/ojima/segway/serial_out", O_WRONLY);
        // this->fd_write = open("/home/ojima/segway/serial_out", O_WRONLY);
        ROS_INFO("write");
        std::stringstream ss;
        ss << "sgvs";
        // ss << "section: " << vs.section << '?';
        // ss << "x: " << vs.x << '?';
        // ss << "t: " << vs.t << '?';
        // ss << "total_time: " << vs.total_time << '?';
        ss << "velocity: " << vs.velocity << '\n';
        // ss << "max velocity: " << vs.vm << '?';
        // ss << "max accel: " << vs.am << '?';
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
        std::cout << "\nread_size: " << read_size << " read_message: " << str << '\n';
        if (read_size == 1 && str.at(0) == 'q') {
            std::cout << "accel_cmd を終了\n";
            close(this->fd_write);
            close(this->fd_read);
            exit(1);
        }
        std_msgs::Float64 msg;
        msg.data = std::stod(str);
        accel_pub.publish(msg);
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
    ros::Subscriber VelocityStatus_sub;
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
