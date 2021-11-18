#include <sys/fcntl.h>
// #include <unistd.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "accel_cmd");
    ros::NodeHandle n;
    ros::Publisher accel_pub = n.advertise<std_msgs::Float64>("accel_cmd", 100);

    int fd;
    while (ros::ok()) {
        fd = open("/home/tristar/segway/serial_out", O_RDONLY);
        if (fd < 0) {
            std::cout << "./momo.run を実行してください。";
        }
        else {
            std::cout << "momo のシリアルポート /home/tristar/segway/serial_out を発見\n";
            break;
        }
        ros::Duration(1).sleep();
    }

    int req_size = 50;
    int count = 0;
    char buf_ptr[50];

    while (ros::ok()) {
        count++;
        int read_size = 0;
        read_size = read(fd, buf_ptr, req_size);

        if (read_size < 1) {
            continue;
        }
        buf_ptr[read_size] = '\0';
        std::string str = std::string(buf_ptr);
        std::cout << count << " read_size: " << read_size << " msg: " << str << '\n';
        if (read_size == 1 && str.at(0) == 'q') {
            break;
        }
        std_msgs::Float64 msg;
        msg.data = std::stod(str);
        accel_pub.publish(msg);
    }
    close(fd);
    return 0;
}
