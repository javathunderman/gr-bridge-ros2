#ifndef GR_PUBLISHER_HPP
#define GR_PUBLISHER_HPP
#define LOCAL_IP "127.0.0.1"
#define LOCAL_PORT 5000
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
class MinimalPublisher : public rclcpp::Node {
    public:
        MinimalPublisher();
        virtual ~MinimalPublisher();
        void msg_callback(void);
        void connect_socket(void);
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        int doa_socket;
        int count_;
        int recv_bytes;
        char *recv_buf;
        struct sockaddr_in *server_addr;
};
#endif