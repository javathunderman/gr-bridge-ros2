// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "gr_publisher.hpp"

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0)
{
  this->publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  this->timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::msg_callback, this));
  this->server_addr = (struct sockaddr_in *) malloc(sizeof(struct sockaddr_in));
  this->recv_buf = (char *) malloc(sizeof(char) * 40000);
  this->connect_socket();
}

MinimalPublisher::~MinimalPublisher()
{
  RCLCPP_INFO(this->get_logger(), "Running destructor");
  free(this->server_addr);
  free(this->recv_buf);
  close(this->doa_socket);
}

void MinimalPublisher::msg_callback(void)
{
  this->recv_bytes = recvfrom(this->doa_socket, this->recv_buf, (sizeof(char) * 40000), 0, NULL, NULL);
  if (this->recv_bytes > 0) {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
}

void MinimalPublisher::connect_socket(void) {
  memset(this->server_addr, 0, sizeof(struct sockaddr_in));     /* Zero out structure */
  this->server_addr->sin_family      = AF_INET;             /* Internet address family */
  this->server_addr->sin_addr.s_addr = inet_addr(LOCAL_IP);   /* Server IP address */
  this->server_addr->sin_port        = htons(LOCAL_PORT); /* Server port */
  this->doa_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->doa_socket < 0) {
      RCLCPP_INFO(this->get_logger(), "Failed to create socket");
  }
  if (bind(this->doa_socket, (struct sockaddr *) this->server_addr, sizeof(struct sockaddr_in)) < 0) {
      RCLCPP_INFO(this->get_logger(), "Failed to bind to socket");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
