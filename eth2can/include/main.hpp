#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdio.h>
#include <cstdint>
#include "can_msgs/msg/frame.hpp"

int client_socket;