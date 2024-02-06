#include "main.hpp"

void sendAndPrintMessage(const char *message)
{
  sleep(1);

  std::string message_with_newline = message;
  message_with_newline += "\r\n";

  ssize_t sent_bytes = send(client_socket, message_with_newline.c_str(), message_with_newline.length(), 0);
  auto logger = rclcpp::get_logger("eth2can_node");

  if (sent_bytes == -1)
  {
    RCLCPP_ERROR(logger, "Message transmission failed");
  }
  else if (sent_bytes == static_cast<ssize_t>(message_with_newline.length()))
  {
    RCLCPP_INFO(logger, "Message successfully sent to the MW-Ethernet2CAN : %s", message);
  }
  else
  {
    RCLCPP_INFO(logger, "Partial message sent (sent bytes: %d)", sent_bytes);
  }

  sleep(1);
}

char Checksum(char *data, int len)
{
    char cs = 0;
    for (int i = 0; i < len; i++)
    {
        cs += data[i];
    }
    return cs;
}

int SendPacket(long ID, int length, char data[8], int Ext, int RTR)
{
  unsigned char buff[18]={0,};

  buff[0] = 0x02; //STX
  buff[1] = 0x00;
  buff[2] = static_cast<unsigned char>(length);
  buff[3] = 0x00;

  buff[3] |= (RTR == 1) ? 0x20 : 0x00;
  buff[3] |= (Ext == 1) ? 0x40 : 0x00;

  buff[4] = static_cast<unsigned char>(ID);
  buff[5] = static_cast<unsigned char>(ID >> 8);
  buff[6] = static_cast<unsigned char>(ID >> 16);
  buff[7] = static_cast<unsigned char>(ID >> 24);

  memcpy(&buff[8], &data[0], length);

  buff[16] = Checksum((char *)&buff[1], 15);

  buff[17] = 0x03;  //ETX

  int ret = send(client_socket, buff, sizeof(buff), 0);

  return ret;
}

int RecvPacket(long *ID, int *length, char data[8], int *Ext, int *RTR)
{
    char buffer[1024];

    int bytes = recv(client_socket, buffer, sizeof(buffer), 0);

    if (bytes <= 0 || buffer[0] != 0x02 || buffer[17] != 0x03 || buffer[1] != 0x00 || buffer[16] != Checksum(&buffer[1], 15))
    {
        return -1; 
    }

    long CAN_ID = buffer[4] | (buffer[5] << 8) | (buffer[6] << 16) | (buffer[7] << 24);

    *ID = CAN_ID;
    *length = buffer[2];

    *Ext = (buffer[3] & 0x40) ? true : false;
    *RTR = (buffer[3] & 0x20) ? true : false;

    memcpy(data, &buffer[8], 8);

    return 1;
}

void can_send_Callback(const can_msgs::msg::Frame::SharedPtr msg) /* When a CAN message is received through ROS2 TOPIC, it is sent. */
{
  char buff[8];

  memcpy(buff, &msg->data[0], msg->dlc);

  SendPacket(msg->id, msg->dlc, buff, msg->extended, msg->rtr);
}

void receive_and_publish_thread(rclcpp::Node::SharedPtr node, rclcpp::QoS qos, rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher)
{
  long rid;
  int rlen, ext, rtr;
  char rdata[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  can_msgs::msg::Frame can_message;

  while (rclcpp::ok())
  {
    if (RecvPacket(&rid, &rlen, rdata, &ext, &rtr))
    {
      can_message.header.stamp = node->now();
      can_message.id = rid;
      can_message.extended = ext ? true : false;
      can_message.dlc = rlen;
      can_message.rtr = rtr ? true : false;

      for (int i = 0; i < 8; i++)
      {
        can_message.data[i] = (int)(unsigned char)rdata[i];
      }

      publisher->publish(can_message);
    }
  }
}

int main(int argc, char **argv)
{
  std::string ip;
  int port, bitrate;
  char message[100];

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("eth2can_node", options);

  rclcpp::QoS qos(rclcpp::KeepLast(10));  
  qos.reliable();                        
  qos.durability_volatile();             

  auto subscriber = node->create_subscription<can_msgs::msg::Frame>("eth2can_send", qos, can_send_Callback);
  auto publisher = node->create_publisher<can_msgs::msg::Frame>("/eth2can_recv", qos);
  auto frame_msg = std::make_shared<can_msgs::msg::Frame>();
  auto logger = node->get_logger();

  node->declare_parameter<std::string>("ip", "192.168.0.10");
  node->get_parameter("ip", ip);

  node->declare_parameter<int>("port", 3000);
  node->get_parameter("port", port);

  node->declare_parameter<int>("bitrate", 1000);
  node->get_parameter("bitrate", bitrate);

  RCLCPP_INFO(logger, "Information for connecting to MW Ethernet2CAN");
  RCLCPP_INFO(logger, "IP : %s ", ip.c_str());
  RCLCPP_INFO(logger, "PORT : %d ", port);
  RCLCPP_INFO(logger, "CAN Bitrate : %d ", bitrate);

  client_socket = socket(AF_INET, SOCK_STREAM, 0);

  if (client_socket == -1)
  {
    RCLCPP_ERROR(logger, "Socket Creation Error");
    return -1;
  }

  struct sockaddr_in server_address;
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(port);
  inet_pton(AF_INET, ip.c_str(), &server_address.sin_addr);

  if (connect(client_socket, (struct sockaddr *)&server_address, sizeof(server_address)) == -1)
  {
    RCLCPP_ERROR(logger, "MW-Ethernet2CAN Connection Error");
    close(client_socket);
    return -1;
  }
  else
  {
    RCLCPP_INFO(logger, "MW-Ethernet2CAN Connection Successful");
  }
  
  std::sprintf(message, "B=%d", bitrate); // 비트레이트 설정
  sendAndPrintMessage(message);

  sendAndPrintMessage("P"); // 비트레이트 변경후 P 명령어 적용해야 한다.

  std::sprintf(message, "T=%d", 1);       // 전송방식 설정
  sendAndPrintMessage(message);

  RCLCPP_INFO(logger, "MW-Ethernet2CAN is operational");

  rclcpp::Rate rate(10000); // 0.1ms

  auto pub_thread = std::thread(receive_and_publish_thread, node, qos, publisher);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  } 

  pub_thread.join(); 
  
  rclcpp::shutdown();
  return 0;
}