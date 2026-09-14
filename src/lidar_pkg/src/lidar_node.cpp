/**
 * @file lidar_node.cpp
 * @brief ROS 2 版激光雷达驱动节点 (带OpenCV可视化 + 离群点滤波)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <limits>

// Linux 串口底层头文件
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <arpa/inet.h>
#include <sys/socket.h>

// OpenCV 头文件
#include <opencv2/opencv.hpp>

// --- 配置宏 ---
#define LIDAR_BAUDRATE 150000
#define RETRY_COUNT 3

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar_node"), fd_(-1), is_shutdown_(false), scan_count_(0), last_point_angle_(0.0)
  {
    // 1. 声明并获取参数
    this->declare_parameter<std::string>("transport", "tcp");
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    this->declare_parameter<std::string>("tcp_host", "192.168.4.1");
    this->declare_parameter<int>("tcp_port", 8889);
    this->declare_parameter<std::string>("frame_id", "lidar_link");

    // 滤波相关参数
    this->declare_parameter<bool>("filter.enabled", true);              // 是否开启滤波
    this->declare_parameter<double>("filter.radius", 0.10);             // 搜索半径 (米)，默认10cm
    this->declare_parameter<int>("filter.min_neighbors", 2);            // 半径内最少邻居数 (包含自己)

    this->get_parameter("transport", transport_);
    this->get_parameter("port_name", port_name_);
    this->get_parameter("tcp_host", tcp_host_);
    this->get_parameter("tcp_port", tcp_port_);
    this->get_parameter("frame_id", frame_id_);

    // 获取滤波参数
    this->get_parameter("filter.enabled", filter_enabled_);
    this->get_parameter("filter.radius", filter_radius_);
    this->get_parameter("filter.min_neighbors", filter_min_neighbors_);

    // 2. 初始化发布者 (ROS 2 写法)
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    full_scan_buffer_.reserve(1000);

    // 3. 初始化OpenCV可视化
    initVisualization();

    // 4. 打开雷达数据通道。实车由 ESP32 把 TTL UART 转发为 TCP；
    //    serial 模式保留给原装 USB-TTL 调试。
    bool opened = false;
    if (transport_ == "tcp") {
      opened = openTcp(tcp_host_, tcp_port_);
    } else if (transport_ == "serial") {
      opened = openSerial(port_name_, LIDAR_BAUDRATE);
    } else {
      RCLCPP_FATAL(
        this->get_logger(), "Unknown transport '%s' (use tcp or serial)", transport_.c_str());
    }
    if (!opened) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open LiDAR transport");
      exit(1);
    }

    // 5. 发送启动指令 (A5 60)。原装 USB 转换器上电后的
    // 前一两次命令可能被忽略，与已验证通过的串口探针保持一致：
    // 连续发送 3 次，每次间隔 200 ms。
    for (int attempt = 0; attempt < RETRY_COUNT; ++attempt) {
      sendCmd({0xA5, 0x60});
      if (attempt + 1 < RETRY_COUNT) {
        usleep(200000);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Lidar started via %s", transport_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Filter Config -> Enabled: %s, Rad: %.2fm, Neighbors: %d",
      filter_enabled_ ? "true" : "false", filter_radius_, filter_min_neighbors_);
  }

  ~LidarNode()
  {
    shutdown();
    cv::destroyAllWindows();
  }

  /**
   * @brief 节点主循环，由外部 main 函数调用
   */
  void run_loop()
  {
    uint8_t buffer[1024];
    // 使用 Rate 控制循环频率
    rclcpp::Rate r(500);

    while (rclcpp::ok() && !is_shutdown_) {
      int n = read(fd_, buffer, sizeof(buffer));
      if (n > 0) {
        for (int i = 0; i < n; i++) {
          processByte(buffer[i]);
        }
      } else if (n < 0) {
        if (errno != EAGAIN) {
          RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
        }
      }

      // OpenCV GUI 事件处理
      //cv::waitKey(1);

      // ROS 2 处理回调
      rclcpp::spin_some(this->get_node_base_interface());

      r.sleep();
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::string transport_;
  std::string port_name_;
  std::string tcp_host_;
  int tcp_port_;
  std::string frame_id_;

  // 滤波控制变量
  bool filter_enabled_;
  double filter_radius_;
  int filter_min_neighbors_;

  int fd_;
  bool is_serial_transport_ = false;
  bool is_shutdown_;

  // 扫描圈数计数器
  int scan_count_ = 0;

  // --- 解析相关变量 ---
  enum State
  {
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ_META,
    READ_PAYLOAD
  };

  State state_ = WAIT_HEADER1;
  std::vector<uint8_t> packet_buffer_;
  uint8_t current_lsn_ = 0;
  size_t target_packet_size_ = 0;

  // --- 点云处理相关变量 ---
  struct LidarPoint
  {
    double angle_rad;     // 原代码这里叫 angle_red，建议改为 angle_rad
    double distance_m;
    // 辅助转笛卡尔坐标，用于加速计算
    double x;
    double y;
  };
  std::vector<LidarPoint> full_scan_buffer_;
  double last_point_angle_;
  rclcpp::Time scan_start_time_;
  bool first_packet_of_scan_ = true;

  // --- OpenCV 可视化相关变量 ---
  cv::Mat map_image_;
  const int img_size_ = 800;
  const double max_range_ = 3.0;
  double px_scale_;
  const std::string win_name_ = "Lidar Scan Monitor";

  // ================= 初始化与辅助 =================

  void initVisualization()
  {
    map_image_ = cv::Mat::zeros(img_size_, img_size_, CV_8UC3);
    px_scale_ = (img_size_ / 2.0) / max_range_;
  }

  void resetImage()
  {
    map_image_ = cv::Scalar(0, 0, 0);
    cv::line(
      map_image_, cv::Point(img_size_ / 2, 0), cv::Point(
        img_size_ / 2,
        img_size_), cv::Scalar(
        50, 50,
        50), 1);
    cv::line(
      map_image_, cv::Point(0, img_size_ / 2), cv::Point(
        img_size_,
        img_size_ / 2),
      cv::Scalar(50, 50, 50), 1);

    // 在界面上显示滤波状态
    std::string status = filter_enabled_ ? "Filter: ON" : "Filter: OFF";
    cv::putText(
      map_image_, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
      cv::Scalar(0, 255, 0), 2);
  }

  // ================= 串口底层操作 =================

  bool openSerial(const std::string & port, int baudrate)
  {
    is_serial_transport_ = true;
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "Open Error: %s", strerror(errno));
      return false;
    }

    struct termios2 tio;
    if (ioctl(fd_, TCGETS2, &tio) != 0) {
      close(fd_);
      return false;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;

    if (ioctl(fd_, TCSETS2, &tio) != 0) {
      close(fd_);
      return false;
    }

    ioctl(fd_, TCFLSH, TCIOFLUSH);
    return true;
  }

  bool openTcp(const std::string & host, int port)
  {
    is_serial_transport_ = false;
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "TCP socket error: %s", strerror(errno));
      return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(this->get_logger(), "Invalid LiDAR TCP address: %s", host.c_str());
      close(fd_);
      fd_ = -1;
      return false;
    }
    if (connect(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      RCLCPP_ERROR(
        this->get_logger(), "TCP connection to %s:%d failed: %s",
        host.c_str(), port, strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags == -1 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to set LiDAR TCP socket non-blocking: %s",
        strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Connected to LiDAR relay at %s:%d", host.c_str(), port);
    return true;
  }

  void shutdown()
  {
    if (is_shutdown_) {return;}
    is_shutdown_ = true;

    if (fd_ != -1) {
      const std::vector<uint8_t> stop_cmd = {0xA5, 0x00, 0xA5, 0x65, 0xA5, 0x65};
      // The original USB adapter may drop a command if the descriptor is
      // closed immediately after write().  Repeat it and drain the serial
      // transmitter before closing, matching the reliable start sequence.
      for (int attempt = 0; attempt < RETRY_COUNT; ++attempt) {
        write(fd_, stop_cmd.data(), stop_cmd.size());
        if (is_serial_transport_) {
          ioctl(fd_, TCSBRK, 1);  // wait until queued output has drained
        }
        if (attempt + 1 < RETRY_COUNT) {
          usleep(200000);
        }
      }
      if (is_serial_transport_) {
        ioctl(fd_, TCSBRK, 1);
      }
      close(fd_);
      fd_ = -1;
    }
  }

  void sendCmd(const std::vector<uint8_t> & cmd)
  {
    if (fd_ != -1) {
      write(fd_, cmd.data(), cmd.size());
    }
  }

  // ================= 协议解析状态机 =================

  void processByte(uint8_t byte)
  {
    switch (state_) {
      case WAIT_HEADER1:
        if (byte == 0xAA) {
          state_ = WAIT_HEADER2;
          packet_buffer_.clear();
          packet_buffer_.push_back(byte);
        }
        break;

      case WAIT_HEADER2:
        if (byte == 0x55) {
          state_ = READ_META;
          packet_buffer_.push_back(byte);
        } else if (byte == 0xAA) {
          state_ = WAIT_HEADER2;
          packet_buffer_.clear();
          packet_buffer_.push_back(byte);
        } else {
          state_ = WAIT_HEADER1;
          packet_buffer_.clear();
        }
        break;

      case READ_META:
        packet_buffer_.push_back(byte);
        if (packet_buffer_.size() == 4) {
          current_lsn_ = packet_buffer_[3];
          if (current_lsn_ == 0) {
            state_ = WAIT_HEADER1;
            packet_buffer_.clear();
          } else {
            // PH(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) +
            // CS(2) + [intensity(1) + distance(2)] * LSN.
            target_packet_size_ = 10 + (current_lsn_ * 3);
            state_ = READ_PAYLOAD;
          }
        }
        break;

      case READ_PAYLOAD:
        packet_buffer_.push_back(byte);
        if (packet_buffer_.size() == target_packet_size_) {
          parsePacket(packet_buffer_);
          state_ = WAIT_HEADER1;
          packet_buffer_.clear();
        }
        break;
    }
  }

  // ================= 点云转换算法 =================

  inline uint16_t bytesToUint16(const std::vector<uint8_t> & data, size_t index)
  {
    return (static_cast<uint16_t>(data[index + 1]) << 8) | data[index];
  }

  void parsePacket(const std::vector<uint8_t> & packet_data)
  {
    uint8_t lsn = packet_data[3];
    constexpr size_t kPacketHeaderSize = 10;
    constexpr size_t kSampleSize = 3;
    if (lsn == 0 ||
      packet_data.size() != kPacketHeaderSize + kSampleSize * lsn)
    {
      return;
    }

    uint16_t fsangle_raw = bytesToUint16(packet_data, 4);
    uint16_t lsangle_raw = bytesToUint16(packet_data, 6);
    uint16_t packet_checksum = bytesToUint16(packet_data, 8);

    // YDLIDAR intensity packet checksum:
    // PH ^ FSA ^ each(intensity ^ distance) ^ [LSN|CT] ^ LSA.
    uint16_t calculated_checksum = 0x55AA;
    calculated_checksum ^= fsangle_raw;
    calculated_checksum ^=
      (static_cast<uint16_t>(lsn) << 8) | packet_data[2];
    calculated_checksum ^= lsangle_raw;
    for (size_t i = 0; i < lsn; ++i) {
      const size_t offset = kPacketHeaderSize + i * kSampleSize;
      calculated_checksum ^= packet_data[offset];
      calculated_checksum ^= bytesToUint16(packet_data, offset + 1);
    }

    if (calculated_checksum != packet_checksum) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "LiDAR checksum mismatch: received=0x%04x calculated=0x%04x",
        packet_checksum, calculated_checksum);
      return;
    }

    if (first_packet_of_scan_) {
      scan_start_time_ = this->now();
      first_packet_of_scan_ = false;
    }

    double angle_start_deg = static_cast<double>(fsangle_raw >> 1) / 64.0;
    double angle_end_deg = static_cast<double>(lsangle_raw >> 1) / 64.0;

    double diff_angle_deg = 0.0;
    if (lsn > 1) {
      diff_angle_deg = angle_end_deg - angle_start_deg;
      if (diff_angle_deg < 0) {
        diff_angle_deg += 360.0;
      }
    }

    for (int i = 0; i < lsn; ++i) {
      size_t offset = kPacketHeaderSize + i * kSampleSize;
      if (offset + 2 >= packet_data.size()) {break;}

      // intensity 位于 offset；当前 LaserScan 只使用后两字节距离。
      uint16_t dist_raw = bytesToUint16(packet_data, offset + 1);

      double distance_m = static_cast<double>(dist_raw) / 4.0 / 1000.0;
      double distance_mm = static_cast<double>(dist_raw) / 4.0;

      double angle_deg = angle_start_deg;
      if (lsn > 1) {
        angle_deg = (diff_angle_deg / (lsn - 1)) * i + angle_start_deg;
      }

      double angle_correct_deg = 0.0;
      if (distance_mm != 0) {
        double numerator = 21.8 * (155.3 - distance_mm);
        double denominator = 155.3 * distance_mm;
        double angle_correct_rad = std::atan(numerator / denominator);
        angle_correct_deg = angle_correct_rad * 180.0 / M_PI;
      }

      double final_angle_deg = angle_deg + angle_correct_deg;
      final_angle_deg = std::fmod(final_angle_deg, 360.0);
      if (final_angle_deg < 0) {final_angle_deg += 360.0;}

      double angle_rad = M_PI * final_angle_deg / 180.0;

      if (distance_m > 0.01) {
        // 注意：这里OpenCV画的是原始数据（包含噪点），因为滤波是整圈完成后才进行的。
        // 如果想看滤波后的图，需要在 publishScan 里重画，或者在这里暂存。
        if (distance_m <= max_range_) {
          int center_xy = img_size_ / 2;
          int px = center_xy + static_cast<int>(distance_m * px_scale_ * std::sin(angle_rad));
          int py = center_xy - static_cast<int>(distance_m * px_scale_ * std::cos(angle_rad));

          if (px >= 0 && px < img_size_ && py >= 0 && py < img_size_) {
            cv::circle(map_image_, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
          }
        }

        // 检测过零 (一圈结束)
        if (angle_rad < last_point_angle_ - M_PI) {
          if (scan_count_ > 0) {
            // 在发布前，调用发布函数，发布函数内部会进行滤波
            publishScan();
            //cv::imshow(win_name_, map_image_);
          } else {
            RCLCPP_INFO(this->get_logger(), "Skipping first partial scan...");
          }

          scan_count_++;
          resetImage();
          full_scan_buffer_.clear();
          first_packet_of_scan_ = true;
          scan_start_time_ = this->now();
        }

        LidarPoint p;
        p.angle_rad = angle_rad;
        p.distance_m = distance_m;
        // 预先计算坐标，加速后续滤波
        p.x = distance_m * std::cos(angle_rad);
        p.y = distance_m * std::sin(angle_rad);

        full_scan_buffer_.push_back(p);
        last_point_angle_ = angle_rad;
      }
    }
  }

  /**
   * @brief 半径滤波算法
   * 遍历所有点，如果某点在 radius 范围内的邻居数量少于 min_neighbors，则视为噪点剔除
   */
  void removeOutliers(std::vector<LidarPoint> & points)
  {
    if (points.empty()) {return;}

    std::vector<LidarPoint> clean_points;
    clean_points.reserve(points.size());

    double r2 = filter_radius_ * filter_radius_;     // 比较距离的平方，避免开根号

    // 由于一圈只有几百个点，双重循环暴力匹配也是很快的 (400*400 = 16万次操作，对CPU可忽略)
    for (size_t i = 0; i < points.size(); ++i) {
      int neighbors = 0;
      const auto & p1 = points[i];

      for (size_t j = 0; j < points.size(); ++j) {
        // 距离平方计算
        double dx = p1.x - points[j].x;
        double dy = p1.y - points[j].y;
        if ((dx * dx + dy * dy) < r2) {
          neighbors++;
        }
        // 如果有足够的邻居，直接通过，不需要数完
        if (neighbors >= filter_min_neighbors_) {break;}
      }

      // 如果邻居足够（注意：neighbors包含点自己，所以至少是1），则保留
      if (neighbors >= filter_min_neighbors_) {
        clean_points.push_back(p1);
      }
    }

    // 替换原始数据
    points = std::move(clean_points);
  }

  void publishScan()
  {
    if (full_scan_buffer_.empty()) {return;}

    // 在转换为 ROS 消息前，先执行滤波
    if (filter_enabled_) {
      removeOutliers(full_scan_buffer_);
      // RCLCPP_INFO(this->get_logger(), "Filtered: %lu -> %lu", raw_size, full_scan_buffer_.size());
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = (scan_start_time_.nanoseconds() == 0) ? this->now() : scan_start_time_;
    scan.header.frame_id = frame_id_;

    scan.angle_min = 0.0;
    const int SCAN_SIZE = 720;
    scan.angle_increment = (2.0 * M_PI) / SCAN_SIZE;
    // angle_max 对应 ranges.back()：720 个点的最后一个角度是 359.5°。
    scan.angle_max = scan.angle_min +
      (SCAN_SIZE - 1) * scan.angle_increment;

    scan.range_min = 0.05;
    scan.range_max = 8.0;

    scan.ranges.assign(SCAN_SIZE, std::numeric_limits<float>::infinity());

    for (const auto & point : full_scan_buffer_) {
      double raw_angle = point.angle_rad;
      // ROS LaserScan 逆时针为正，但部分雷达输出顺时针，这里保持你原有的逻辑
      double index_angle = 2.0 * M_PI - raw_angle;

      if (index_angle >= 2.0 * M_PI) {index_angle -= 2.0 * M_PI;}
      if (index_angle < 0) {index_angle += 2.0 * M_PI;}

      int index = static_cast<int>(index_angle / scan.angle_increment);

      if (index >= 0 && index < SCAN_SIZE) {
        if (scan.ranges[index] == std::numeric_limits<float>::infinity() ||
          point.distance_m < scan.ranges[index])
        {
          scan.ranges[index] = static_cast<float>(point.distance_m);
        }
      }
    }

    const double measured_scan_time =
      (this->now() - scan.header.stamp).seconds();
    scan.scan_time = measured_scan_time > 0.0 ? measured_scan_time : 1.0 / 7.0;
    scan.time_increment = scan.scan_time / SCAN_SIZE;

    scan_pub_->publish(scan);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarNode>();
  try {
    node->run_loop();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
