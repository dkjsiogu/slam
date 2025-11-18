
# 全向轮机器人 SLAM 导航系统技术报告

## 1. 项目总体介绍

本项目是基于 ROS2 Humble 的一套完整的全向轮移动机器人 SLAM 与自主导航系统。系统集成了激光雷达（RPLIDAR A1）、IMU 和轮式里程计等多传感器数据，使用 **Google Cartographer** 进行实时 2D SLAM 建图与定位，并在此基础上实现了包含颜色识别、框识别、A* 路径规划、路径跟踪、任务管理在内的全套自主导航功能。

**技术栈:**
- **ROS 版本:** ROS2 Humble Hawksbill
- **SLAM 方案:** Google Cartographer (实时建图模式)
- **硬件平台:** 全向轮底盘 (342mm × 300mm)
- **传感器:** 
  - SLAMTEC RPLIDAR A1 激光雷达 (8Hz, 12m)
  - IMU (3轴陀螺仪 + 加速度计)
  - 轮式编码器 (全向轮速度反馈)
  - USB 摄像头 (目标识别)
- **下位机:** STM32 (115200 波特率, 自定义二进制协议)
- **编程语言:** C++ (底层驱动/里程计), Python (导航规划/任务调度)
- **核心功能:** 
  - 多传感器融合 (Odom + Laser + IMU)
  - 实时 SLAM 建图与定位 (Cartographer)
  - 固定起始点导航 (0,0)
  - 颜色跟踪与篮筐检测 (视觉任务)
  - A* 路径规划 (矩形 footprint 避障)
  - 航点导航与任务调度

系统遵循模块化设计，各功能节点职责分明，通过 ROS2 的 Topic 和 Service 进行高效通信。下面将按照机器人自动化工作流程的顺序，详细介绍各模块的功能与核心代码。

---

## 2. 串口通信 - 与下位机数据交互

**模块:** `serial_communication` (`navigation_control/src/serial_communication.cpp`)

**功能:**
该节点负责通过 LibSerial 库与 STM32 下位机进行实际串口通信。它以 115200 波特率连接 `/dev/ttyACM0`（Micro USB），接收包含底盘速度、IMU 姿态、游戏状态的二进制数据包（50字节/包），并原封不动地发布到 `serial_rx_data` 话题供轮式里程计解析。同时订阅 `serial_tx_data` 以向下位机发送控制指令。节点包含自动重连机制，确保通信鲁棒性。

**核心代码:**
```cpp
// 打开串口并配置参数 (8N1 无流控)
bool SerialCommunication::openSerialPort()
{
    serial_port_obj_.Open(serial_port_);  // /dev/ttyACM0
    serial_port_obj_.SetBaudRate(BaudRate::BAUD_115200);
    serial_port_obj_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port_obj_.SetParity(Parity::PARITY_NONE);
    serial_port_obj_.SetStopBits(StopBits::STOP_BITS_1);
    serial_port_obj_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    
    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ 115200", serial_port_.c_str());
    return true;
}

// 读取串口数据（10ms定时器）
void readSerialData() {
    if (!is_connected_ || !serial_port_obj_.IsOpen()) return;
    
    try {
        if (serial_port_obj_.IsDataAvailable()) {
            std_msgs::msg::UInt8MultiArray raw_data;
            size_t available = serial_port_obj_.GetNumberOfBytesAvailable();
            raw_data.data.resize(available);
            serial_port_obj_.Read(raw_data.data, available, timeout_ms_);
            
            // 发布原始字节流给里程计节点解析
            serial_rx_pub_->publish(raw_data);
            bytes_received_ += available;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
        is_connected_ = false;  // 触发自动重连
    }
}
```

---

## 3. 运动控制指令下发

**模块:** `serial_data_publisher` (`navigation_control/src/serial_data_publisher.cpp`)

**功能:**
全向轮底盘控制节点。订阅 `/cmd_vel` 速度指令 (vx, vy, wz) 和颜色跟踪结果 `/color_tracking/result`，将其打包成 31 字节的 `OmniWheelCmd` 二进制协议（包含 CRC16 校验），以 50Hz 频率持续发送给下位机。下位机按照自定义协议解析后控制全向轮电机。节点支持速度平滑、超时保护、速度限幅等安全机制。

**核心代码:**
```cpp
// 全向轮控制协议结构体 (31字节)
struct __attribute__((packed)) OmniWheelCmd {
    uint8_t header;        // 0xA5 帧头
    float x;               // dx - 目标偏转角度 (rad, 用于视觉跟踪)
    float y;               // dy - 目标垂直偏移 (pixels)
    float z;               // flog - 跟踪有效标志
    float vx;              // 前后速度 (m/s)
    float vy;              // 左右速度 (m/s)
    float vz;              // 旋转速度 (rad/s)
    uint32_t cap_timestamp; // 时间戳
    uint16_t checksum;     // CRC16校验
    
    void calculateChecksum() {
        checksum = Get_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t*>(this), 
            sizeof(OmniWheelCmd) - 2,  // 除去CRC本身
            CRC16_INIT
        );
    }
};

// /cmd_vel 订阅回调 - 限速并平滑处理
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // 限速保护
    cmd_vx_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    cmd_vy_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    cmd_wz_ = std::clamp(msg->angular.z, -max_wz_, max_wz_);
    
    last_cmd_time_ = this->now();  // 更新时间戳用于超时检测
}

// 定时发送器 (50Hz) - 持续向下位机发送控制包
void sendTimerCallback() {
    OmniWheelCmd packet;
    packet.header = 0xA5;
    
    // 速度平滑：指数加权平均
    current_vx_ = current_vx_ * (1.0 - smooth_factor_) + cmd_vx_ * smooth_factor_;
    current_vy_ = current_vy_ * (1.0 - smooth_factor_) + cmd_vy_ * smooth_factor_;
    current_wz_ = current_wz_ * (1.0 - smooth_factor_) + cmd_wz_ * smooth_factor_;
    
    packet.vx = current_vx_;
    packet.vy = current_vy_;
    packet.vz = current_wz_;
    
    // 视觉跟踪数据
    packet.x = tracking_rad_;
    packet.y = tracking_dy_;
    packet.z = tracking_status_;  // 0=无效, 1=彩色, 2=黑色
    
    packet.cap_timestamp = static_cast<uint32_t>(
        this->now().nanoseconds() / 1000000);  // 转毫秒
    
    packet.calculateChecksum();
    
    // 转字节数组并发布到串口
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.resize(sizeof(packet));
    memcpy(msg.data.data(), &packet, sizeof(packet));
    serial_data_pub_->publish(msg);
}
```

---

## 4. 轮式里程计 - 多传感器融合定位

**模块:** `wheel_odometry_node` (`navigation_control/src/wheel_odometry_node.cpp`)

**功能:**
导航系统的核心定位节点。解析下位机发送的 50 字节数据包（包含底盘速度 `chassis_vx/vy/w` 和 IMU 姿态 `roll/pitch/yaw`），使用 ROS 时间戳对速度进行实时积分，并融合 IMU 角度信息，计算出高精度位姿。发布标准 `/odom` 消息和 `odom→base_link` TF，为 SLAM 提供运动先验。支持 **SLAM 校正**、IMU 漂移补偿、静止检测等高级功能。

**数据包结构 (50字节):**
```cpp
struct __attribute__((packed)) VisionSendPacket {
    uint8_t header;              // 0x5A 帧头
    uint8_t detect_color : 1;    // 0-red 1-blue
    uint8_t task_mode : 2;       // 0-auto 1-aim 2-buff
    uint8_t reset_tracker : 1;
    uint8_t is_play : 1;
    uint8_t change_target : 1;
    uint8_t reserved_bits : 2;
    float roll;                  // IMU横滚角 (rad)
    float pitch;                 // IMU俯仰角 (rad)
    float yaw;                   // IMU航向角 (rad) - 关键！
    float delta_theta;           // 遗留字段
    float disp_x;                // 遗留字段
    float disp_y;                // 遗留字段
    float heading_diff;          // 遗留字段
    float chassis_vx;            // ⚠️ 底盘实时速度X (m/s)
    float chassis_vy;            // ⚠️ 底盘实时速度Y (m/s)
    float chassis_w;             // ⚠️ 底盘角速度 (rad/s)
    uint16_t game_time;
    uint32_t timestamp;          // 板载时间戳 (不使用)
    uint16_t checksum;           // CRC16校验
    // 总计: 50字节
};
```

**核心代码 - 速度积分与 IMU 融合:**
```cpp
void WheelOdometryNode::processOdometryDelta(const VisionSendPacket& packet)
{
    // 1. 计算时间增量 (使用ROS时间，而非下位机时间戳)
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    
    // 2. dt 保护 (防止丢包或异常)
    const double MIN_DT = 0.0001;  // 0.1ms
    const double MAX_DT = 0.5;     // 500ms
    if (dt < MIN_DT || dt > MAX_DT) {
        RCLCPP_WARN(this->get_logger(), "异常dt: %.4fs，跳过本帧", dt);
        return;
    }
    
    // 3. 读取机器人坐标系速度
    float vx_robot = packet.chassis_vx;  // 前进速度 (m/s)
    float vy_robot = packet.chassis_vy;  // 左侧速度 (m/s)
    float wz = packet.chassis_w;         // 角速度 (rad/s)
    
    // 4. 读取 IMU yaw 角（用于后面更新角度）
    double imu_yaw = packet.yaw;  // IMU航向角 (rad)
    
    // 4.1 应用 IMU 漂移补偿（基于运行时间累积）
    double elapsed_time = (current_time - imu_compensation_start_time_).seconds();
    double drift_compensation = elapsed_time * imu_drift_compensation_rate_;  // 累计补偿量
    imu_yaw += drift_compensation;  // 应用补偿
    
    // 5. 速度死区过滤（过滤编码器噪声）
    const double VEL_THRESHOLD = 0.001;    // 1mm/s
    const double ANGULAR_THRESHOLD = 0.001; // ~0.06°/s
    if (std::abs(vx_robot) < VEL_THRESHOLD) vx_robot = 0.0f;
    if (std::abs(vy_robot) < VEL_THRESHOLD) vy_robot = 0.0f;
    if (std::abs(wz) < ANGULAR_THRESHOLD) wz = 0.0f;
    
    // 6. 静止检测（综合运动大小）
    double translation_speed = std::sqrt(vx_robot*vx_robot + vy_robot*vy_robot);
    double rotation_speed = std::abs(wz) * 0.15;  // 乘特征半径转为线速度等效
    double total_motion_speed = translation_speed + rotation_speed;
    
    const double MOTION_THRESHOLD = 0.002; // 2mm/s 综合运动阈值
    
    if (total_motion_speed < MOTION_THRESHOLD) {
        // 静止：速度指数衰减，冻结 IMU 更新
        current_vx_ *= 0.7;
        current_vy_ *= 0.7;
        current_wz_ *= 0.7;
        last_imu_yaw_ = imu_yaw;  // 更新基准但不累积
        last_imu_yaw_valid_ = true;
        publishOdometry();
        if (publish_tf_) publishTransform();
        return;
    }
    
    // 7. 机器人坐标系速度 -> 位移增量
    double dx_robot = vx_robot * dt;
    double dy_robot = vy_robot * dt;
    
    // 8. 转换到世界坐标系（odom frame）
    double cos_theta = std::cos(current_theta_);
    double sin_theta = std::sin(current_theta_);
    double dx_world = dx_robot * cos_theta - dy_robot * sin_theta;
    double dy_world = dx_robot * sin_theta + dy_robot * cos_theta;
    
    // 9. 累加位姿
    current_x_ += dx_world;
    current_y_ += dy_world;
    
    // ⚠️ 使用 IMU Yaw 的变化量来更新累积角度（而非编码器角速度积分）
    double delta_yaw_for_pose = 0.0;
    if (last_imu_yaw_valid_) {
        delta_yaw_for_pose = imu_yaw - last_imu_yaw_;
        
        // 角度归一化到 [-π, π]
        while (delta_yaw_for_pose > M_PI) delta_yaw_for_pose -= 2.0 * M_PI;
        while (delta_yaw_for_pose < -M_PI) delta_yaw_for_pose += 2.0 * M_PI;
        
        current_theta_ += delta_yaw_for_pose;
        current_theta_ = std::atan2(std::sin(current_theta_), std::cos(current_theta_));
    }
    last_imu_yaw_ = imu_yaw;
    last_imu_yaw_valid_ = true;
    
    // 10. 更新速度
    current_vx_robot_ = vx_robot;
    current_vy_robot_ = vy_robot;
    current_wz_robot_ = wz;
    
    // 11. 发布里程计消息和TF
    publishOdometry();
    if (publish_tf_) publishTransform();
}
```

**SLAM 校正功能 (关键！):**
```cpp
void WheelOdometryNode::correctFromSlam()
{
    if (!enable_slam_correction_) return;
    
    try {
        // 获取 map -> base_link 的变换（SLAM校正后的真实位姿）
        geometry_msgs::msg::TransformStamped map_to_base;
        map_to_base = tf_buffer_->lookupTransform(
            "map", base_frame_, tf2::TimePointZero);
        
        double slam_x = map_to_base.transform.translation.x;
        double slam_y = map_to_base.transform.translation.y;
        
        // 提取角度
        tf2::Quaternion q(...);
        double slam_theta;
        tf2::Matrix3x3(q).getRPY(roll, pitch, slam_theta);
        
        // 计算误差
        double error_x = slam_x - current_x_;
        double error_y = slam_y - current_y_;
        double error_theta = slam_theta - current_theta_;
        
        double error_dist = std::sqrt(error_x * error_x + error_y * error_y);
        
        // 如果误差超过阈值，平滑校正里程计
        const double CORRECTION_THRESHOLD_DIST = 0.05;   // 5cm
        const double CORRECTION_GAIN = 0.3;  // 每次校正30%的误差
        
        if (error_dist > CORRECTION_THRESHOLD_DIST) {
            current_x_ += error_x * CORRECTION_GAIN;
            current_y_ += error_y * CORRECTION_GAIN;
            current_theta_ += error_theta * CORRECTION_GAIN;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "🔄 SLAM校正: 误差 %.2fcm → 校正 %.2fcm",
                error_dist * 100, error_dist * CORRECTION_GAIN * 100);
        }
    } catch (const tf2::TransformException& ex) {
        // SLAM 可能还未初始化，静默处理
    }
}
```

---

## 5. 视觉任务 - 颜色跟踪与篮筐检测

**模块:** `unified_vision_node` (`get-toy/src/color_tracking_node/src/unified_vision_node.cpp`)

**功能:**
统一视觉处理节点，支持两种模式：
1. **彩色跟踪** (ColorTracking): 识别红色/蓝色目标，计算偏转角度和垂直偏移，供下位机进行视觉伺服。
2. **黑色篮筐检测** (BasketDetection): 识别放置框（方形黑色区域），计算位置和方向。

通过订阅 `/task_command` 话题（0=停止, 1=彩色跟踪, 2=篮筐检测）动态切换模式，两种模式均使用统一输出格式：`"status,rad,y_offset"`。

**核心代码 - 彩色跟踪:**
```cpp
// HSV 颜色分割（红色+蓝色，严格排除黑色）
static void makeMaskHSV(const Mat& hsv, Mat& maskOut) {
    Mat maskRed, maskBlue, lower, upper;
    
    // 红色检测 (H: 0-10 和 170-180)
    inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), lower);
    inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), upper);
    bitwise_or(lower, upper, maskRed);
    
    // 蓝色检测（严格排除黑色）
    Mat blue_mask_primary, black_mask;
    inRange(hsv, Scalar(100, 150, 80), Scalar(130, 255, 255), blue_mask_primary);  // 高饱和度
    inRange(hsv, Scalar(0, 0, 0), Scalar(180, 50, 50), black_mask);  // 低饱和度+低亮度=黑色
    bitwise_and(blue_mask_primary, Scalar(255) - black_mask, maskBlue);  // 减去黑色
    
    bitwise_or(maskRed, maskBlue, maskOut);
    
    // 形态学处理
    Mat kernel_open = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat kernel_close = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    morphologyEx(maskOut, maskOut, MORPH_OPEN, kernel_open, Point(-1,-1), 2);
    morphologyEx(maskOut, maskOut, MORPH_CLOSE, kernel_close, Point(-1,-1), 2);
}

// 处理彩色跟踪帧
void UnifiedVisionNode::processColorTracking(Mat& frame) {
    Mat hsv, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    makeMaskHSV(hsv, mask);
    
    std::vector<std::vector<Point>> contours;
    findContours(mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    bool found = false;
    Point2f target_center(0, 0);
    double best_score = -1.0;
    
    for (const auto& c : contours) {
        double area = contourArea(c);
        if (area < 2000 || area > 50000) continue;  // 面积筛选
        
        double score = calculateRegularityScore(c);  // 规整度评分
        if (score > best_score) {
            best_score = score;
            Moments m = moments(c);
            if (std::abs(m.m00) > 1e-5) {
                target_center = Point2f(m.m10 / m.m00, m.m01 / m.m00);
                found = true;
            }
        }
    }
    
    Point2f image_center(frame.cols / 2.0f, frame.rows / 2.0f);
    double deflection_rad = 0.0;
    int dy = 0;
    
    if (found) {
        float dx = target_center.x - image_center.x;
        float dy_f = image_center.y - target_center.y;  // 图像坐标系反转
        deflection_rad = std::atan2(dx, dy_f);  // 偏转角度 (rad)
        dy = static_cast<int>(std::round(target_center.y - image_center.y));
    }
    
    // 发布统一格式："status,rad,dy"
    auto msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << (found ? "1.00" : "0.00") << ","  // 1=彩色跟踪有效
        << deflection_rad << ","
        << dy;
    msg.data = oss.str();
    color_result_pub_->publish(msg);
}
```

**核心代码 - 篮筐检测:**
```cpp
// 黑色篮筐形状评估
static ShapeMetrics evaluateShape(const std::vector<Point>& contour) {
    ShapeMetrics metrics;
    double area = contourArea(contour);
    
    // 多边形近似
    std::vector<Point> approx;
    approxPolyDP(contour, approx, 0.04 * arcLength(contour, true), true);
    int vertices = approx.size();
    
    // 计算矩形度、方形度、凸性等
    Rect boundingBox = boundingRect(contour);
    double boxArea = boundingBox.width * boundingBox.height;
    double aspectRatio = std::min(boundingBox.width, boundingBox.height) / 
                        (double)std::max(boundingBox.width, boundingBox.height);
    
    metrics.rectangularity = boxArea > 0 ? area / boxArea : 0;
    metrics.squareness = aspectRatio;
    metrics.isRectangle = (vertices == 4);
    metrics.isSquare = metrics.isRectangle && (aspectRatio > 0.85);
    
    // 根据形状给出不同评分
    if (metrics.isSquare) {
        metrics.score = 100.0 + metrics.squareness * 50;  // 正方形最高分
    } else if (metrics.isRectangle) {
        metrics.score = 80.0 + metrics.rectangularity * 20;
    }
    return metrics;
}

// 处理篮筐检测帧
void UnifiedVisionNode::processBasketDetection(Mat& frame) {
    Mat hsv, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // 黑色检测 (V < 60, S < 80)
    inRange(hsv, Scalar(0, 0, 0), Scalar(180, 80, 60), mask);
    
    std::vector<std::vector<Point>> contours;
    findContours(mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    bool found = false;
    Point2f basket_center(0, 0);
    double best_score = -1.0;
    
    for (const auto& c : contours) {
        double area = contourArea(c);
        if (area < 3000 || area > 60000) continue;  // 篮筐面积范围
        
        ShapeMetrics metrics = evaluateShape(c);
        if (metrics.score > best_score && metrics.isRectangle) {
            best_score = metrics.score;
            Moments m = moments(c);
            basket_center = Point2f(m.m10 / m.m00, m.m01 / m.m00);
            found = true;
        }
    }
    
    // 发布统一格式："status,rad,dy" (status=2 表示黑色检测)
    auto msg = std_msgs::msg::String();
    msg.data = found ? "2.00,0.0,0" : "0.00,0.0,0";
    color_result_pub_->publish(msg);
}
```

---

## 6. 任务系统 - 高层任务编排

**模块:** `mission_controller` (`navigation_control/scripts/mission_controller.py`)

**功能:**
高级任务调度节点，管理机器人执行复杂任务序列。支持多种任务类型：
- `goto`: 前往航点（包含位置和朝向）
- `wait`: 等待指定时间
- `start_vision`: 启动视觉任务
- `stop_vision`: 停止视觉任务
- `set_vision_mode`: 切换视觉模式（彩色跟踪 / 篮筐检测）
- `sequence`: 执行航点序列

从 YAML 文件加载任务定义，监控导航状态，支持暂停/恢复/停止操作。

**核心代码:**
```python
class VisionMode(Enum):
    DISABLED = 0          # 视觉关闭
    COLOR_TRACKING = 1    # 颜色跟踪任务（默认）
    BASKET_DETECTION = 2  # 放置框识别任务

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 加载航点和任务配置
        self.waypoints = {}  # {name: {x, y, yaw, description}}
        self.missions = {}   # {name: {tasks: [...]}}
        self.load_waypoints()  # 从 waypoints.yaml 加载
        self.load_missions()   # 从 missions.yaml 加载
        
        # 发布器
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.task_command_pub = self.create_publisher(Int32, '/task_command', 10)
        
        # 服务
        self.execute_srv = self.create_service(
            Trigger, '/mission/execute', self.execute_mission_callback)
        
        # 任务状态机定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.mission_tick)
        
        self.vision_mode = VisionMode.COLOR_TRACKING  # 默认模式
    
    def execute_mission_callback(self, request, response):
        """执行预设任务"""
        mission_name = self.pending_mission_name
        if mission_name not in self.missions:
            response.success = False
            response.message = f'任务 "{mission_name}" 不存在'
            return response
        
        self.current_mission = self.missions[mission_name]
        self.current_task_index = 0
        self.state = MissionState.RUNNING
        
        response.success = True
        response.message = f'开始执行任务: {mission_name}'
        self.get_logger().info(f'✅ {response.message}')
        return response
    
    def mission_tick(self):
        """任务状态机 - 每100ms调用"""
        if self.state != MissionState.RUNNING:
            return
        
        if self.current_task_index >= len(self.current_mission['tasks']):
            self.state = MissionState.COMPLETED
            self.get_logger().info('✅ 任务序列完成！')
            return
        
        current_task = self.current_mission['tasks'][self.current_task_index]
        task_type = TaskType(current_task['type'])
        
        # 执行当前任务
        if task_type == TaskType.GOTO:
            self.execute_goto_task(current_task)
        elif task_type == TaskType.WAIT:
            self.execute_wait_task(current_task)
        elif task_type == TaskType.SET_VISION_MODE:
            self.execute_set_vision_mode(current_task)
            self.current_task_index += 1  # 立即完成
    
    def execute_goto_task(self, task):
        """执行前往航点任务"""
        waypoint_name = task['waypoint']
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'航点 "{waypoint_name}" 不存在')
            self.state = MissionState.FAILED
            return
        
        wp = self.waypoints[waypoint_name]
        
        # 发布目标航点
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        
        # 四元数转换
        q = quaternion_from_euler(0, 0, wp['yaw'])
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'📍 前往航点: {waypoint_name} ({wp["x"]:.2f}, {wp["y"]:.2f})')
    
    def execute_set_vision_mode(self, task):
        """切换视觉模式"""
        mode_name = task['mode']  # 'color_tracking' or 'basket_detection'
        
        if mode_name == 'color_tracking':
            self.vision_mode = VisionMode.COLOR_TRACKING
            cmd = Int32()
            cmd.data = 1
            self.task_command_pub.publish(cmd)
            self.get_logger().info('👀 切换到: 颜色跟踪模式')
        elif mode_name == 'basket_detection':
            self.vision_mode = VisionMode.BASKET_DETECTION
            cmd = Int32()
            cmd.data = 2
            self.task_command_pub.publish(cmd)
            self.get_logger().info('👀 切换到: 篮筐检测模式')
```

---

## 7. 地图重发布

**模块:** `map_republisher` (`navigation_control/scripts/map_republisher.py`)

**功能:**
解决 ROS2 地图话题 QoS 兼容性问题。slam_toolbox 发布的地图使用 `TRANSIENT_LOCAL` 持久化策略，部分节点（如自定义 A* 规划器、ICP 重定位）需要 `VOLATILE` 策略才能正常订阅。该节点订阅 `/map` 原始地图，以 5Hz 频率重发布到 `/map_viz`，确保所有下游节点都能稳定获取地图数据。

**核心代码:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # 订阅 slam_toolbox 的 TRANSIENT_LOCAL 地图
        qos_transient = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',  # slam_toolbox 发布的原始地图
            self.map_callback,
            qos_transient
        )
        
        # 发布 VOLATILE 地图给 ICP 和 A*
        qos_volatile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map_viz',  # 供 A* 和 ICP 使用
            qos_volatile
        )
        
        self.map_data = None
        
        # 定时重发布 (5Hz)
        self.timer = self.create_timer(0.2, self.republish_map)
        
        self.get_logger().info('地图重发布节点已启动: /map → /map_viz')
    
    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info(f'收到地图: {msg.info.width}x{msg.info.height}')
        self.map_data = msg
    
    def republish_map(self):
        if self.map_data is not None:
            self.map_data.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_data)
```

---

## 8. Cartographer 实时建图 (证明使用 SLAM 技术)

**模块:** `cartographer_ros` (通过 `cartographer.lua` 配置)

**功能:**
这是证明使用 SLAM 技术的核心。Cartographer 采用实时建图模式，机器人从固定起始点 (0,0) 启动，边移动边构建地图。它订阅激光雷达的 `/scan` 话题和轮式里程计的 `/odom` 话题，进行扫描匹配和图优化，实时构建环境的 2D 栅格地图，并发布 `map` -> `odom` 的 TF 变换。关键特性：
- 无需预建图，启动即可导航
- 支持闭环检测，自动消除累积误差
- 融合里程计数据，提高定位精度
- 实时发布地图供导航使用

**核心配置代码:**
```lua
-- src/navigation_control/config/cartographer.lua

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",         -- Cartographer 发布 map->odom
  odom_frame = "odom",
  provide_odom_frame = false,       -- 由轮式里程计发布 odom->base_link
  use_odometry = true,              -- 融合轮式里程计
  num_laser_scans = 1,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- ========== 2D SLAM 实时建图配置 ==========
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 激光扫描参数
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- 子地图构建（实时建图模式）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90     -- 每90帧激光创建一个子地图
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 扫描匹配参数
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- ========== 后端优化配置（闭环检测）==========
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
```

---

## 9. 航点管理系统

**模块:** `waypoint_manager` (`navigation_control/scripts/waypoint_manager.py`)

**功能:**
提供完整的航点生命周期管理：
- **保存航点**: 获取当前 `map→base_link` 位姿，保存为命名航点
- **加载/存储**: 从 `waypoints.yaml` 读写航点数据（包含 x, y, yaw, description）
- **可视化**: 在 RViz 中显示所有航点为带箭头的 Marker
- **查询/删除**: 支持列出、删除指定航点
- **导航触发**: 发布航点到 `/goal_pose` 供 A* 规划器使用

使用 TF 监听器实时获取机器人位姿，支持动态保存当前位置。航点数据持久化存储在包的 `maps/` 目录。

---

## 10. A* 路径规划器 - 矩形Footprint避障

**模块:** `astar_planner` (`navigation_control/scripts/astar_planner.py`)

**功能:**
自定义全局路径规划器，专为全向轮机器人优化。核心特性：
- **矩形Footprint碰撞检测**: 考虑机器人实际尺寸（342mm×300mm + 安全裕量），检查四个角点是否碰撞
- **A*算法**: 使用优先队列，启发式函数为欧氏距离，支持斜向移动惩罚
- **路径后处理**:
  1. 插值（增加密度）
  2. 平滑（生成圆弧过渡）
  3. 曲率简化（保留转角点，简化直线段）
- **实时规划**: 订阅 `/map_viz` 和 `/goal_pose`，发布到 `/planned_path`

规划出的路径可直接供 Pure Pursuit 路径跟踪器使用。

---

## 11. 航点管理系统

**模块:** `waypoint_manager` (`navigation_control/scripts/waypoint_manager.py`)

**功能:**
提供完整的航点生命周期管理：
- **保存航点**: 获取当前 `map→base_link` 位姿，保存为命名航点
- **加载/存储**: 从 `waypoints.yaml` 读写航点数据（包含 x, y, yaw, description）
- **可视化**: 在 RViz 中显示所有航点为带箭头的 Marker
- **查询/删除**: 支持列出、删除指定航点
- **导航触发**: 发布航点到 `/goal_pose` 供 A* 规划器使用

使用 TF 监听器实时获取机器人位姿，支持动态保存当前位置。航点数据持久化存储在包的 `maps/` 目录。

**核心代码:**
```python
class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # 参数
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.waypoints_file = self.get_parameter('waypoints_file').value
        
        # 完整路径（包的maps目录）
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory('navigation_control')
        maps_dir = os.path.join(pkg_dir, 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        self.waypoints_file = os.path.join(maps_dir, self.waypoints_file)
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 航点数据 {name: {'x': float, 'y': float, 'yaw': float, 'description': str}}
        self.waypoints = {}
        self.load_waypoints()
        
        # 服务
        self.save_srv = self.create_service(
            Trigger, '/waypoint/save', self.save_waypoint_callback)
        self.list_srv = self.create_service(
            Trigger, '/waypoint/list', self.list_waypoints_callback)
        self.goto_srv = self.create_service(
            Trigger, '/waypoint/goto', self.goto_waypoint_callback)
        
        # 发布器
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 定时发布航点标记（1Hz）
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info(f'🗺️  航点管理器已启动，已加载 {len(self.waypoints)} 个航点')
    
    def get_current_pose(self):
        """获取机器人当前位姿（map坐标系）"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # 四元数转欧拉角
            quat = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'获取位姿失败: {e}')
            return None, None, None
    
    def save_waypoint_callback(self, request, response):
        """保存当前位置为航点"""
        waypoint_name = self.pending_waypoint_name or f'waypoint_{len(self.waypoints)+1}'
        self.pending_waypoint_name = None
        
        x, y, yaw = self.get_current_pose()
        if x is None:
            response.success = False
            response.message = '❌ 无法获取当前位姿'
            return response
        
        # 保存航点
        self.waypoints[waypoint_name] = {
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'description': ''
        }
        
        # 写入YAML文件
        if self.save_waypoints_to_file():
            response.success = True
            response.message = f'✅ 航点 "{waypoint_name}" 已保存: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)'
        else:
            response.success = False
            response.message = '❌ 保存到文件失败'
        
        return response
    
    def publish_markers(self):
        """在RViz中可视化所有航点"""
        marker_array = MarkerArray()
        
        for i, (name, wp) in enumerate(self.waypoints.items()):
            # 箭头标记（显示位置和朝向）
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.1
            
            # 四元数朝向
            q = quaternion_from_euler(0, 0, wp['yaw'])
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = 0.3  # 箭头长度
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            
            # 文字标记（显示航点名称）
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'waypoint_names'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z = 0.3
            text_marker.text = name
            text_marker.scale.z = 0.15
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
```

**模块:** `astar_planner` (`navigation_control/scripts/astar_planner.py`)

**功能:**
自定义全局路径规划器，专为全向轮机器人优化。核心特性：
- **矩形Footprint碰撞检测**: 考虑机器人实际尺寸（342mm×300mm + 安全裕量），检查四个角点是否碰撞
- **A*算法**: 使用优先队列，启发式函数为欧氏距离，支持斜向移动惩罚
- **路径后处理**:
  1. 插值（增加密度）
  2. 平滑（生成圆弧过渡）
  3. 曲率简化（保留转角点，简化直线段）
- **实时规划**: 订阅 `/map_viz` 和 `/goal_pose`，发布到 `/planned_path`

规划出的路径可直接供 Pure Pursuit 路径跟踪器使用。

**核心代码 - 矩形Footprint碰撞检测:**
```python
class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # 机器人实际尺寸（从URDF）
        self.declare_parameter('robot_length', 0.342)  # 342mm
        self.declare_parameter('robot_width', 0.300)   # 300mm
        self.declare_parameter('safety_margin', 0.03)  # 30mm安全裕量
        
        self.robot_length = self.get_parameter('robot_length').value
        self.robot_width = self.get_parameter('robot_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        
        # 机器人半长和半宽（用于碰撞检测）
        self.robot_half_length = (self.robot_length + self.safety_margin) / 2.0
        self.robot_half_width = (self.robot_width + self.safety_margin) / 2.0
    
    def is_footprint_collision_free(self, x_world, y_world, yaw=0.0):
        """检查机器人矩形footprint是否与障碍物碰撞
        
        Args:
            x_world, y_world: 机器人中心世界坐标(米)
            yaw: 机器人航向角(弧度)
        
        Returns:
            bool: 无碰撞返回True
        """
        if self.map_data is None:
            return False
        
        # 计算机器人矩形的四个角点（世界坐标）
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        corners = [
            (self.robot_half_length, self.robot_half_width),   # 右前
            (self.robot_half_length, -self.robot_half_width),  # 右后
            (-self.robot_half_length, self.robot_half_width),  # 左前
            (-self.robot_half_length, -self.robot_half_width)  # 左后
        ]
        
        # 检查每个角点是否碰撞
        for local_x, local_y in corners:
            # 旋转到世界坐标系
            world_x = x_world + local_x * cos_yaw - local_y * sin_yaw
            world_y = y_world + local_x * sin_yaw + local_y * cos_yaw
            
            # 转换为栅格坐标
            grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
            
            # 边界检查
            if grid_x < 0 or grid_x >= self.map_info.width or \
               grid_y < 0 or grid_y >= self.map_info.height:
                return False  # 超出地图范围
            
            # 碰撞检查（占用概率 > 50% 或未知区域）
            if self.map_data[grid_y, grid_x] > 50 or self.map_data[grid_y, grid_x] < 0:
                return False  # 碰撞
        
        return True  # 所有角点都安全
```

**A*算法核心:**
```python
def astar(self, start, goal):
    """A*路径搜索算法"""
    open_set = [(0, start)]  # (f_cost, node)
    closed_set = set()
    g_costs = {start: 0}
    came_from = {}
    
    while open_set:
        # 1. 取f_cost最小的节点
        current_f, current = heappop(open_set)
        
        if current == goal:
            # 回溯路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        if current in closed_set:
            continue
        closed_set.add(current)
        
        # 2. 遍历邻居节点（8方向）
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1),  # 直角
                       (-1,-1), (-1,1), (1,-1), (1,1)]:  # 斜向
            neighbor = (current[0] + dx, current[1] + dy)
            
            # 边界检查
            if not self.is_valid_cell(neighbor):
                continue
            
            # Footprint碰撞检测
            neighbor_world = self.grid_to_world(neighbor[0], neighbor[1])
            if not self.is_footprint_collision_free(neighbor_world[0], neighbor_world[1]):
                continue
            
            # 计算g_cost（斜向移动惩罚）
            move_cost = 1.414 * self.diagonal_penalty if (dx != 0 and dy != 0) else 1.0
            tentative_g = g_costs[current] + move_cost
            
            if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                g_costs[neighbor] = tentative_g
                h_cost = self.heuristic(neighbor, goal)  # 欧氏距离
                f_cost = tentative_g + h_cost
                heappush(open_set, (f_cost, neighbor))
                came_from[neighbor] = current
    
    return None  # 未找到路径

def heuristic(self, node1, node2):
    """启发式函数：欧氏距离"""
    return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
```

---

## 总结

本项目实现了一套完整的全向轮机器人 SLAM 导航系统，从底层串口通信、多传感器融合里程计、实时SLAM建图与定位、视觉识别、到上层路径规划与任务调度，各模块协同工作。**关键技术证明**：

✅ **SLAM技术应用**:
- 使用 `Google Cartographer` 进行实时 2D SLAM 建图
- 固定起始点 (0,0) 启动，坐标系一致
- 闭环检测自动优化地图，消除累积误差
- 发布 `map→odom` TF变换，提供全局位姿校正
- 边建图边导航，无需预建图步骤

✅ **多传感器融合**:
- 激光雷达（RPLIDAR A1）：2D 扫描数据，8Hz频率
- 轮式编码器：底盘速度反馈，实时积分
- IMU：姿态角度（yaw）融合，提高旋转精度
- Cartographer 内部融合所有传感器数据

✅ **自主导航能力**:
- 实时地图更新，动态环境适应
- A* 矩形 Footprint 路径规划
- 航点管理与任务调度
- 视觉伺服（颜色跟踪、篮筐检测）
- Pure Pursuit 路径跟踪控制

✅ **系统优势**:
- 无需预建图，启动即可导航
- 固定起始点，坐标系一致性好
- 闭环检测，长期运行精度高
- 完全自定义导航栈，轻量高效

整个系统已在实际硬件平台上运行验证，能够稳定完成实时建图、定位、导航等任务。适用于动态环境和无预建图需求的场景。
