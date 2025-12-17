从https://github.com/Livox-SDK/livox_ros_driver克隆而来。该项目用于资源受限的嵌入式设备，移除PCL库和rivz的依赖，移除hub相关代码，只保留rosbag的录制功能。已经在树莓派4B（Ubuntu 20.05+ROS noetic）已经验证编译通过。
## Livox ROS 驱动完整代码结构分析

### 目录结构

```
ws_livox/src/livox_ros_driver/livox_ros_driver/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── livox_lidar.launch          # LiDAR 启动文件
│   ├── livox_lidar_msg.launch      # 自定义消息格式启动
│   ├── livox_lidar_rviz.launch     # 带可视化启动
│   └── lvx_to_rosbag.launch        # LVX 转 ROSBAG
├── config/
│   └── livox_lidar_config.json     # 配置文件
├── livox_ros_driver/               # 核心源代码
│   ├── common/                     # 通用库
│   │   ├── comm/                   # 通信协议
│   │   ├── FastCRC/                # CRC 校验
│   │   ├── rapidjson/              # JSON 解析
│   │   └── rapidxml/               # XML 解析
│   ├── timesync/                   # 时间同步
│   │   ├── timesync.h/cpp          # 时间同步主类
│   │   └── user_uart/              # UART 通信
│   ├── msg/                        # ROS 消息定义
│   │   ├── CustomMsg.msg           # 自定义点云消息
│   │   └── CustomPoint.msg         # 自定义点
│   ├── include/livox_ros_driver.h  # 版本信息
│   ├── lds.h/cpp                   # 基础数据结构和函数
│   ├── lds_lidar.h/cpp             # LiDAR 设备管理
│   ├── lds_lvx.h/cpp               # LVX 文件处理
│   ├── ldq.h                       # 数据队列
│   ├── lddc.h/cpp                  # 数据分发控制
│   ├── lvx_file.h/cpp              # LVX 文件读写
│   └── livox_ros_driver.cpp        # 主程序入口
└── cmake/
    └── version.cmake               # 版本管理
```

---

## 核心组件调用关系图

```
┌─────────────────────────────────────────────────────────────┐
│                    主程序入口 (main)                        │
│              livox_ros_driver.cpp:main()                    │
└───────────────────────┬───────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              数据分发控制 (Lddc)                            │
│              lddc.h/cpp                                     │
│  - 管理发布者、订阅者                                       │
│  - 控制数据格式转换 (PointCloud2/CustomMsg)                │
│  - 管理 ROSBAG 录制                                        │
└───────────────────────┬───────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│  LiDAR数据  │ │  LVX文件   │ │   IMU数据   │
│  源管理器   │ │  读取器    │ │    读取器   │
│ lds_lidar   │ │  lds_lvx   │ │             │
└──────┬──────┘ └──────┬──────┘ └──────┬──────┘
       │               │               │
       └───────────────┴───────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│              基础数据结构 (Lds)                            │
│              lds.h/cpp                                      │
│  - 设备信息存储                                            │
│  - 数据队列管理 (基于 ldq.h)                               │
│  - 点云数据转换函数                                        │
│  - 时间戳处理                                              │
└───────────────────────┬───────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              数据队列 (LidarDataQueue)                     │
│              ldq.h                                          │
│  - 环形缓冲区实现                                          │
│  - 线程安全的队列操作                                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 详细调用流程

### 1. 启动流程 (Initialization Flow)

```
main()
  │
  ├─► 解析命令行参数和 ROS 参数
  │   └─► xfer_format (0=PointCloud2, 1=CustomMsg, 2=PCL)
  │   └─► data_src (0=LiDAR, 1=Hub, 2=LVX file)
  │   └─► publish_freq
  │   └─► multi_topic
  │
  ├─► 创建 Lddc 对象
  │   Lddc::Lddc()
  │   └─► 设置数据格式、频率、话题模式
  │
  ├─► 根据 data_src 选择数据源
  │   │
  │   ├─► data_src == 0 (原始 LiDAR)
  │   │   │
  │   │   ├─► LdsLidar::GetInstance()  // 获取单例
  │   │   │
  │   │   ├─► Lddc::RegisterLds()      // 注册数据源
  │   │   │
  │   │   └─► LdsLidar::InitLdsLidar() // 初始化 LiDAR
  │   │       ├─► 读取配置文件
  │   │       ├─► 初始化 SDK
  │   │       ├─► 设置广播码白名单
  │   │       ├─► 设置回调函数
  │   │       └─► 启动设备扫描
  │   │
  │   └─► data_src == 2 (LVX 文件)
  │       │
  │       ├─► LdsLvx::GetInstance()
  │       │
  │       ├─► Lddc::RegisterLds()
  │       │
  │       └─► LdsLvx::InitLdsLvx()      // 初始化 LVX 文件读取
  │
  └─► 主循环
      └─► Lddc::DistributeLidarData()   // 分发数据
```

---

### 2. 数据接收流程 (Data Reception Flow)

```
LiDAR 设备
    │
    │ UDP 数据包
    │
    ▼
LdsLidar::OnLidarDataCb()  // 数据接收回调
    │
    ├─► 解析数据包头
    ├─► 检查广播码
    ├─► 验证时间戳
    │
    ├─► 选择转换函数
    │   └─► GetConvertHandler(data_type)
    │       └─► 根据数据类型选择：
    │           ├─► LivoxRawPointToPxyzrtl
    │           ├─► LivoxSpherPointToPxyzrtl
    │           ├─► LivoxExtendRawPointToPxyzrtl
    │           └─► ...
    │
    └─► Lds::StorageRawPacket()  // 存储到队列
        │
        └─► QueuePushAny()        // 推入环形缓冲区
            └─► ldq.h 实现
```

---

### 3. 数据分发流程 (Data Distribution Flow)

```
主循环
  │
  └─► Lddc::DistributeLidarData()
      │
      ├─► 遍历所有设备
      │   │
      │   └─► 检查队列状态
      │       │
      │       ├─► 队列有足够数据？
      │       │
      │       └─► 根据 transfer_format 分发
      │           │
      │           ├─► kPointCloud2Msg (0)
      │           │   └─► PublishPointcloud2()
      │           │       ├─► GetPublishStartTime()
      │           │       ├─► QueuePrePop()
      │           │       ├─► InitPointcloud2MsgHeader()
      │           │       ├─► GetConvertHandler()
      │           │       ├─► 填充数据到 sensor_msgs::PointCloud2
      │           │       ├─► publisher.publish()
      │           │       └─► 可选：bag_->write()  // 录制到 rosbag
      │           │
      │           └─► kLivoxCustomMsg (1)
      │               └─► PublishCustomPointcloud()
      │                   ├─► 类似流程
      │                   └─► 填充 livox_ros_driver::CustomMsg
      │                       └─► 包含：时间戳、线号、标签、反射率等
      │
      └─► 处理 IMU 数据（如果有）
          └─► PublishImuData()
```

---

### 4. 文件读取流程 (File Reading Flow - LVX)

```
main()
  │
  └─► data_src == 2 (LVX 文件)
      │
      ├─► LdsLvx::InitLdsLvx()
      │   │
      │   └─► LvxFileHandle::LvxFileHandle()
      │
      └─► 启动读取线程
          │
          └─► LdsLvx::ReadLvxFile()
              │
              ├─► LvxFileHandle::LoadLvxFile()
              ├─► 读取文件头
              ├─► 读取设备信息
              │
              └─► 循环读取帧数据
                  │
                  ├─► lvx_file_->GetPacketsOfFrame()
                  │
                  └─► 转换为内部格式
                      └─► Lds::StorageRawPacket()
```

---

## 核心类详解

### 1. **Lds 类 (lds.h/cpp)**

**基类，定义通用接口和数据结构**

```cpp
class Lds {
public:
  // 存储原始数据包到队列
  void StorageRawPacket(uint8_t handle, LivoxEthPacket* eth_packet);
  
  // 获取设备类型
  uint8_t GetDeviceType(uint8_t handle);
  
  // 重置设备
  static void ResetLidar(LidarDevice *lidar, uint8_t data_src);
  
  // 检查所有队列是否为空
  bool IsAllQueueEmpty();
  
  // 请求退出
  void RequestExit();
  
public:
  uint8_t lidar_count_;                 // LiDAR 数量
  LidarDevice lidars_[kMaxSourceLidar]; // 设备数组
  Semaphore semaphore_;                 // 信号量
  
protected:
  uint32_t buffer_time_ms_;             // 缓冲区时间
  uint8_t data_src_;                    // 数据源类型
  
private:
  volatile bool request_exit_;          // 退出标志
};
```

**关键数据结构 `LidarDevice`：**

```cpp
typedef struct {
  uint8_t handle;                    // 设备句柄
  uint8_t data_src;                  // 数据源
  uint8_t raw_data_type;             // 原始数据类型
  bool data_is_pubulished;           // 是否已发布
  uint32_t timestamp_type;           // 时间戳类型
  volatile uint32_t packet_interval; // 包间隔
  volatile LidarConnectState connect_state;  // 连接状态
  DeviceInfo info;                   // 设备信息
  LidarPacketStatistic statistic_info;      // 统计信息
  LidarDataQueue data;               // 点云数据队列
  LidarDataQueue imu_data;           // IMU 数据队列
  uint32_t firmware_ver;             // 固件版本
  UserConfig config;                 // 用户配置
  ExtrinsicParameter extrinsic_parameter;   // 外参
} LidarDevice;
```

---

### 2. **LdsLidar 类 (lds\_lidar.h/cpp)**

**继承自 Lds，管理真实 LiDAR 设备**

```cpp
class LdsLidar : public Lds {
public:
  static LdsLidar *GetInstance(uint32_t interval_ms);
  
  // 初始化
  int InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                   const char *user_config_path);
  
private:
  // SDK 回调函数（静态）
  static void OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                            uint32_t data_num, void *client_data);
  
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  
  // 采样控制回调
  static void StartSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data);
  
  // 配置回调
  static void ControlFanCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  
  static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                        uint8_t response, void *clent_data);
  
  // 时间同步回调
  static void SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                               uint8_t response, void *client_data);
  
  static void ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                      void *client_data);
};
```

**初始化流程：**

1. 读取 JSON 配置文件
2. 解析广播码列表
3. 初始化 Livox SDK
4. 设置回调函数
5. 启用自动连接模式
6. 启动设备扫描线程

---

### 3. **Lddc 类 (lddc.h/cpp)**

**Lidar Data Distribute Control - 数据分发控制器**

```cpp
class Lddc {
public:
  Lddc(int format, int multi_topic, int data_src, int output_type,
       double frq, std::string &frame_id, bool lidar_bag, bool imu_bag);
  
  // 注册数据源
  int RegisterLds(Lds *lds);
  
  // 主循环函数
  void DistributeLidarData(void);
  
  // 创建 rosbag 文件
  void CreateBagFile(const std::string &file_name);
  
private:
  // 发布函数
  uint32_t PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle);
  
  uint32_t PublishCustomPointcloud(LidarDataQueue *queue,
                                   uint32_t packet_num, uint8_t handle);
  
  uint32_t PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                          uint8_t handle);
  
  // 获取发布者
  ros::Publisher *GetCurrentPublisher(uint8_t handle);
  ros::Publisher *GetCurrentImuPublisher(uint8_t handle);
  
  // 轮询数据
  void PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t handle, LidarDevice *lidar);
  
  // 填充消息
  void FillPointsToCustomMsg(livox_ros_driver::CustomMsg& livox_msg,
      LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time,
      uint32_t point_interval, uint32_t echo_num);
  
private:
  uint8_t transfer_format_;    // 传输格式 (0/1/2)
  uint8_t use_multi_topic_;    // 是否多话题
  uint8_t data_src_;           // 数据源
  uint8_t output_type_;        // 输出类型
  double publish_frq_;         // 发布频率
  uint32_t publish_period_ns_; // 发布周期
  
  std::string frame_id_;       // 坐标系
  
  bool enable_lidar_bag_;      // 是否录制点云
  bool enable_imu_bag_;        // 是否录制IMU
  
  ros::Publisher *private_pub_[kMaxSourceLidar];     // 私有发布者
  ros::Publisher *global_pub_;                       // 全局发布者
  ros::Publisher *private_imu_pub_[kMaxSourceLidar]; // IMU 发布者
  ros::Publisher *global_imu_pub_;                   // 全局 IMU 发布者
  
  ros::NodeHandle *cur_node_;  // ROS 节点
  rosbag::Bag *bag_;           // rosbag 对象
  
  Lds *lds_;                   // 数据源指针
};
```

---

### 4. **LvxFileHandle 类 (lvx\_file.h/cpp)**

**LVX 文件读写管理**

```cpp
class LvxFileHandle {
public:
  // 打开 LVX 文件
  int Open(const char *lvx_path);
  
  // 关闭文件
  void Close();
  
  // 读取文件头
  int LoadLvxFile();
  
  // 获取一帧的数据
  int GetPacketsOfFrame(OutPacketBuffer &packet_stream);
  
  // 保存 LVX 文件（录制用）
  int SaveFrameToLvxFile(uint8_t device_handle,
                         LivoxEthPacket &packet,
                         uint8_t *packet_data);

private:
  // 写入文件头
  void WriteHeaderToLvxFile();
  
  // 写入设备信息
  void WriteDeviceInfoToLvxFile();
  
  // 写入帧数据
  int WriteFrameToLvxFile(const FrameHeader &frame_header,
                          uint32_t packet_num,
                          uint8_t *packet_data);
  
private:
  std::fstream cur_file_;              // 文件流
  std::string cur_file_path_;          // 文件路径
  LvxFilePublicHeader lvx_file_header_; // 文件头
  std::vector<DeviceInfo> device_info_list_;  // 设备列表
  uint64_t cur_frame_index_;           // 当前帧索引
  std::mutex file_mutex_;              // 文件互斥锁
};
```

---

### 5. **数据队列 (ldq.h)**

**环形缓冲区实现**

```cpp
typedef struct {
  volatile bool valid;           // 是否有效
  volatile uint32_t size;        // 队列大小
  volatile uint32_t rd_idx;      // 读索引
  volatile uint32_t wr_idx;      // 写索引
  volatile uint32_t used_size;   // 已用大小
  StoragePacket *storage_packet; // 数据包数组
} LidarDataQueue;



// 初始化队列
int32_t InitQueue(LidarDataQueue *queue, uint32_t size);

// 入队
int32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data,
                     uint32_t length, uint64_t time,
                     uint32_t point_num);

// 预出队（查看但不删除）
int32_t QueuePrePop(LidarDataQueue *queue,
                    StoragePacket *storage_packet);

// 更新读索引（确认删除）
void QueuePopUpdate(LidarDataQueue *queue);

// 检查是否满
bool QueueIsFull(const LidarDataQueue *queue);

// 检查是否空
bool QueueIsEmpty(const LidarDataQueue *queue);

// 获取已用大小
uint32_t QueueUsedSize(const LidarDataQueue *queue);

// 获取空大小
uint32_t QueueUnusedSize(const LidarDataQueue *queue);

// 销毁队列
void DeInitQueue(LidarDataQueue *queue);
```

---

## 数据格式说明

### 1. **原始数据格式**

Livox 设备支持多种原始数据格式（在 `lds.h` 中定义）：

```cpp
const DataTypePointInfoPair data_type_info_pair_table[kMaxPointDataType] = {
    {100, 1318, sizeof(LivoxRawPoint), 1},           // 0: Cartesian, 1 return
    {100, 918,  9,  1},                              // 1: Spherical, 1 return
    {96,  1362, 14, 1},                              // 2: Extend Cartesian
    {96,  978,  9,  1},                              // 3: Extend Spherical
    {48,  1362, sizeof(LivoxDualExtendRawPoint), 2}, // 4: Dual return Cartesian
    {48,  786,  sizeof(LivoxDualExtendSpherPoint), 2},// 5: Dual return Spherical
    {1,   42,   sizeof(LivoxImuPoint), 1},           // 6: IMU data
    {30,  1278, sizeof(LivoxTripleExtendRawPoint), 3},// 7: Triple return Cartesian
    {30,  678,  sizeof(LivoxTripleExtendSpherPoint), 3}// 8: Triple return Spherical
};
```

**不同 LiDAR 模型的数据类型：**

* Mid-40: 通常使用类型 0 (Cartesian) 或 2 (Extend Cartesian)
* Horizon/Tele: 可能使用其他类型

### 2. **内部转换格式**

所有原始数据最终转换为 `LivoxPointXyzrtl` 格式：

```cpp
typedef struct {
  float x;            // X坐标 (米)
  float y;            // Y坐标 (米)
  float z;            // Z坐标 (米)
  float reflectivity; // 反射率
  uint8_t tag;        // 标签
  uint8_t line;       // 线号 (多线LiDAR)
} LivoxPointXyzrtl;
```

**点大小：16 字节**

### 3. **ROS 输出格式**

#### **Format 0: PointCloud2 (sensor\_msgs::PointCloud2)**

标准 ROS 点云格式：

* PointField: x, y, z, intensity
* 每个点 16 字节
* 兼容 RViz 和其他 ROS 工具

**适用场景：**

* 快速可视化
* 与其他 ROS 包集成
* 通用点云处理

#### **Format 1: LivoxCustomMsg (livox\_ros\_driver::CustomMsg)**

自定义格式，包含更多信息：

```cpp
# CustomMsg.msg
std_msgs/Header header    # 时间戳、坐标系
uint64 timebase           # 基准时间戳 (ns)
uint32 point_num          # 点数
uint8 lidar_id            # LiDAR ID
CustomPoint[] points      # 点数组

# CustomPoint.msg
uint32 offset_time        # 相对于 timebase 的时间偏移 (ns)
float32 x                 # X坐标
float32 y                 # Y坐标
float32 z                 # Z坐标
uint8 reflectivity        # 反射率
uint8 tag                 # 标签
uint8 line                # 线号 (多线LiDAR)
```

**适用场景：**

* 需要精确时间戳
* 需要线号信息（多线LiDAR）
* 需要标签信息
* 录制和分析

---

## 配置文件详解

**文件：`config/livox_lidar_config.json`**

```json
{
  "lidar_config": [
    {
      "broadcast_code": "3WEDH7600101xxx",
      "enable_connect": true,         // 是否连接此设备
      "enable_fan": true,            // 启用风扇
      "return_mode": 0,              // 回波模式 (0=单回波, 1=双回波)
      "coordinate": 0,               // 坐标类型 (0=笛卡尔, 1=球面)
      "imu_rate": 0,                 // IMU频率 (0=关闭, 其他值)
      "extrinsic_parameter_source": 0,  // 外参来源
      "enable_high_sensitivity": false   // 高灵敏度模式
    }
  ]
}
```

---

## 时间同步机制

Livox 支持多种时间同步模式（在 `lds.h` 中定义）：

```cpp
typedef enum {
  kTimestampTypeNoSync = 0,   // 不同步
  kTimestampTypePtp = 1,      // PTP 同步
  kTimestampTypePpsGps = 2,   // PPS + GPS RMC (串口)
  kTimestampTypePps = 3,      // 仅 PPS
} TimestampType;
```

**PPS 同步流程：**

1. LiDAR 接收到 PPS 脉冲
2. 内部时钟重置为 0
3. 后续点云时间戳相对于 PPS 脉冲
4. 下一 PPS 脉冲到来时再次重置

**实际时间戳计算（在 `lds.cpp:RawLdsStampToNs()`）：**

```cpp
if (kTimestampTypePps) {
  if (cur_timestamp.stamp < last_timestamp) {
    // 新的 PPS 周期开始
    // 记录当前系统时间作为基准
    packet_statistic->timebase = sync_time;
  }
  // 返回：timebase + packet_timestamp
  return timestamp + packet_statistic->timebase;
}
```

---


## 总结

### 核心调用链

```
main()
  └─► Lddc::DistributeLidarData() [主循环]
      └─► PollingLidarPointCloudData()
          └─► GetCurrentPublisher()
          │
          ├─► 根据 transfer_format 选择
          │   ├─► PublishPointcloud2()      // Format 0
          │   └─► PublishCustomPointcloud() // Format 1
          │
          └─► publisher.publish()  // ROS 发布
```

### 关键设计模式

1. **单例模式**：LdsLidar、LdsLvx、TimeSync
2. **策略模式**：不同的 transfer\_format、data\_src
3. **观察者模式**：SDK 回调函数
4. **模板方法**：Lds 基类，LdsLidar/LdsLvx 派生类
5. **生产者-消费者**：数据接收和分发

### 修改重点


1. ✅ 删除了 Hub 支持（不需要）
2. ✅ 删除了 PCL 依赖（使用 Format 1，LivoxCustomMsg）
3. ✅ 保留了核心功能：LiDAR 数据采集、LVX 录制
4. ✅ 代码更简洁，编译更快，依赖更少

## 测试

```
roslaunch livox_ros_driver livox_lidar.launch   xfer_format:=1   output_type:=1   lidar_bag:=true   imu_bag:=false   publish_freq:=10.0
```

现在可以看到录制的bag消息了。需要开三个终端，A:roscore B:rosbag play yourbagname.bag C:rostopic echo /livox/lidar

```
offset_time: 100032270
x: 1.628000020980835
y: 0.41100001335144043
z: -0.29100000858306885
reflectivity: 1
tag: 0
line: 0

```
