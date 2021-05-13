# ARTINX步兵上位机串口通信手册
ARTINX步兵上下位机通信主要关系到两个包。

1. `infantry_msgs`:   定义步兵上下位机通信的消息类型与节点间通信的消息类型

2. `infantry_serial`: 步兵上下位机的通信逻辑与启动文件

以传输双方颜色信息为例，具体步骤如下：

## step1: 定义消息类型

1. **添加消息文件**
   在`src/infantry_msgs/msg`文件夹下添加msg文件，将颜色信息的消息文件命名为`EnemyColor.msg`，内容如下：
```msg
# if True, enemy color is red
bool isRed
```
2. **修改`CMakeLists.txt`**
    在`src/infantry_msgs/CmakeLists.txt`下`add_message_files()`函数添加参数`EnemyColor.msg`，示例如下：

```
add_message_files(
    FILES
    GimbalRate.msg
    GimbalSetAngle.msg
    GimbalFdbAngle.msg
    EnemyColor.msg
)
```

3. **编译并测试**
运行如下命令
```bash
# 编译
catkin_make
# 更新消息类型
source devel/setup.bash
# 显示包中所含消息内容
rosmsg package infantry_msgs
```
结果显示如下：
```
infantry_msgs/GimbalFdbAngle
infantry_msgs/GimbalRate
infantry_msgs/GimbalSetAngle
infantry_msgs/EnemyColor
```
其中包含`EnemyColor`，编译正常。
   
## step2: 修改串口节点

#### 定义 PacketInfo

在`src/infantry_serial/include/PacketInfo.hpp`添加`ENEMY_COLOR_PACKET`包所对应ID与SIZE，内容如下：
```cpp
#ifndef PACKET_INFO_HPP
#define PACKET_INFO_HPP
    #define GIMBAL_ANGLE_SET_PACKET_ID 0x05
    #define GIMBAL_ANGLE_SET_PACKET_SIZE 10
    #define GIMBAL_ANGLE_FDB_PACKET_ID 0x06
    #define GIMBAL_ANGLE_FDB_PACKET_SIZE 8
    #define ENEMY_COLOR_PACKET_ID 0x07
    #define ENEMY_COLOR_PACKET_SIZE 1
#endif
```
> ****
> **注意：**
>1. ID范围在0x00至0xff之间（十六进制）
>2. SIZE的大小由包所包含的内容的字节长度决定，大小可以按如下方法来计算：
>```
>uint8_t     1
>short       2
>int         4
>float       4
>```
> 其中bool值使用uint8_t来计算长度
> ****
#
### 定义 PacketModel

1. 编辑头文件`src/infantry_serial/include/PacketModel.hpp`

```cpp
class EnemyColorPacket : public PacketModelProto
    {
    public:
        EnemyColorPacket();
        ~EnemyColorPacket() { ; }

        DataUnit<uint8_t> is_red;
    };
```
> ****
> **注意：**
> is_red在msg文件中定义为bool值，在Packet中需要定义为uint8_t。对于其他基本变量保持远种类即可。
> ****


1. 编辑源文件`src/infantry_serial/include/PacketModel.cpp`

```cpp
EnemyColorPacket::EnemyColorPacket():PacketModelProto(ENEMY_COLOR_PACKET_ID, ENEMY_COLOR_PACKET_SIZE)
{
	m_unit_list.push_back(&is_red);
}  
```
> ****
>**注意：** 
> 当一个msg文件包含多个数据时，`m_unit_list.push_back()`函数的顺序与下位机处理包的函数有关，此顺序需要与msg文件中的消息顺序保持一致。在与电控完成下位机相关代码的编写后，**此顺序若没有经过沟通，可能会导致问题**。
> ****

#### 定义Serial逻辑
编辑main文件`src/infantry_serial/include/main.cpp`
1. 添加头文件
在文件开头添加`#include "infantry_msgs/EnemyColor.h"`以调用消息所使用的头文件。
2. 编辑main函数
在`ros_topic_input_stream.Init();`前添加如下语句
```cpp
// initialize enemy_color_packet
SerialTemplatePacket<EnemyColorPacket> serial_enemy_color_packet;
RostopicTemplatePacket<infantry_msgs::EnemyColor,EnemyColorPacket> ros_enemy_color_packet("infantry_enemy_color");
serial_input_stream.RegisterPacket(&serial_enemy_color_packet);
ros_topic_output_stream.RegisterPacket(&ros_enemy_color_packet);
```
> ****
> **注意：**
> 1. 其中`"infantry_enemy_color"`即为serial节点收到下位机信息所发布消息所在的节点名。
> 2. `enemy_color`是下位机向上位机发布信息所使用的语法，如果需要上位机向下位机发布消息，需要参考`gimbal_angle_set`的用法：
> ```cpp
> // initialize gimbal_angle_set_packet
>    SerialTemplatePacket<GimbalAngleSetPacket> serial_gimble_angle_set_packet;
>    RostopicTemplatePacket<infantry_msgs::GimbalSetAngle,GimbalAngleSetPacket> ros_gimbal_angle_set_packet("infantry_gimbal_angle_set");
>    serial_output_stream.RegisterPacket(&serial_gimble_angle_set_packet);
>    ros_topic_input_stream.RegisterPacket(&ros_gimbal_angle_set_packet);
> ```
> 如上代码作出的改动如下：
> ```
> serial_output_stream      ->  serial_input_stream
> ros_topic_output_stream   ->  ros_topic_input_stream
> ```
> ****


## step3: 功能测试

1. 编译运行节点
```bash
catkin_make
source devel/setup.bash
rosrun infantry_serial infantry_serial_node
```
此前需要确保`rosmaster`已经正常运行。

2. 查看节点运行情况
在相同路径下打开新的终端，输入命令
```bash
source devel/setup.bash
rosnode info infantry_serial
```

输出结果如下：
```
Node [/infantry_serial]
Publications: 
 * /infantry_serial/infantry_enemy_color [infantry_msgs/EnemyColor]
 * /infantry_serial/infantry_gimbal_angle_fdb [infantry_msgs/GimbalFdbAngle]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /infantry_serial/infantry_gimbal_angle_set [unknown type]

Services: 
 * /infantry_serial/get_loggers
 * /infantry_serial/set_logger_level


contacting node http://babyxin-NUC7i7BNH:41789/ ...
Pid: 30309
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
```
可以看见发布的话题中包含`/infantry_serial/infantry_enemy_color`，说明节点编译运行成功。