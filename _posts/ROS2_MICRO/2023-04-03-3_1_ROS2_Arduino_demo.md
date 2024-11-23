---
title: 3 ROS2 和 arduino 通讯
category: ROS2_MICRO
layout: post
---

# 3.1 基本介绍

在完成了上面两个简单的 arduino 的例子之后 也能用 python 从串口拿到消息，然后做 ROS2 消息的转发

![main_concept](/assets/pics/ROS2_MICRO/3_main_concept.png)

接下来我们就可以对拿到的消息做处理

处理主要:

- Python 从串口中读取消息的功能模块
- Python 发送消息给到 ROS2 的节点的格式(Twist)
- 有了上面两个辅助的模块，就可以创建一个 ROS2 的节点，在不断的从串口获取数据，然后解析数据转成控制 ROS2 turtlesim 的消息

![ros2_arduino_concept](/assets/pics/ROS2_MICRO/3_ros2_arduino_concept.png)

那么 我们的文件夹结构就是这样的

```shell
ros2_arduino_demo/
|-- main.py                 // 主要的控制逻辑和ROS2节点
|-- move_instruction.py     // 辅助生成控制消息的模块，简单的模拟管理机器人是不是要急停
`-- read_serial.py          // 连接串口和读取串口信息

1 directory, 3 files

```

# 3.2 实现的效果

遥控器控制机器人移动

距离传感器报警之后机器人急停，解除之后恢复

![demo](/assets/pics/ROS2_MICRO/3_demo.jpg)

# 3.3 处理串口消息的模块 read_serial.py

废话不多说 直接看代码 这个比较简单

```py
'''
利用pyserial从串口中获得消息,转成指令
'''

import serial


class ReadSerial:
    def __init__(self) -> None:
        # 串口的句柄
        self.ser = serial.Serial()

    def open_serial(self, port, baudrate) -> bool:
        '''连接打开串口'''
        try:
            # 打开串口
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port} at {baudrate} baud.")
            return True
        except serial.SerialException as e:
            print(f"Error: {e}")
            return False

    def read_from_serial(self) -> str:
        '''读取串口的消息'''
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line != None:
                # 处理数据
                return line
        except Exception as e:
            print(f"Read data from serial error {e}")
            return ''

```

# 3.4 发送 ROS2 cmd_vel 运动指令的模块 move_instruction.py

这个也比较简单 直接看代码

```py
'''
接收指令控制机器人移动,也就是根据某个字符发送对应的运动指令
'''

from geometry_msgs.msg import Twist


class MoveInstruction():
    def __init__(self):
        # 急停的标志
        self.is_estop = False
        # 收到消息要发送的行为
        self.twist = Twist()

    def move_msg(self, key) -> Twist:
        # 急停不做任何操作
        if self.is_estop:
            return self.e_stop()

        if key == 'w' or key == '2':  # 向前移动
            self.twist.linear.x = 2.0
            self.twist.angular.z = 0.0
        elif key == 'x' or key == '8':  # 向后移动
            self.twist.linear.x = -2.0
            self.twist.angular.z = 0.0
        elif key == 'd' or key == '6':  # 向右转
            self.twist.linear.x = 0.0
            self.twist.angular.z = -2.0
        elif key == 'a' or key == '4':  # 向左转
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0
        elif key == 'q' or key == '1':  # 左上旋转
            self.twist.linear.x = 2.0
            self.twist.angular.z = 2.0
        elif key == 'e' or key == '3':  # 右上旋转
            self.twist.linear.x = 2.0
            self.twist.angular.z = -2.0
        elif key == 'z' or key == '7':  # 左下旋转
            self.twist.linear.x = -2.0
            self.twist.angular.z = 2.0
        elif key == 'c' or key == '9':  # 右下旋转
            self.twist.linear.x = -2.0
            self.twist.angular.z = -2.0
        elif key == 's' or key == '5':    # 停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:               # 无效指令也停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        return self.twist

    def e_stop(self):
        '''急停状态全置0, 设置flag'''
        self.is_estop = True

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        return self.twist

    def resume(self):
        '''退出estop状态'''
        if self.is_estop:
            self.is_estop = False

```

唯一值得一提的就是 geometry_msgs.msg.Twist 消息是 ROS 中用于表示速度的消息类型，它包含两个主要部分：线速度（linear）和角速度（angular）。这两个部分都是 geometry_msgs.Vector3 类型。Twist 消息包含以下参数：

- linear：线速度（geometry_msgs.Vector3 类型）
  - x：沿 x 轴的线速度
  - y：沿 y 轴的线速度
  - z：沿 z 轴的线速度
- angular：角速度（geometry_msgs.Vector3 类型）
  - x：绕 x 轴的角速度
  - y：绕 y 轴的角速度
  - z：绕 z 轴的角速度

在 TurtleSim 通常只会用到 linear.x 和 angular.z，因为乌龟只在 2D 平面上移动。

# 3.5 ROS2 节点 main.py

这个才是重中之重, 主要的实现也就如下所示

![ros2_arduino_core_graph](/assets/pics/ROS2_MICRO/3_ros2_arduino_core_graph.png)

那么这时候看代码就清楚了

```py
'''
处理串口消息，增加各种逻辑，创建控制机器人运动的节点
'''
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from move_instruction import MoveInstruction
from read_serial import ReadSerial

# 定义几个常量
NODE_NAME = 'ros2_arduino_demo'
PUB_TOPIC = '/turtle1/cmd_vel'
SER_PORT_NAME = '/dev/ttyUSB0'
SER_BAUD_RATE = 57600

# 约定的串口消息的分隔符
SPLIT_SYMBOL = '='
# 红外传感器串口消息的关键字
IRREMOTE_KEY = 'MOVE'
# 声波传感器的关键字
ESTOP_KEY = 'ESTOP'


class ROS2ArduinoDemo(Node):
    def __init__(self):
        # ROS2的基本节点信息
        super().__init__(NODE_NAME)
        self.publisher_ = self.create_publisher(Twist, PUB_TOPIC, 10)

        # 读取传感器和发送消息的句柄
        self.ins_handler = MoveInstruction()
        self.ser_handler = ReadSerial()

    def run(self):
        '''开始执行ROS2节点'''
        # 连接串口
        flag = self.ser_handler.open_serial(SER_PORT_NAME, SER_BAUD_RATE)

        if not flag:
            print("exit ... ...")
            exit(0)

        # 要发送的控制消息
        twist = Twist()

        # 开始循环接收串口消息，用串口的消息来控制机器人
        while True:
            ser_data_str = self.ser_handler.read_from_serial()

            # 无效信息
            if ser_data_str == '':
                continue

            # 字符串处理
            ser_data_arr = ser_data_str.split(SPLIT_SYMBOL)

            # 数组长度不为2 也是无效信息
            if len(ser_data_arr) != 2:
                continue

            if ser_data_arr[0] == ESTOP_KEY:
                # 急停
                if ser_data_arr[1] == '1':
                    twist = self.ins_handler.e_stop()
                    self.publisher_.publish(twist)
                    print("stop!")
                else:
                    # 退出急停
                    self.ins_handler.resume()
            # 处理红外的信号控制运行
            elif ser_data_arr[0] == IRREMOTE_KEY:
                twist = self.ins_handler.move_msg(ser_data_arr[1])
                self.publisher_.publish(twist)
            else:
                # 其他消息无操作
                pass


if __name__ == '__main__':
    rclpy.init(args=None)
    node = ROS2ArduinoDemo()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

```

# 3.6 总结

用串口的方式处理 arduino 和 ROS2 的通讯，这种模式可以适用到其他的单片机上

控制机器人的运动在考虑急停时候也就是考虑不让 `cmd_vel` 发送有效速度信息即可

后面有更复杂的在导航过程的通讯，可以用 `action` 来做, 这里不介绍了，用到再看
