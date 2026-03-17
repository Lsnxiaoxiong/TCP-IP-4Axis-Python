import socket
import threading
from tkinter import Text, END
import datetime
import numpy as np
import os
import json

alarmControllerFile = "files/alarm_controller.json"
alarmServoFile = "files/alarm_servo.json"

# Port Feedback
MyType = np.dtype([('len', np.int16,),
                   ('Reserve', np.int16, (3,)),
                   ('digital_input_bits', np.int64,),
                   ('digital_outputs', np.int64,),
                   ('robot_mode', np.int64,),
                   ('controller_timer', np.int64,),
                   ('run_time', np.int64,),
                   ('test_value', np.int64,),
                   ('safety_mode', np.float64,),
                   ('speed_scaling', np.float64,),
                   ('linear_momentum_norm', np.float64,),
                   ('v_main', np.float64,),
                   ('v_robot', np.float64,),
                   ('i_robot', np.float64,),
                   ('program_state', np.float64,),
                   ('safety_status', np.float64,),
                   ('tool_accelerometer_values', np.float64, (3,)),
                   ('elbow_position', np.float64, (3,)),
                   ('elbow_velocity', np.float64, (3,)),
                   ('q_target', np.float64, (6,)),
                   ('qd_target', np.float64, (6,)),
                   ('qdd_target', np.float64, (6,)),
                   ('i_target', np.float64, (6,)),
                   ('m_target', np.float64, (6,)),
                   ('q_actual', np.float64, (6,)),
                   ('qd_actual', np.float64, (6,)),
                   ('i_actual', np.float64, (6,)),
                   ('i_control', np.float64, (6,)),
                   ('tool_vector_actual', np.float64, (6,)),
                   ('TCP_speed_actual', np.float64, (6,)),
                   ('TCP_force', np.float64, (6,)),
                   ('Tool_vector_target', np.float64, (6,)),
                   ('TCP_speed_target', np.float64, (6,)),
                   ('motor_temperatures', np.float64, (6,)),
                   ('joint_modes', np.float64, (6,)),
                   ('v_actual', np.float64, (6,)),
                   ('handtype', np.int8, (4,)),
                   ('userCoordinate', np.int8, (1,)),
                   ('toolCoordinate', np.int8, (1,)),
                   ('isRunQueuedCmd', np.int8, (1,)),
                   ('isPauseCmdFlag', np.int8, (1,)),
                   ('velocityRatio', np.int8, (1,)),
                   ('accelerationRatio', np.int8, (1,)),
                   ('jerkRatio', np.int8, (1,)),
                   ('xyzVelocityRatio', np.int8, (1,)),
                   ('rVelocityRatio', np.int8, (1,)),
                   ('xyzAccelerationRatio', np.int8, (1,)),
                   ('rAccelerationRatio', np.int8, (1,)),
                   ('xyzJerkRatio', np.int8, (1,)),
                   ('rJerkRatio', np.int8, (1,)),
                   ('BrakeStatus', np.int8, (1,)),
                   ('EnableStatus', np.int8, (1,)),
                   ('DragStatus', np.int8, (1,)),
                   ('RunningStatus', np.int8, (1,)),
                   ('ErrorStatus', np.int8, (1,)),
                   ('JogStatus', np.int8, (1,)),
                   ('RobotType', np.int8, (1,)),
                   ('DragButtonSignal', np.int8, (1,)),
                   ('EnableButtonSignal', np.int8, (1,)),
                   ('RecordButtonSignal', np.int8, (1,)),
                   ('ReappearButtonSignal', np.int8, (1,)),
                   ('JawButtonSignal', np.int8, (1,)),
                   ('SixForceOnline', np.int8, (1,)),  # 1037
                   ('Reserve2', np.int8, (82,)),
                   ('m_actual[6]', np.float64, (6,)),
                   ('load', np.float64, (1,)),
                   ('centerX', np.float64, (1,)),
                   ('centerY', np.float64, (1,)),
                   ('centerZ', np.float64, (1,)),
                   ('user', np.float64, (6,)),
                   ('tool', np.float64, (6,)),
                   ('traceIndex', np.int64,),
                   ('SixForceValue', np.int64, (6,)),
                   ('TargetQuaternion', np.float64, (4,)),
                   ('ActualQuaternion', np.float64, (4,)),
                   ('Reserve3', np.int8, (24,)),
                   ])


def alarmAlarmJsonFile():
    """
    读取控制器和伺服告警码 JSON 文件

    Returns:
        tuple: (dataController, dataServo) 控制器和伺服告警码数据
    """
    currrntDirectory = os.path.dirname(__file__)
    jsonContrellorPath = os.path.join(currrntDirectory, alarmControllerFile)
    jsonServoPath = os.path.join(currrntDirectory, alarmServoFile)

    with open(jsonContrellorPath, encoding='utf-8') as f:
        dataController = json.load(f)
    with open(jsonServoPath, encoding='utf-8') as f:
        dataServo = json.load(f)
    return dataController, dataServo


class DobotApi:
    """
    Dobot API 基类

    提供与 Dobot 机器人通信的基础 TCP 连接和消息收发功能。
    支持 Dashboard、Move、Feedback 三种端口的连接。

    Attributes:
        ip: 机器人控制器 IP 地址
        port: 连接端口号
        socket_dobot: Socket 连接对象
        text_log: 日志文本控件（可选）
    """
    def __init__(self, ip, port, *args):
        """
        初始化 Dobot API 连接

        Args:
            ip: 机器人控制器 IP 地址
            port: 连接端口号 (29999/30003/30004)
            *args: 可选参数，第一个参数为日志文本控件
        """
        self.ip = ip
        self.port = port
        self.socket_dobot = 0
        self.__globalLock = threading.Lock()
        self.text_log: Text = None
        if args:
            self.text_log = args[0]

        if self.port == 29999 or self.port == 30003 or self.port == 30004:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
            except socket.error:
                print(socket.error)
                raise Exception(
                    f"Unable to set socket connection use port {self.port} !", socket.error)
        else:
            raise Exception(
                f"Connect to dashboard server need use port {self.port} !")

    def log(self, text):
        """
        记录日志信息

        Args:
            text: 要记录的日志文本
        """
        if self.text_log:
            date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S ")
            self.text_log.insert(END, date + text + "\n")
        else:
            print(text)

    def send_data(self, string):
        """
        发送数据到机器人

        Args:
            string: 要发送的命令字符串
        """
        try:
            self.log(f"Send to {self.ip}:{self.port}: {string}")
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            print(e)

    def wait_reply(self):
        """
        等待并接收机器人返回值

        Returns:
            str: 接收到的响应字符串
        """
        data = ""
        try:
            data = self.socket_dobot.recv(1024)
        except Exception as e:
            print(e)

        finally:
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
                self.log(f'Receive from {self.ip}:{self.port}: {data_str}')
            return data_str

    def close(self):
        """
        关闭 socket 连接
        """
        if (self.socket_dobot != 0):
            self.socket_dobot.close()

    def sendRecvMsg(self, string):
        """
        发送并接收同步消息

        Args:
            string: 要发送的命令字符串

        Returns:
            str: 机器人返回的响应
        """
        with self.__globalLock:
            self.send_data(string)
            recvData = self.wait_reply()
            return recvData

    def __del__(self):
        self.close()


class DobotApiDashboard(DobotApi):
    """
    Dobot Dashboard API 类

    继承自 DobotApi，提供机器人控制相关的命令接口。
    包括使能、清除错误、速度设置、坐标系选择等功能。
    """

    def EnableRobot(self, *dynParams):
        """
        使能机器人

        Args:
            *dynParams: 可选动态参数

        Returns:
            str: 机器人响应
        """
        string = "EnableRobot("
        for i in range(len(dynParams)):
            if i == len(dynParams) - 1:
                string = string + str(dynParams[i])
            else:
                string = string + str(dynParams[i]) + ","
        string = string + ")"
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        """
        禁用机器人

        Returns:
            str: 机器人响应
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        """
        清除控制器报警信息

        Returns:
            str: 机器人响应
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def ResetRobot(self):
        """
        机器人停止

        Returns:
            str: 机器人响应
        """
        string = "ResetRobot()"
        return self.sendRecvMsg(string)

    def SpeedFactor(self, speed):
        """
        设置全局速度比例

        Args:
            speed: 速度值 (范围：1~100)

        Returns:
            str: 机器人响应
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def User(self, index):
        """
        选择已标定的用户坐标系

        Args:
            index: 用户坐标系索引

        Returns:
            str: 机器人响应
        """
        string = "User({:d})".format(index)
        return self.sendRecvMsg(string)

    def Tool(self, index):
        """
        选择已标定的工具坐标系

        Args:
            index: 工具坐标系索引

        Returns:
            str: 机器人响应
        """
        string = "Tool({:d})".format(index)
        return self.sendRecvMsg(string)

    def RobotMode(self):
        """
        查看机器人状态

        Returns:
            str: 机器人响应
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PayLoad(self, weight, inertia):
        """
        设置机器人负载

        Args:
            weight: 负载重量
            inertia: 负载转动惯量

        Returns:
            str: 机器人响应
        """
        string = "PayLoad({:f},{:f})".format(weight, inertia)
        return self.sendRecvMsg(string)

    def DO(self, index, status):
        """
        设置数字信号输出状态（队列指令）

        Args:
            index: 数字输出索引 (范围：1~24)
            status: 数字输出状态 (0:低电平，1:高电平)

        Returns:
            str: 机器人响应
        """
        string = "DO({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        设置关节加速度比例（仅对 MovJ、MovJIO、MovJR、JointMovJ 命令有效）

        Args:
            speed: 关节加速度比例 (范围：1~100)

        Returns:
            str: 机器人响应
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        设置坐标系加速度比例（仅对 MovL、MovLIO、MovLR、Jump、Arc、Circle 命令有效）

        Args:
            speed: 笛卡尔加速度比例 (范围：1~100)

        Returns:
            str: 机器人响应
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def SpeedJ(self, speed):
        """
        设置关节速度比例（仅对 MovJ、MovJIO、MovJR、JointMovJ 命令有效）

        Args:
            speed: 关节速度比例 (范围：1~100)

        Returns:
            str: 机器人响应
        """
        string = "SpeedJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def SpeedL(self, speed):
        """
        设置笛卡尔加速度比例（仅对 MovL、MovLIO、MovLR、Jump、Arc、Circle 命令有效）

        Args:
            speed: 笛卡尔加速度比例 (范围：1~100)

        Returns:
            str: 机器人响应
        """
        string = "SpeedL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def Arch(self, index):
        """
        设置 Jump 门型参数索引（包含：起点抬升高度、最大抬升高度、终点下降高度）

        Args:
            index: 参数索引 (范围：0~9)

        Returns:
            str: 机器人响应
        """
        string = "Arch({:d})".format(index)
        return self.sendRecvMsg(string)

    def CP(self, ratio):
        """
        设置平滑过渡比例

        Args:
            ratio: 平滑过渡比例 (范围：1~100)

        Returns:
            str: 机器人响应
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)

    def LimZ(self, value):
        """
        设置门型参数最大抬升高度

        Args:
            value: 最大抬升高度（限制：不超过机械臂 z 轴极限位置）

        Returns:
            str: 机器人响应
        """
        string = "LimZ({:d})".format(value)
        return self.sendRecvMsg(string)

    def RunScript(self, project_name):
        """
    Run the script file
    project_name ：Script file name
    """
        string = "RunScript({:s})".format(project_name)
        return self.sendRecvMsg(string)

    def StopScript(self):
        """
        停止脚本

        Returns:
            str: 机器人响应
        """
        string = "StopScript()"
        return self.sendRecvMsg(string)

    def PauseScript(self):
        """
        暂停脚本

        Returns:
            str: 机器人响应
        """
        string = "PauseScript()"
        return self.sendRecvMsg(string)

    def ContinueScript(self):
        """
        继续运行脚本

        Returns:
            str: 机器人响应
        """
        string = "ContinueScript()"
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, id, addr, count, type=None):
        """
        读取保持寄存器

        Args:
            id: 从设备号（最多支持 5 个从设备，范围 0~4，访问控制器内部从机时设为 0）
            addr: 保持寄存器起始地址 (范围：3095~4095)
            count: 读取指定类型数据的个数 (范围：1~16)
            type: 数据类型
                - 为空时默认读取 16 位无符号整数 (2 字节，占 1 个寄存器)
                - "U16": 读取 16 位无符号整数 (2 字节，占 1 个寄存器)
                - "U32": 读取 32 位无符号整数 (4 字节，占 2 个寄存器)
                - "F32": 读取 32 位单精度浮点数 (4 字节，占 2 个寄存器)
                - "F64": 读取 64 位双精度浮点数 (8 字节，占 4 个寄存器)

        Returns:
            str: 机器人响应
        """
        if type is not None:
            string = "GetHoldRegs({:d},{:d},{:d},{:s})".format(
                id, addr, count, type)
        else:
            string = "GetHoldRegs({:d},{:d},{:d})".format(
                id, addr, count)
        return self.sendRecvMsg(string)

    def SetHoldRegs(self, id, addr, count, table, type=None):
        """
        写入保持寄存器

        Args:
            id: 从设备号（最多支持 5 个从设备，范围 0~4，访问控制器内部从机时设为 0）
            addr: 保持寄存器起始地址 (范围：3095~4095)
            count: 写入指定类型数据的个数 (范围：1~16)
            table: 数据表
            type: 数据类型
                - 为空时默认写入 16 位无符号整数 (2 字节，占 1 个寄存器)
                - "U16": 16 位无符号整数 (2 字节，占 1 个寄存器)
                - "U32": 32 位无符号整数 (4 字节，占 2 个寄存器)
                - "F32": 32 位单精度浮点数 (4 字节，占 2 个寄存器)
                - "F64": 64 位双精度浮点数 (8 字节，占 4 个寄存器)

        Returns:
            str: 机器人响应
        """
        if type is not None:
            string = "SetHoldRegs({:d},{:d},{:d},{:d})".format(
                id, addr, count, table)
        else:
            string = "SetHoldRegs({:d},{:d},{:d},{:d},{:s})".format(
                id, addr, count, table, type)
        return self.sendRecvMsg(string)

    def GetErrorID(self):
        """
        获取机器人错误码

        Returns:
            str: 机器人响应
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)

    def DOExecute(self, offset1, offset2):
        """
        执行 DO 输出

        Args:
            offset1: 输出索引
            offset2: 输出状态

        Returns:
            str: 机器人响应
        """
        string = "DOExecute({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def ToolDO(self, offset1, offset2):
        """
        工具数字输出

        Args:
            offset1: 输出索引
            offset2: 输出状态

        Returns:
            str: 机器人响应
        """
        string = "ToolDO({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def ToolDOExecute(self, offset1, offset2):
        """
        执行工具 DO 输出

        Args:
            offset1: 输出索引
            offset2: 输出状态

        Returns:
            str: 机器人响应
        """
        string = "ToolDOExecute({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def SetArmOrientation(self, offset1):
        """
        设置机械臂方位

        Args:
            offset1: 方位参数

        Returns:
            str: 机器人响应
        """
        string = "SetArmOrientation({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def SetPayload(self, offset1, *dynParams):
        string = "SetPayload({:f}".format(
            offset1)
        for params in dynParams:
            string = string + "," + str(params) + ","
        string = string + ")"
        return self.sendRecvMsg(string)

    def PositiveSolution(self, offset1, offset2, offset3, offset4, user, tool):
        """
        正解计算

        Args:
            offset1: 关节角 1
            offset2: 关节角 2
            offset3: 关节角 3
            offset4: 关节角 4
            user: 用户坐标系索引
            tool: 工具坐标系索引

        Returns:
            str: 机器人响应
        """
        string = "PositiveSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(offset1, offset2, offset3, offset4, user,
                                                                         tool) + ")"
        return self.sendRecvMsg(string)

    def InverseSolution(self, offset1, offset2, offset3, offset4, user, tool, *dynParams):
        string = "InverseSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(offset1, offset2, offset3, offset4, user, tool)
        for params in dynParams:
            print(type(params), params)
            string = string + repr(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self, offset1):
        """
        设置碰撞等级

        Args:
            offset1: 碰撞等级 (1-5)

        Returns:
            str: 机器人响应
        """
        string = "SetCollisionLevel({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def GetAngle(self):
        """
        获取关节角度

        Returns:
            str: 机器人响应
        """
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def GetPose(self):
        """
        获取机械臂位姿

        Returns:
            str: 机器人响应
        """
        string = "GetPose()"
        return self.sendRecvMsg(string)

    def EmergencyStop(self):
        """
        紧急停止

        Returns:
            str: 机器人响应
        """
        string = "EmergencyStop()"
        return self.sendRecvMsg(string)

    def ModbusCreate(self, ip, port, slave_id, isRTU):
        """
        创建 Modbus 连接

        Args:
            ip: Modbus 设备 IP 地址
            port: Modbus 端口
            slave_id: 从站 ID
            isRTU: 是否为 RTU 模式 (1:RTU, 0:TCP)

        Returns:
            str: 机器人响应
        """
        string = "ModbusCreate({:s},{:d},{:d},{:d}".format(ip, port, slave_id, isRTU) + ")"
        return self.sendRecvMsg(string)

    def ModbusClose(self, offset1):
        """
        关闭 Modbus 连接

        Args:
            offset1: Modbus 索引

        Returns:
            str: 机器人响应
        """
        string = "ModbusClose({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def GetInBits(self, offset1, offset2, offset3):
        """
        读取输入位

        Args:
            offset1: Modbus 索引
            offset2: 起始地址
            offset3: 读取数量

        Returns:
            str: 机器人响应
        """
        string = "GetInBits({:d},{:d},{:d}".format(offset1, offset2, offset3) + ")"
        return self.sendRecvMsg(string)

    def GetInRegs(self, offset1, offset2, offset3, *dynParams):
        string = "GetInRegs({:d},{:d},{:d}".format(offset1, offset2, offset3)
        for params in dynParams:
            print(type(params), params)
            string = string + params[0]
        string = string + ")"
        return self.sendRecvMsg(string)

    def GetCoils(self, offset1, offset2, offset3):
        """
        读取线圈

        Args:
            offset1: Modbus 索引
            offset2: 起始地址
            offset3: 读取数量

        Returns:
            str: 机器人响应
        """
        string = "GetCoils({:d},{:d},{:d}".format(offset1, offset2, offset3) + ")"
        return self.sendRecvMsg(string)

    def SetCoils(self, offset1, offset2, offset3, offset4):
        """
        设置线圈状态

        Args:
            offset1: Modbus 索引
            offset2: 起始地址
            offset3: 写入数量
            offset4: 线圈状态

        Returns:
            str: 机器人响应
        """
        string = "SetCoils({:d},{:d},{:d}".format(offset1, offset2, offset3) + "," + repr(offset4) + ")"
        print(str(offset4))
        return self.sendRecvMsg(string)

    def DI(self, offset1):
        """
        读取数字输入

        Args:
            offset1: 输入索引

        Returns:
            str: 机器人响应
        """
        string = "DI({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def ToolDI(self, offset1):
        """
        读取工具数字输入

        Args:
            offset1: 输入索引

        Returns:
            str: 机器人响应
        """
        string = "DI({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def DOGroup(self, *dynParams):
        string = "DOGroup("
        for params in dynParams:
            string = string + str(params) + ","
        string = string + ")"
        return self.wait_reply()

    def BrakeControl(self, offset1, offset2):
        """
        制动器控制

        Args:
            offset1: 制动器索引
            offset2: 控制状态

        Returns:
            str: 机器人响应
        """
        string = "BrakeControl({:d},{:d}".format(offset1, offset2) + ")"
        return self.sendRecvMsg(string)

    def StartDrag(self):
        """
        开始拖拽示教

        Returns:
            str: 机器人响应
        """
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        """
        停止拖拽示教

        Returns:
            str: 机器人响应
        """
        string = "StopDrag()"
        return self.sendRecvMsg(string)

    def LoadSwitch(self, offset1):
        """
        负载开关控制

        Args:
            offset1: 开关状态

        Returns:
            str: 机器人响应
        """
        string = "LoadSwitch({:d}".format(offset1) + ")"
        return self.sendRecvMsg(string)

    def wait(self,t):
        string = "wait({:d}".format(t)+")"
        return self.sendRecvMsg(string)

    def pause(self):
        string = "pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        string = "continue()"
        return self.sendRecvMsg(string)


class DobotApiMove(DobotApi):
    """
    Dobot Move API 类

    继承自 DobotApi，提供机器人运动控制相关的命令接口。
    包括点到点运动、直线运动、圆弧运动等功能。
    """

    def MovJ(self, x, y, z, r, *dynParams):
        """
    Joint motion interface (point-to-point motion mode)
    x: A number in the Cartesian coordinate system x
    y: A number in the Cartesian coordinate system y
    z: A number in the Cartesian coordinate system z
    r: A number in the Cartesian coordinate system R
    """
        string = "MovJ({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def MovL(self, x, y, z, r, *dynParams):
        """
        直线运动 (线性运动模式)

        Args:
            x: 笛卡尔坐标系 X 坐标
            y: 笛卡尔坐标系 Y 坐标
            z: 笛卡尔坐标系 Z 坐标
            r: 笛卡尔坐标系 R 坐标
            *dynParams: 动态参数

        Returns:
            str: 机器人响应
        """
        string = "MovL({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def JointMovJ(self, j1, j2, j3, j4, *dynParams):
        """
            关节运动 (线性运动模式)

        Args:
            j1: 关节 1 位置
            j2: 关节 2 位置
            j3: 关节 3 位置
            j4: 关节 4 位置
            *dynParams: 动态参数

        Returns:
            str: 机器人响应
        """
        string = "JointMovJ({:f},{:f},{:f},{:f}".format(
            j1, j2, j3, j4)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def Jump(self):
        """
        门型跃迁运动（待实现）

        Note: 此方法当前仅打印提示信息，尚未实现
        """
        print("待定")

    def RelMovJ(self, x, y, z, r, *dynParams):
        """
            偏移运动 (点到点运动模式)

        Args:
            x: X 轴偏移量
            y: Y 轴偏移量
            z: Z 轴偏移量
            r: R 轴偏移量
            *dynParams: 动态参数

        Returns:
            str: 机器人响应
        """
        string = "RelMovJ({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelMovL(self, offsetX, offsetY, offsetZ, offsetR, *dynParams):
        """
            偏移运动 (直线运动模式)

        Args:
            offsetX: X 轴偏移量
            offsetY: Y 轴偏移量
            offsetZ: Z 轴偏移量
            offsetR: R 轴偏移量
            *dynParams: 动态参数

        Returns:
            str: 机器人响应
        """
        string = "RelMovL({:f},{:f},{:f},{:f}".format(offsetX, offsetY, offsetZ, offsetR)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MovLIO(self, x, y, z, r, *dynParams):
        """
    Set the digital output port state in parallel while moving in a straight line
    x: A number in the Cartesian coordinate system x
    y: A number in the Cartesian coordinate system y
    z: A number in the Cartesian coordinate system z
    r: A number in the Cartesian coordinate system r
    *dynParams :Parameter Settings（Mode、Distance、Index、Status）
                Mode :Set Distance mode (0: Distance percentage; 1: distance from starting point or target point)
                Distance :Runs the specified distance（If Mode is 0, the value ranges from 0 to 100；When Mode is 1, if the value is positive,
                         it indicates the distance from the starting point. If the value of Distance is negative, it represents the Distance from the target point）
                Index ：Digital output index （Value range：1~24）
                Status ：Digital output state（Value range：0/1）
    """
        # example： MovLIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovLIO({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MovJIO(self, x, y, z, r, *dynParams):
        """
    Set the digital output port state in parallel during point-to-point motion
    x: A number in the Cartesian coordinate system x
    y: A number in the Cartesian coordinate system y
    z: A number in the Cartesian coordinate system z
    r: A number in the Cartesian coordinate system r
    *dynParams :Parameter Settings（Mode、Distance、Index、Status）
                Mode :Set Distance mode (0: Distance percentage; 1: distance from starting point or target point)
                Distance :Runs the specified distance（If Mode is 0, the value ranges from 0 to 100；When Mode is 1, if the value is positive,
                         it indicates the distance from the starting point. If the value of Distance is negative, it represents the Distance from the target point）
                Index ：Digital output index （Value range：1~24）
                Status ：Digital output state（Value range：0/1）
    """
        # example： MovJIO(0,50,0,0,0,0,(0,50,1,0),(1,1,2,1))
        string = "MovJIO({:f},{:f},{:f},{:f}".format(
            x, y, z, r)
        self.log("Send to 192.168.1.6:29999:" + string)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def Arc(self, x1, y1, z1, r1, x2, y2, z2, r2, *dynParams):
        """
        圆弧运动指令

        Args:
            x1, y1, z1, r1: 中间点坐标值
            x2, y2, z2, r2: 终点坐标值
            *dynParams: 动态参数

        Note: 此指令应与其他运动指令配合使用

        Returns:
            str: 机器人响应
        """
        string = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f}".format(
            x1, y1, z1, r1, x2, y2, z2, r2)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        print(string)
        return self.sendRecvMsg(string)

    def Circle(self, x1, y1, z1, r1, x2, y2, z2, r2, count, *dynParams):
        """
        整圆运动指令

        Args:
            x1, y1, z1, r1: 中间点坐标值
            x2, y2, z2, r2: 终点坐标值
            count: 运行圈数
            *dynParams: 动态参数

        Note: 此指令应与其他运动指令配合使用

        Returns:
            str: 机器人响应
        """
        string = "Circle({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(
            x1, y1, z1, r1, x2, y2, z2, r2, count)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MoveJog(self, axis_id=None, *dynParams):
        """
        关节点动

        Args:
            axis_id: 关节运动轴，可选值：
                J1+ J2+ J3+ J4+ J5+ J6+
                J1- J2- J3- J4- J5- J6-
                X+ Y+ Z+ Rx+ Ry+ Rz+
                X- Y- Z- Rx- Ry- Rz-
            *dynParams: 参数设置 (coord_type, user_index, tool_index)
                - coord_type: 坐标类型 (1: 用户坐标，2: 工具坐标，默认 1)
                - user_index: 用户坐标系索引 (0~9，默认 0)
                - tool_index: 工具坐标系索引 (0~9，默认 0)

        Returns:
            str: 机器人响应
        """
        if axis_id is not None:
            string = "MoveJog({:s}".format(axis_id)
        else:
            string = "MoveJog("
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def Sync(self):
        """
        同步等待

        阻塞程序执行，等待所有队列指令执行完成后返回。

        Returns:
            str: 机器人响应
        """
        string = "Sync()"
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_r, user, *dynParams):
        """
        沿用户坐标系的相对运动（关节运动模式）

        Args:
            offset_x: X 轴方向偏移量
            offset_y: Y 轴方向偏移量
            offset_z: Z 轴方向偏移量
            offset_r: R 轴方向偏移量
            user: 用户坐标系索引 (范围：0~9)
            *dynParams: 参数设置 (speed_j, acc_j, tool)
                - speed_j: 关节速度比例 (范围：1~100)
                - acc_j: 关节加速度比例 (范围：1~100)
                - tool: 工具坐标系索引

        Returns:
            str: 机器人响应
        """
        string = "RelMovJUser({:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_r, user)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_r, user, *dynParams):
        """
        沿用户坐标系的相对运动（直线运动模式）

        Args:
            offset_x: X 轴方向偏移量
            offset_y: Y 轴方向偏移量
            offset_z: Z 轴方向偏移量
            offset_r: R 轴方向偏移量
            user: 用户坐标系索引 (范围：0~9)
            *dynParams: 参数设置 (speed_l, acc_l, tool)
                - speed_l: 笛卡尔速度比例 (范围：1~100)
                - acc_l: 笛卡尔加速度比例 (范围：1~100)
                - tool: 工具坐标系索引

        Returns:
            str: 机器人响应
        """
        string = "RelMovLUser({:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_r, user)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def RelJointMovJ(self, offset1, offset2, offset3, offset4, *dynParams):
        """
        沿各轴关节坐标系的相对运动（关节运动模式）

        Args:
            offset1: 关节 1 偏移量
            offset2: 关节 2 偏移量
            offset3: 关节 3 偏移量
            offset4: 关节 4 偏移量
            *dynParams: 参数设置 (speed_j, acc_j, user)
                - speed_j: 关节速度比例 (范围：1~100)
                - acc_j: 关节加速度比例 (范围：1~100)
                - user: 用户坐标系索引

        Returns:
            str: 机器人响应
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f}".format(
            offset1, offset2, offset3, offset4)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def MovJExt(self, offset1, *dynParams):
        """
        外部轴关节运动

        Args:
            offset1: 外部轴位置
            *dynParams: 动态参数

        Returns:
            str: 机器人响应
        """
        string = "MovJExt({:f}".format(
            offset1)
        for params in dynParams:
            string = string + "," + str(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def SyncAll(self):
        """
        同步所有运动

        Returns:
            str: 机器人响应
        """
        string = "SyncAll()"
        return self.sendRecvMsg(string)

