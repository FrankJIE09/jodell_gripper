from jodellSdk.jodellSdkDemo import RgClawControl
import os
import time

class ClawOperation:
    def __init__(self, com_port, baud_rate):
        """
        初始化手爪操作对象，进行串口连接。

        :param com_port: 串口号（如：/dev/ttyUSB0）
        :param baud_rate: 波特率（如：115200）
        """
        self.com_port = com_port  # 串口号
        self.baud_rate = baud_rate  # 波特率
        self.is_connected = False  # 初始化连接状态
        self.clawControl = RgClawControl()  # 创建 RgClawControl 类的实例，控制夹爪

    def log_result(self, action, result, success_msg, fail_msg):
        """统一处理日志输出"""
        if result == 1:
            print(f"{action} 成功: {success_msg}")  # 打印操作成功的日志
        else:
            print(f"{action} 失败: {fail_msg}")  # 打印操作失败的日志
        return result

    def check_device_exists(self):
        """检查设备文件是否存在"""
        if not os.path.exists(self.com_port):  # 检查设备文件是否存在
            print(f"设备 {self.com_port} 不存在")
            return False  # 如果设备不存在，返回 False
        return True  # 如果设备存在，返回 True

    def connect(self):
        """连接到串口设备"""
        if not self.check_device_exists():  # 如果设备文件不存在，直接返回 False
            return False

        # 通过 serialOperation 方法连接设备，state=True 表示打开串口连接
        self.is_connected = self.clawControl.serialOperation(self.com_port, self.baud_rate, True)
        return self.log_result(
            "连接设备",
            self.is_connected,
            f"成功连接到串口：{self.com_port}",
            f"连接失败：{self.com_port}"
        )

    def disconnect(self):
        """断开与设备的串口连接"""
        if not self.check_device_exists():  # 如果设备文件不存在，直接返回 False
            return False

        # 通过 serialOperation 方法断开设备，state=False 表示关闭串口连接
        self.is_connected = not self.clawControl.serialOperation(self.com_port, self.baud_rate, False)
        return self.log_result(
            "断开设备",
            not self.is_connected,
            f"成功断开串口：{self.com_port}",
            f"断开连接失败：{self.com_port}"
        )

    def check_connection(self):
        """检查连接状态"""
        if not self.is_connected:  # 如果设备未连接
            print("设备未连接，请先连接设备。")
            return False  # 返回 False
        return True  # 返回 True

    def enable_claw(self, claw_id, enable=True):
        """使能或禁用夹爪"""
        if not self.check_connection():  # 如果设备未连接，返回 False
            return False
        # 控制夹爪的使能状态，使用 enableClamp 方法
        flag = self.clawControl.enableClamp(claw_id, enable)  # 假设 enableClamp 是正确的方法
        return self.log_result(
            f"使能夹爪 {claw_id}",
            flag,
            f"夹爪 {claw_id} {'已使能' if enable else '已禁用'}",
            f"夹爪 {claw_id} 使能失败"
        )

    def run_without_param(self, claw_id, command_id):
        """无参数模式运行夹爪"""
        if not self.check_connection():  # 如果设备未连接，返回 False
            return False
        # 运行夹爪，通过 runWithoutParam 方法
        flag = self.clawControl.runWithoutParam(claw_id, command_id)
        return self.log_result(
            f"运行夹爪 {claw_id}",
            flag,
            f"以预设命令编号 {command_id} 执行",
            f"无参数模式执行失败"
        )

    def run_with_param(self, claw_id, force, speed, position,block=True):
        """
        有参数模式运行夹爪，控制夹爪的力矩、速度和位置，直到夹爪到达目标位置。

        :param claw_id: 夹爪 ID
        :param force: 夹爪的力矩（单位：N·m），范围为 0-255
        :param speed: 夹爪的速度（单位：mm/s），范围为 0-255
        :param position: 夹爪的位置（单位：mm），范围为 0-255
        :return: 返回执行结果
        :raises ValueError: 如果 force、speed 或 position 超出指定范围
        """
        # 检查 force、speed 和 position 是否在合法范围内
        if not (0 <= force <= 255):
            raise ValueError("force must be between 0 and 255.")
        if not (0 <= speed <= 255):
            raise ValueError("speed must be between 0 and 255.")
        if not (0 <= position <= 255):
            raise ValueError("position must be between 0 and 255.")

        # 通过 runWithParam 方法运行夹爪，传入力矩、速度和位置参数
        flag = self.clawControl.runWithParam(claw_id, position, speed, force)

        if not flag:
            return False  # 如果执行失败，返回 False

        # 循环检查当前位置，直到达到目标位置
        if block:
            while True:
                current_location = self.get_info(claw_id, "getClampCurrentLocation", "")

                # 假设返回的是一个列表，如 [255]，取列表中的第一个值作为当前位置
                if current_location and current_location[0] == position:
                    break  # 目标位置已到达，跳出循环

                # 可以选择添加一个小的延时，避免过于频繁地查询
                time.sleep(0.1)

        return True

    def get_status(self, claw_id, start_addr, count):
        """查询夹爪状态"""
        if not self.check_connection():  # 如果设备未连接，返回 None
            return None
        # 通过 getStatus 方法获取夹爪的状态
        status_list = self.clawControl.getStatus(claw_id, start_addr, count, 2)
        if isinstance(status_list, list):
            print(f"夹爪 {claw_id} 状态：{status_list}")
            return status_list
        else:
            print(f"夹爪 {claw_id} 状态查询失败")
            return None

    def get_info(self, claw_id, func_name, description):
        """通用方法，用于获取夹爪的各种信息"""
        if not self.check_connection():  # 如果设备未连接，返回 None
            return None
        # 动态调用夹爪相关的查询方法（如：当前位置、当前速度等）
        func = getattr(self.clawControl, func_name)
        info = func(claw_id)
        if isinstance(info, list):
            # print(f"夹爪 {claw_id} {description}：{info}")
            return info
        else:
            # print(f"夹爪 {claw_id} {description}查询失败")
            return None

    def get_current_location(self, claw_id):
        """获取夹爪当前位置"""
        return self.get_info(claw_id, "getClampCurrentLocation", "当前位置")

    def get_current_speed(self, claw_id):
        """获取夹爪当前速度"""
        return self.get_info(claw_id, "getClampCurrentSpeed", "当前速度")

    def get_current_torque(self, claw_id):
        """获取夹爪当前力矩"""
        return self.get_info(claw_id, "getClampCurrentTorque", "当前力矩")


# 示例：如何使用此 API 进行手爪操作
if __name__ == "__main__":
    claw = ClawOperation(com_port="/dev/ttyUSB0", baud_rate=115200)  # 假设串口设备在 /dev/ttyUSB0
    if claw.connect():  # 连接设备
        # 使能夹爪
        claw.enable_claw(claw_id=9, enable=True)
        # 无参数运行夹爪
        # claw.run_without_param(claw_id=9, command_id=2)
        # 获取夹爪当前位置、速度和力矩
        for i in range(100):
            claw.run_with_param(claw_id=9, force=255, speed=255, position=255)
            claw.run_with_param(claw_id=9, force=255, speed=255, position=0)

        print(claw.get_current_location(claw_id=9))
        print(claw.get_current_speed(claw_id=9))
        print(claw.get_current_torque(claw_id=9))
        # 断开连接
        claw.disconnect()
