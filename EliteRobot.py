import socket
import json
import time
import cv2
import numpy as np
import ast
from Jodell_gripper import Gripper


class eliteRobot:
    def __init__(self, ip, port=8055):
        self.ip = ip
        self.port = port
        self.gripper = Gripper()
        self.stop = False
        con, self.sock = self.connectETController()

        if con:
            print("Connected to ET Controller")
        else:
            print("Failed to connect to ET Controller")

    def connectETController(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((self.ip, self.port))
            return (True, sock)
        except Exception as e:
            sock.close()
            return (False, None)

    def open_tci(self):
        suc, result, _ = self.sendCMD("open_tci")
        return result

    def set_tci(self, baud_rate=9600, bits=8, event="N", stop=1):
        suc, result, _ = self.sendCMD("setopt_tci",
                                      {"baud_rate": baud_rate, "bits": bits, "event": event, "stop": stop})
        return result

    def recv_tci(self, count=100, hex=0, timeout=200):
        suc, result, _ = self.sendCMD("recv_tci", {"count": count, "hex": hex, "timeout": timeout})
        return result

    def send_tci(self, send_buf, hex=1):
        # send_buf_string = str(send_buf)
        suc, result, _ = self.sendCMD("send_tci", {"send_buf": send_buf, "hex": hex})
        print(result)
        return suc, result

    def flush_tci(self):
        suc, result, _ = self.sendCMD("flush_tci")
        return result

    def close_tci(self):
        suc, result, _ = self.sendCMD("close_tci")
        return result

    def connect_gripper(self):
        self.open_tci()

        result = self.set_tci(self.gripper.serial_params["baud_rate"], self.gripper.serial_params["data_bits"],
                              self.gripper.serial_params["event"], self.gripper.serial_params["stop_bits"])
        # 清空寄存器
        self.flush_tci()
        # 发送激活指令
        suc, result = self.send_tci(self.gripper.activate_request(), 1)

        # 接收回复
        received = self.recv_tci(100, 1, 100000)

        print(received)
        self.flush_tci()
        received1 = self.recv_tci(100, 1, 200)
        # 清空缓存
        self.flush_tci()

        result = self.send_tci(self.gripper.enable_gripper())
        received = self.recv_tci()

        self.flush_tci()
        function_code, received = self.gripper.decode_response(received)
        while True:
            # 读状态请求
            self.send_tci(self.gripper.read_gripper_state(3))
            # 接收回复
            received = self.recv_tci()
            # 粗解码
            function_code, received = self.gripper.decode_response(received)
            # 清空缓存
            self.flush_tci()
            # 状态解码
            activate_state, _, _, _, error_state, _, _ = self.gripper.decode_state_data(received)
            if activate_state == 3:
                print("使能成功")
                break
            if error_state != 0:
                print("使能失败！错误码：", error_state)
                break
        return

    def run_gripper(self, target_position=0x00, force=100, speed=100):
        # 首先清空缓存
        self.flush_tci()
        # 发送打开夹爪指令
        self.send_tci(self.gripper.run_gripper(target_position, force, speed))
        # 接收指令回复
        received = self.recv_tci()
        self.flush_tci()
        while True:
            self.send_tci(self.gripper.read_gripper_state(3))
            received = self.recv_tci()
            self.flush_tci()
            function_code, received = self.gripper.decode_response(received)
            data_length = received[0]
            move_state, _, current_position, error_state, _, _ = self.gripper.decode_state_data(data_length, received)
            if error_state != 0:
                print("error occurred! error state:", error_state)
                break
            if not move_state:
                print(f"运动已停止当前位置为：{current_position}")
                break
            else:
                continue
        return

    def open_gripper(self):
        self.run_gripper(00, 255, 255)
        return

    def close_gripper(self):
        self.run_gripper(255, 255, 255)
        return

    def disconnectETController(self):
        if self.sock:
            self.sock.close()
        return

    def servo_enable(self):
        suc, servo_status, _ = self.sendCMD("getServoStatus")
        if suc:
            print("Current Servo Status:", servo_status)
            # 如果伺服没有启用，设置伺服为开启状态
            if servo_status == "false" or servo_status == "0":
                print("Enabling servo...")
                suc, result, _ = self.sendCMD("set_servo_status", {"status": 1})  # 1表示启用伺服
                if suc:
                    print("Servo enabled successfully.")
                else:
                    print("Failed to enable servo.")
                    return
            else:
                print("Servo is already enabled.")
                return

    def sendCMD(self, cmd, params=None, id=1):
        if not params:
            params = []
        else:
            params = json.dumps(params)

        sendStr = "{{\"jsonrpc\":\"2.0\",\"method\":\"{0}\",\"params\":{1},\"id\":{2}}}".format(cmd, params, id) + "\n"
        print(sendStr)
        try:
            self.sock.sendall(bytes(sendStr, "utf-8"))
            ret = self.sock.recv(1024)
            jdata = json.loads(str(ret, "utf-8"))
            if "result" in jdata.keys():
                return (True, jdata["result"], jdata["id"])
            elif "error" in jdata.keys():
                return (False, jdata["error"], jdata["id"])
            else:
                return (False, None, None)
        except Exception as e:
            return (False, None, None)

    def getServoStatus(self):
        _, servo_status, _ = self.sendCMD("getServoStatus")
        return servo_status

    def setServoStatus(self, status):
        return self.sendCMD("set_servo_status", {"status": status})

    def syncMotorStatus(self):
        return self.sendCMD("syncMotorStatus")

    def clearAlarm(self):
        return self.sendCMD("clearAlarm")

    def getMotorStatus(self):
        return self.sendCMD("getMotorStatus")

    ### ParamService APIs ###
    def getRobotState(self):
        return self.sendCMD("getRobotState")

    def getRobotMode(self):
        return self.sendCMD("getRobotMode")

    def getJointPos(self):
        return self.sendCMD("get_joint_pos")

    def getTcpPose(self, coordinate_num=-1, tool_num=-1, unit_type=1):
        params = {"coordinate_num": coordinate_num, "tool_num": tool_num, "unit_type": unit_type}
        pose_info = self.sendCMD("get_tcp_pose", params)
        pose_info_str = pose_info[1]
        array = ast.literal_eval(pose_info_str)
        if array:
            return array
        else:
            return None  # 如果没有找到方括号内容，返回 None

    def getMotorSpeed(self):
        return self.sendCMD("get_motor_speed")

    def getCurrentCoord(self):
        return self.sendCMD("getCurrentCoord")

    def getCycleMode(self):
        return self.sendCMD("getCycleMode")

    def getCurrentJobLine(self):
        return self.sendCMD("getCurrentJobLine")

    def getCurrentEncode(self):
        return self.sendCMD("getCurrentEncode")

    def getToolNumber(self):
        return self.sendCMD("getToolNumber")

    def setToolNumber(self, tool_num):
        return self.sendCMD("setToolNumber", {"tool_num": tool_num})

    def getUserNumber(self):
        return self.sendCMD("getUserNumber")

    def setUserNumber(self, user_num):
        return self.sendCMD("setUserNumber", {"user_num": user_num})

    def getMotorTorque(self):
        return self.sendCMD("get_motor_torque")

    def getPathPointIndex(self):
        return self.sendCMD("getPathPointIndex")

    def setCurrentCoord(self, coord_mode):
        return self.sendCMD("setCurrentCoord", {"coord_mode": coord_mode})

    def moveByJoint(self, targetPos, speed=10, block=True):
        # 发送目标位置和速度的命令
        # 发送关节目标位置
        suc, result, _ = self.sendCMD("moveByJoint", {"targetPos": targetPos, "speed": speed})
        if suc:
            print(f"Move command sent: Target position {targetPos} with speed {speed}")

            if block:
                # 如果是阻塞模式，持续检查机器人状态
                while True:
                    suc, state, _ = self.getRobotState()
                    if suc and state == '0':  # 0表示机器人已停止（或已到达目标）
                        print("Robot reached the target position.")
                        break
                    else:
                        print("1Robot is still moving...")
                        time.sleep(0.1)  # 每1秒检查一次，避免过多的请求
        else:
            print("Failed to send move command.")

    def moveByLineBaseCoord(self, targetPos, speed=10):
        print(targetPos)
        suc, result, _ = self.sendCMD("moveByLineCoord", {
            "targetUserPose": targetPos,
            "speed_type": 0,
            "speed": speed,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1
        })
        if suc:
            print(f"Line movement command sent to {targetPos}")

            # 阻塞等待直到机器人到达目标点
            while True:
                suc, result, _ = self.sendCMD("getRobotState")
                if suc and result == "0":  # 0表示机器人已停止
                    print(f"Robot reached target point: {targetPos}")
                    break
                else:
                    print("Robot is still moving...")
                    time.sleep(0.1)  # 每秒检查一次状态
            return
        else:
            print(f"Failed to send linecoord movement command to {targetPos}")

    def moveByLineTCPCoord(self, targetPos, speed=10):
        print(targetPos)
        current_Tcp = self.getTcpPose()
        suc, result, _ = self.sendCMD("moveByLineCoord", {
            "targetUserPose": targetPos,
            "speed_type": 0,
            "speed": speed,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1,
            "user_coord": current_Tcp
        })
        if suc:
            print(f"Line movement command sent to {targetPos}")

            # 阻塞等待直到机器人到达目标点
            while True:
                suc, result, _ = self.sendCMD("getRobotState")
                if suc and result == "0":  # 0表示机器人已停止
                    print(f"Robot reached target point: {targetPos}")
                    break
                else:
                    print("Robot is still moving...")
                    time.sleep(0.1)  # 每秒检查一次状态
            return
        else:
            print(f"Failed to send linecoord movement command to {targetPos}")

    def send_moveByLineTCPCoord_CMD(self, targetPos, speed=10):
        print(targetPos)
        current_Tcp = self.getTcpPose()
        suc, result, _ = self.sendCMD("moveByLineCoord", {
            "targetUserPose": targetPos,
            "speed_type": 0,
            "speed": speed,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1,
            "user_coord": current_Tcp
        })
        return suc, result

    def stop_move(self):
        suc, result, _ = self.sendCMD("stop")

        if suc:
            print(f"Stop command sent")
            while True:
                suc, result, _ = self.sendCMD("stop")
                print(result)
                if result == "true":
                    print("Robot stopped")
                    break
                else:
                    print("Robot is still moving...")
                    time.sleep(0.1)  # 每秒检查一次状态
        else:
            print(f"Failed to send stop command")
        return

    def moveByLine(self, point, speed=10):
        print(type(point))
        # 发送直线运动命令
        suc, result, _ = self.sendCMD("moveByLine", {
            "targetPos": point,
            "speed_type": 0,
            "speed": speed,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1
        })

        if suc:
            print(f"Line movement command sent to {point}")

            # 阻塞等待直到机器人到达目标点
            while True:
                suc, result, _ = self.sendCMD("getRobotState")
                if suc and result == "0":  # 0表示机器人已停止
                    print(f"Robot reached target point: {point}")
                    break
                else:
                    print("Robot is still moving...")
                    time.sleep(0.1)  # 每秒检查一次状态
        else:
            print(f"Failed to send line movement command to {point}")

    def get_all_dh(self, num_parameters):
        all_dh_parameters = []

        for index in range(num_parameters):
            ret, result, id = self.sendCMD("getDH", {"index": index})
            if ret:
                print(f"DH参数 (index {index}) = {result}")
                all_dh_parameters.append(result)
            else:
                print(f"获取DH参数失败 (index {index})，错误信息: {result.get('message', '未知错误')}")

        return all_dh_parameters

    def moveByPath(self):
        return self.sendCMD("moveByPath")

    def clearPathPoint(self):
        return self.sendCMD("clearPathPoint")

    def addPathPoint(self, wayPoint, moveType=0, speed=50, circular_radius=0):
        params = {"wayPoint": wayPoint, "moveType": moveType, "speed": speed, "circular_radius": circular_radius}
        return self.sendCMD("addPathPoint", params)

    def moveBySpeedl(self, speed_l, acc, arot, t, id=1):
        params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
        return self.sendCMD("moveBySpeedl", params, id)

    def moveBySpeedj(self, speed_j, acc=20, t=0.01, id=1):
        params = {"vj": speed_j, "acc": acc, "t": t}
        return self.sendCMD("moveBySpeedj", params, id)

    def stopj(self, acc=20, id=1):
        params = {"acc": acc}
        return self.sendCMD("stopj", params, id)

    def jog(self, coord, index=4, speed=10):
        """
        :param coord: 关节：0，基座：1，工具：2，用户：3，圆柱：4
        :param index:轴编号，0-11，如果是笛卡尔坐标系就分别对应xyz轴的加减和旋转加减，如果是关节坐标系则对应各个关节额的
        :param speed:速度
        这个函数只负责发送jog的指令，jog停止需要调用stop_move
        """
        self.stop = True
        # 默认为z轴增加
        # 记录当前坐标系
        current_coord = self.getCurrentCoord()
        # 将当前坐标系设置为工具坐标系
        self.setCurrentCoord(coord)
        params = {"index": index, "speed": speed}
        self.sendCMD("jog", params)
        self.setCurrentCoord(current_coord)
        # 要在线程中给self.stop赋值
        while True:
            if not self.stop:  # 如果stop_jog为False，停止运动
                self.stop_move()
                break
        return

    def keyboardControl(self):
        # 创建窗口
        cv2.namedWindow('Robot Control', cv2.WINDOW_NORMAL)

        # 初始化速度为0
        speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

        # 定义不同轴的速度变化步长
        step_size_xyz = 3  # XYZ轴的步长
        step_size_rpy = 3  # RPY（Roll, Pitch, Yaw）轴的步长

        acc = 100  # 加速度
        arot = 10  # 姿态加速度
        t = 0.2  # 执行时间
        current_time = time.time()

        while True:
            # 显示当前速度
            img = np.zeros((300, 600, 3), dtype=np.uint8)
            cv2.putText(img, f"Speed: {speed}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(img, "Control Keys:", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(img, "'w'/'s' -> X-axis", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, "'a'/'d' -> Y-axis", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, "'q'/'e' -> Z-axis", (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, "'r'/'f' -> Roll (R)", (50, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, "'t'/'g' -> Pitch (P)", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, "'y'/'h' -> Yaw (Y)", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, "Press 'Esc' to exit", (50, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow('Robot Control', img)

            # 等待用户输入
            key = cv2.waitKey(100) & 0xFF
            speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]
            flag = False
            # 控制XYZ速度
            if key == ord('w'):  # X轴正向
                speed[0] = step_size_xyz
                flag = True
            if key == ord('s'):  # X轴负向
                speed[0] = -step_size_xyz
                flag = True
            if key == ord('a'):  # Y轴负向
                speed[1] = -step_size_xyz
                flag = True
            if key == ord('d'):  # Y轴正向
                speed[1] = step_size_xyz
                flag = True
            if key == ord('q'):  # Z轴正向
                speed[2] = step_size_xyz
                flag = True
            if key == ord('e'):  # Z轴负向
                speed[2] = -step_size_xyz
                flag = True

            # 控制RPY速度
            if key == ord('r'):  # Roll正向
                speed[3] = step_size_rpy
                flag = True
            if key == ord('f'):  # Roll负向
                speed[3] = -step_size_rpy
                flag = True
            if key == ord('t'):  # Pitch正向
                speed[4] = step_size_rpy
                flag = True
            if key == ord('g'):  # Pitch负向
                speed[4] = -step_size_rpy
                flag = True
            if key == ord('y'):  # Yaw正向
                speed[5] = step_size_rpy
                flag = True
            if key == ord('h'):  # Yaw负向
                speed[5] = -step_size_rpy
                flag = True
            if key != 255:  # 判断是否按下了键
                print(key)

            # 更新速度
            print(f"Current Speed: {speed}")
            suc, theta, _ = self.getJointPos()
            # 发送速度命令到机器人
            if flag:
                suc, result, _ = self.moveBySpeedj(list(speed), acc, t)
                if suc:
                    print("Movement command sent successfully.")
                else:
                    print("Failed to send movement command.")
            # else:
            #     suc, result, _ = moveBySpeedj(sock, list(speed),acc)
            #     if suc:
            #         print("Movement command sent successfully.")
            #     else:
            #         print("Failed to send movement command.")

            # 按Esc键退出
            if key == 27:
                break
            loop_time = time.time() - current_time
            print(loop_time)
            current_time = time.time()

        # 关闭窗口
        cv2.destroyAllWindows()

    def AutoControl(self):

        # 初始化速度为0
        speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

        # 定义不同轴的速度变化步长
        step_size_xyz = 3  # XYZ轴的步长
        step_size_rpy = 3  # RPY（Roll, Pitch, Yaw）轴的步长

        acc = 100  # 加速度
        arot = 10  # 姿态加速度
        t = 0.1  # 执行时间
        set_time = time.time()
        change_time = 5
        speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        speed[0] = step_size_xyz
        stats = ["increase", "stop", "decrease", "stop"]
        i = 0
        current_time = time.time()
        while True:
            if time.time() - set_time > change_time:
                i = (i + 1) % 4
                set_time = time.time()
            stat = stats[i]
            img = np.zeros((300, 600, 3), dtype=np.uint8)
            cv2.putText(img, f"Speed: {speed}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(img, f"stat:{stat}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow('Robot Control', img)

            # 等待用户输入
            key = cv2.waitKey(10) & 0xFF
            if key == 27:
                break
            if stat == "increase":
                speed[0] = step_size_xyz
                suc, theta, _ = self.getJointPos()
                # 发送速度命令到机器人
                suc, result, _ = self.moveBySpeedj(list(speed), acc, t)
                if suc:
                    print("Movement command sent successfully.")
                else:
                    print("Failed to send movement command.")
            elif stat == "stop":
                speed[0] = 0
            elif stat == "decrease":
                speed[0] = -step_size_xyz
                suc, theta, _ = self.getJointPos()
                # 发送速度命令到机器人
                suc, result, _ = self.moveBySpeedj(list(speed), acc, t)
                if suc:
                    print("Movement command sent successfully.")
                else:
                    print("Failed to send movement command.")
            print(f"Current Speed: {speed}")

            time.sleep(0.02)
            loop_time = time.time() - current_time
            print(loop_time)
            current_time = time.time()


def main():
    robot_ip = "192.168.1.201"  # 机器人IP地址
    robot = eliteRobot(robot_ip)

    # 获取机器人当前伺服状态
    suc, servo_status, _ = robot.sendCMD("getServoStatus")
    if suc:
        print("Current Servo Status:", servo_status)
    robot.connect_gripper()
    robot.run_gripper(0)
    robot.disconnectETController()


if __name__ == "__main__":
    main()
