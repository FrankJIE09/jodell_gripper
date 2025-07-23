#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from RG.rg_api import ClawOperation

USB_PORT = "/dev/ttyUSB1"
CLAW_ID = 9
claw = None

def gripper_cmd_callback(msg):
    global claw
    # 期望格式: position=120
    params = dict(x.split('=') for x in msg.data.split(',') if '=' in x)
    try:
        position = int(params.get('position', 0))
    except Exception:
        rospy.logwarn("position参数格式错误: %s", msg.data)
        return
    rospy.loginfo("收到控制命令: position=%d (端口:%s, id:%d)", position, USB_PORT, CLAW_ID)
    if not (0 <= position <= 255):
        rospy.logwarn("position必须在0-255之间: %d", position)
        return
    try:
        if claw is None:
            rospy.loginfo("初始化ClawOperation: usb_port=%s, id=%d", USB_PORT, CLAW_ID)
            claw = ClawOperation(com_port=USB_PORT, baud_rate=115200)
            if not claw.connect():
                rospy.logerr("串口连接失败: %s", USB_PORT)
                claw = None
                return
            claw.enable_claw(claw_id=CLAW_ID, enable=True)
        rospy.loginfo("运行夹爪: id=%d, force=255, speed=255, position=%d", CLAW_ID, position)
        claw.run_with_param(claw_id=CLAW_ID, force=255, speed=255, position=position)
        rospy.loginfo("操作成功")
    except Exception as e:
        rospy.logerr("操作异常: %s", str(e))
        claw = None


def main():
    rospy.init_node('gripper_topic_server')
    rospy.Subscriber('/gripper_control_cmd', String, gripper_cmd_callback)
    rospy.loginfo("夹爪控制Topic节点已启动，订阅/gripper_control_cmd ...")
    rospy.spin()

if __name__ == "__main__":
    main() 