from rg_api import ClawOperation

# 初始化手爪对象
claw = ClawOperation(com_port="/dev/ttyUSB0", baud_rate=115200)
if claw.connect():  # 连接设备
    # 使能夹爪
    claw.enable_claw(claw_id=9, enable=True)

    # 控制夹爪运行
    claw.run_with_param(claw_id=9, force=100, speed=100, position=0)
    claw.run_with_param(claw_id=9, force=100, speed=100, position=255)

    # 查询夹爪当前位置
    location = claw.get_current_location(claw_id=9)
    print(f"当前位置: {location}")

    # 断开设备连接
    claw.disconnect()