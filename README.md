# Jodell Gripper SDK 示例
![Video](./RG/RG.mp4)

本项目提供了对 Jodell 手爪设备的 Python 控制接口，基于 Jodell 提供的 SDK 构建。通过本项目，可以使用 Python 控制 Jodell 手爪的连接、使能、参数设置、运行和状态查询等功能。

## 目录结构
```
.
├── RG
│   ├── docs
│   │   ├── JodellTool-0.1.3-py3-none-any.whl       # SDK Python Wheel 包
│   │   ├── SDK开发说明(python版本).docx           # SDK 开发文档（Word 版）
│   │   └── SDK开发说明(python版本).pdf            # SDK 开发文档（PDF 版）
│   └── rg_api.py                                  # 手爪控制的核心 Python 脚本
```

## 主要功能

- **连接与断开设备**：
  - 自动检查设备是否存在
  - 通过串口与设备进行通信
- **手爪操作**：
  - 支持夹爪使能/禁用
  - 无参数运行模式
  - 有参数运行模式（可设置力矩、速度、位置）
- **状态与信息查询**：
  - 查询夹爪当前位置、速度、力矩
  - 获取夹爪运行状态

## 安装

1. 确保已安装 Python 3.6 或更高版本。
2. 使用以下命令安装 Jodell 提供的 Python SDK：
   ```bash
   pip install RG/docs/JodellTool-0.1.3-py3-none-any.whl
   ```
3. 确保您的设备已正确连接到计算机，并记录设备的串口号（例如：`/dev/ttyUSB0` 或 `COM3`）。

## 使用说明

1. 修改并运行 `rg_api.py` 示例代码以实现您的功能需求。
2. 示例：
   ```python
   from rg_api import ClawOperation

   # 初始化手爪对象
   claw = ClawOperation(com_port="/dev/ttyUSB0", baud_rate=115200)
   if claw.connect():  # 连接设备
       # 使能夹爪
       claw.enable_claw(claw_id=9, enable=True)

       # 控制夹爪运行
       claw.run_with_param(claw_id=9, force=100, speed=100, position=255)

       # 查询夹爪当前位置
       location = claw.get_current_location(claw_id=9)
       print(f"当前位置: {location}")

       # 断开设备连接
       claw.disconnect()
   ```

## 注意事项

1. 确保设备的串口配置与代码中的 `com_port` 和 `baud_rate` 参数匹配。
2. 力矩、速度和位置参数必须在合法范围（0-255）内。
3. 如需详细了解 SDK 功能，请参阅 `docs/SDK开发说明(python版本).pdf`。
