# OriginCar 车载结果屏

独立 ROS2 节点，只订阅以下两个低频文本话题：

- `/qr_result`：扫码编号、方向和位置。
- `/vlm/result`：图生文节点输出的 JSON 或纯文本结果。

节点不发布控制消息、不调用任务服务。没有桌面环境或 Tk 无法连接屏幕时，
它会打印错误并自行退出，其他比赛节点继续运行。

```bash
sudo apt install python3-tk
cd /userdata/dev_ws
colcon build --symlink-install --packages-select origincar_screen_display
source install/setup.bash
ros2 launch origincar_screen_display screen_display.launch.py
```

若车载桌面的 X11 显示号不是 `:0`：

```bash
ros2 launch origincar_screen_display screen_display.launch.py display_server:=:1
```

按 `Esc` 退出全屏，按 `F11` 切换全屏。
