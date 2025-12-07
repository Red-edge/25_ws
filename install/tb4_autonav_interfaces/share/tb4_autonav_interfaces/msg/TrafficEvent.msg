# TrafficEvent.msg
# 视觉节点和导航节点之间传递“视觉是否就绪 + 交通事件检测结果”

builtin_interfaces/Time stamp

# 视觉节点是否已经就绪：
#   false：节点刚启动，还没收到相机图像
#   true ：至少成功处理过一帧图像
bool is_ready

# 检测到的事件类型：
#   "RED"        红灯
#   "GREEN"      绿灯
#   "STOP_SIGN"  停止标志
#   "NONE"       无关键事件
string type

# 事件的粗略距离，单位 m；若不可估计则为 -1.0
float32 distance