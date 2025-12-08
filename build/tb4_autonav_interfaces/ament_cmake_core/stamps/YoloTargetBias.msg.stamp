# tb4_autonav_interfaces/msg/YoloTargetBias.msg

std_msgs/Header header

# 是否当前有目标（true = 有，false = 没有）
bool has_target

# 目标类型："RED" / "GREEN" / "STOP_SIGN" / "NONE"
string type

# 目标距离（米），未知时设为 -1.0
float32 distance_m

# 像素归一化偏差：以图像中心为 0
# u_norm: 水平偏移，[-1, 1]，左负右正
# v_norm: 垂直偏移，[-1, 1]，上负下正
float32 u_norm
float32 v_norm
