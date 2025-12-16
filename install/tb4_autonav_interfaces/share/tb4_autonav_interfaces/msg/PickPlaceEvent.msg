# 状态标志 status%2 = 0 Nav开
# 0: 未到达  Nav开
# 1: 到达 Nav关 Pick开
# 2: Pick结束， Nav开 Pick关
# 3: 放置到达， Nav关 Place开
# 4：放置完毕 Nav开

int32 status

# float linear_velocity
# float angular_velocity
