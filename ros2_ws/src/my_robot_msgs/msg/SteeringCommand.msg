# 8 关节目标角 (rad)，顺序 1~8（见 README 转向关节标定）

uint8 SOURCE_IDLE=0
uint8 SOURCE_MAP=1
uint8 SOURCE_SPIN=2

std_msgs/Header header
float64[8] joint_positions
uint8 source
bool valid
