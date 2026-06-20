/*
 * void 控制器：四轮转动，其余腿关节位置锁定。
 *
 * 控制方式：
 *   - 轮胎（link_004/007/010/013）：速度模式，position = INFINITY
 *   - 腿关节（link_002/003/005/006/008/009/011/012）：位置模式，锁在仿真开始时的角度
 */

#include <math.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

/* 控制器调用周期（ms），需与 PositionSensor 采样周期一致 */
#define TIME_STEP 64

/* 轮胎角速度（rad/s），RotationalMotor maxVelocity 默认上限为 5 */
#define WHEEL_SPEED 0.2

/* 四个轮胎关节（每条腿最末级 HingeJoint） */
static const char *WHEEL_MOTORS[] = {
    "link_004_joint",
    "link_007_joint",
    "link_010_joint",
    "link_013_joint",
};

typedef struct {
  const char *motor;
  const char *sensor;
} JointPair;

/* 需要锁定的中间腿关节（hip / knee，不含轮胎） */
static const JointPair LEG_JOINTS[] = {
    {"link_002_joint", "link_002_joint_sensor"},
    {"link_003_joint", "link_003_joint_sensor"},
    {"link_005_joint", "link_005_joint_sensor"},
    {"link_006_joint", "link_006_joint_sensor"},
    {"link_008_joint", "link_008_joint_sensor"},
    {"link_009_joint", "link_009_joint_sensor"},
    {"link_011_joint", "link_011_joint_sensor"},
    {"link_012_joint", "link_012_joint_sensor"},
};

int main(int argc, char **argv) {
  const int wheel_count = sizeof(WHEEL_MOTORS) / sizeof(WHEEL_MOTORS[0]);
  const int leg_count = sizeof(LEG_JOINTS) / sizeof(LEG_JOINTS[0]);
  WbDeviceTag wheels[4];
  WbDeviceTag leg_motors[8];
  WbDeviceTag leg_sensors[8];
  double leg_hold[8]; /* 腿关节锁定角度（rad），首帧读取后不变 */

  (void)argc;
  (void)argv;

  wb_robot_init();

  /* 初始化腿关节：仅启用位置传感器，暂不设目标角度 */
  for (int i = 0; i < leg_count; ++i) {
    leg_motors[i] = wb_robot_get_device(LEG_JOINTS[i].motor);
    leg_sensors[i] = wb_robot_get_device(LEG_JOINTS[i].sensor);
    wb_position_sensor_enable(leg_sensors[i], TIME_STEP);
  }

  /* 轮胎切换为速度控制模式（position = INFINITY 表示不控位置） */
  for (int i = 0; i < wheel_count; ++i) {
    wheels[i] = wb_robot_get_device(WHEEL_MOTORS[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }

  /*
   * 必须先 step 一次，PositionSensor 才有有效读数；
   * 若直接 get_value 会得到 NaN。
   */
  if (wb_robot_step(TIME_STEP) == -1)
    goto cleanup;

  /* 读取当前姿态并锁定腿关节（纯位置控制，力矩上限由 wbt 里 maxTorque 决定） */
  for (int i = 0; i < leg_count; ++i) {
    leg_hold[i] = wb_position_sensor_get_value(leg_sensors[i]);
    if (!isfinite(leg_hold[i]))
      leg_hold[i] = 0.0;
    wb_motor_set_position(leg_motors[i], leg_hold[i]);
  }

  /* 首帧后再启动轮胎旋转 */
  for (int i = 0; i < wheel_count; ++i)
    wb_motor_set_velocity(wheels[i], WHEEL_SPEED);

  while (wb_robot_step(TIME_STEP) != -1) {
    for (int i = 0; i < wheel_count; ++i)
      wb_motor_set_velocity(wheels[i], WHEEL_SPEED);

    /* 每帧重复发送同一目标角度，维持锁定 */
    for (int i = 0; i < leg_count; ++i)
      wb_motor_set_position(leg_motors[i], leg_hold[i]);
  }

cleanup:
  wb_robot_cleanup();
  return 0;
}
