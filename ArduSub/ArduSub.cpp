/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// ArduSub scheduling, originally copied from ArduCopter

#include "Sub.h"

// 定义一个宏，用于创建调度任务
// 参数说明：
// - func：任务函数的名称
// - rate_hz：任务的执行频率（以赫兹为单位）
// - max_time_micros：任务的最大执行时间（以微秒为单位）
#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
/*
  快速CPU的调度表 - 所有常规任务（除了fast_loop()）都应该在此处列出，
  以及它们应该被调用的频率（以赫兹为单位）和预计最长执行时间（以微秒为单位）
*/
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    // 每秒50次的任务，最大执行时间不超过75微秒
    SCHED_TASK(fifty_hz_loop,         50,     75),

    // 调用AP_GPS类的update函数，频率为50赫兹，最大执行时间为200微秒
    SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200),

// 如果光流（OpticalFlow）功能已启用，则调用光流（OpticalFlow）类的update函数，
// 频率为200赫兹，最大执行时间为160微秒
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &sub.optflow,             update,         200, 160),
#endif

    // 更新电池和罗盘数据，以10赫兹的频率调用，最大执行时间不超过120微秒
    SCHED_TASK(update_batt_compass,   10,    120),

    // 读取测距仪数据，以20赫兹的频率调用，最大执行时间不超过100微秒
    SCHED_TASK(read_rangefinder,      20,    100),

    // 更新高度数据，以10赫兹的频率调用，最大执行时间不超过100微秒
    SCHED_TASK(update_altitude,       10,    100),

    // 3.3赫兹的任务循环，最大执行时间不超过75微秒
    SCHED_TASK(three_hz_loop,          3,     75),

    // 更新转弯计数器，以10赫兹的频率调用，最大执行时间不超过50微秒
    SCHED_TASK(update_turn_counter,   10,     50),

    // 调用AP_Baro类的accumulate函数，以50赫兹的频率调用，最大执行时间不超过90微秒
    SCHED_TASK_CLASS(AP_Baro,             &sub.barometer,    accumulate,          50,  90),

    // 调用AP_Notify类的update函数，以50赫兹的频率调用，最大执行时间不超过90微秒
    SCHED_TASK_CLASS(AP_Notify,           &sub.notify,       update,              50,  90),

    // 1赫兹的任务循环，最大执行时间不超过100微秒
    SCHED_TASK(one_hz_loop,            1,    100),

    // 调用GCS类的update_receive函数，以400赫兹的频率调用，最大执行时间不超过180微秒
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180),

    // 调用GCS类的update_send函数，以400赫兹的频率调用，最大执行时间不超过550微秒
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550),

// 如果AC_FENCE被启用，则调用AC_Fence类的update函数，以10赫兹的频率调用，最大执行时间不超过100微秒
#if AC_FENCE == ENABLED
    SCHED_TASK_CLASS(AC_Fence, &sub.fence, update, 10, 100),
#endif

// 如果HAL_MOUNT_ENABLED被启用，则调用AP_Mount类的update函数，以50赫兹的频率调用，最大执行时间不超过75微秒
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &sub.camera_mount, update, 50, 75),
#endif

// 如果相机功能（CAMERA）被启用，则调用AP_Camera类的update函数，以50赫兹的频率调用，最大执行时间不超过75微秒
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera, &sub.camera, update, 50, 75),
#endif

    // 调用ten_hz_logging_loop函数，以10赫兹的频率调用，最大执行时间不超过350微秒
    SCHED_TASK(ten_hz_logging_loop, 10, 350),

    // 调用twentyfive_hz_logging函数，以25赫兹的频率调用，最大执行时间不超过110微秒
    SCHED_TASK(twentyfive_hz_logging, 25, 110),

    // 调用AP_Logger类的periodic_tasks函数，以400赫兹的频率调用，最大执行时间不超过300微秒
    SCHED_TASK_CLASS(AP_Logger, &sub.logger, periodic_tasks, 400, 300),

    // 调用AP_InertialSensor类的periodic函数，以400赫兹的频率调用，最大执行时间不超过50微秒
    SCHED_TASK_CLASS(AP_InertialSensor, &sub.ins, periodic, 400, 50),

    // 调用AP_Scheduler类的update_logging函数，以0.1赫兹的频率调用，最大执行时间不超过75微秒
    SCHED_TASK_CLASS(AP_Scheduler, &sub.scheduler, update_logging, 0.1, 75),

// 如果RPM功能（RPM_ENABLED）被启用，则调用rpm_update函数，以10赫兹的频率调用，最大执行时间不超过200微秒
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update, 10, 200),
#endif

    // 调用Compass类的cal_update函数，以100赫兹的频率调用，最大执行时间不超过100微秒
    SCHED_TASK_CLASS(Compass, &sub.compass, cal_update, 100, 100),

    // 调用accel_cal_update函数，以10赫兹的频率调用，最大执行时间不超过100微秒
    SCHED_TASK(accel_cal_update, 10, 100),

    // 调用terrain_update函数，以10赫兹的频率调用，最大执行时间不超过100微秒
    SCHED_TASK(terrain_update, 10, 100),

// 如果GRIPPER功能（GRIPPER_ENABLED）被启用，则调用AP_Gripper类的update函数，以10赫兹的频率调用，最大执行时间不超过75微秒
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper, &sub.g2.gripper, update, 10, 75),
#endif

// 如果定义了USERHOOK_FASTLOOP，则调用userhook_FastLoop函数，以100赫兹的频率调用，最大执行时间不超过75微秒
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop, 100, 75),
#endif

// 如果定义了USERHOOK_50HZLOOP，则调用userhook_50Hz函数，以50赫兹的频率调用，最大执行时间不超过75微秒
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz, 50, 75),
#endif

// 如果定义了USERHOOK_MEDIUMLOOP，则调用userhook_MediumLoop函数，以10赫兹的频率调用，最大执行时间不超过75微秒
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop, 10, 75),
#endif

// 如果定义了USERHOOK_SLOWLOOP，则调用userhook_SlowLoop函数，以3.3赫兹的频率调用，最大执行时间不超过75微秒
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop, 3.3, 75),
#endif

// 如果定义了USERHOOK_SUPERSLOWLOOP，则调用userhook_SuperSlowLoop函数，以1赫兹的频率调用，最大执行时间不超过75微秒
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1, 75),
#endif

    // 调用read_airspeed函数，以10赫兹的频率调用，最大执行时间不超过100微秒
    SCHED_TASK(read_airspeed, 10, 100),
};

// 获取调度任务信息的函数
void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit)
{
    // 设置任务信息的指针为调度任务数组的第一个元素
    tasks = &scheduler_tasks[0];
    
    // 设置任务数量为调度任务数组的大小
    task_count = ARRAY_SIZE(scheduler_tasks);
    
    // 设置日志位（log_bit）为MASK_LOG_PM
    log_bit = MASK_LOG_PM;
}

// Sub类中的失败安全（failsafe）优先级数组
constexpr int8_t Sub::_failsafe_priorities[5];

// Main loop - 400hz 主循环
void Sub::fast_loop()
{
    // 立即更新INS以获取当前的陀螺仪数据 update INS immediately to get current gyro data populated
    ins.update();

    // 在手动模式或电机检测模式下不运行速率控制器 don't run rate controller in manual or motordetection modes
    if (control_mode != MANUAL && control_mode != MOTOR_DETECT) {
        // 运行只需要IMU数据的低级速率控制器 run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // 将输出发送到电机库 send outputs to the motors library
    motors_output();

    // 运行EKF状态估计器（计算成本较高） run EKF state estimator (expensive)
    read_AHRS();

    // 惯性导航 Inertial Nav
    read_inertia();

    // 检查EKF是否已重置目标航向 check if ekf has reset target heading
    check_ekf_yaw_reset();

    // 运行姿态控制器 run the attitude controllers
    update_flight_mode();

    // 根据需要从EKF更新家位置 update home from EKF if necessary
    update_home_from_EKF();

    // 检查是否达到水面或底部 check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

// 如果HAL_MOUNT_ENABLED被启用，则调用camera_mount对象的update_fast函数，用于相机云台的快速更新
#if HAL_MOUNT_ENABLED
    // 相机云台的快速更新 camera mount's fast update
    camera_mount.update_fast();
#endif

    // 如果需要记录任何传感器健康状态，就调用Log_Sensor_Health函数 log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }

    // 调用AP_Vehicle类的fast_loop函数，这是飞行控制系统的快速循环的最后一步
    AP_Vehicle::fast_loop();
}

// 50赫兹任务
void Sub::fifty_hz_loop()
{
    // 检查飞行员输入失效保护
    failsafe_pilot_input_check();

    // 检查飞行器碰撞保护
    failsafe_crash_check();

    // 检查EKF（扩展卡尔曼滤波器）失效保护
    failsafe_ekf_check();

    // 检查传感器失效保护
    failsafe_sensors_check();

    // 更新遥控器输入/输出
    rc().read_input();
    SRV_Channels::output_ch_all();
}


// update_batt_compass - 读取电池和罗盘数据
// 应该以10赫兹的频率调用
void Sub::update_batt_compass()
{
    // 在读取罗盘之前读取电池数据，因为电池数据可能用于电机干扰补偿
    battery.read();

    // 如果罗盘功能已启用
    if (AP::compass().enabled()) {
        // 使用油门值来更新罗盘 - 用于罗盘干扰补偿
        compass.set_throttle(motors.get_throttle());
        
        // 读取罗盘数据
        compass.read();
    }
}


// ten_hz_logging_loop
// should be run at 10hz
// 应以10赫兹的频率运行
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    // 如果尚未以更高的速率记录姿态数据，就记录姿态数据

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();   // 记录姿态数据
        ahrs_view.Write_Rate(motors, attitude_control, pos_control);

        // 如果需要记录PID数据，就记录PID数据
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }
    // 如果需要记录电机电池数据，就记录电机电池数据
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }

    // 如果需要记录遥控器输入数据，就记录遥控器输入数据
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }

    // 如果需要记录遥控器输出数据，就记录遥控器输出数据
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }

    // 如果需要记录导航控制器输出数据，并且飞行模式要求GPS或者不要求手动油门控制，就记录导航控制器输出数据
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || !mode_has_manual_throttle(control_mode))) {
        pos_control.write_log();
    }
    
    // 如果需要记录IMU数据或者IMU快速数据或者IMU原始数据，就记录IMU数据的振动信息
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }

    // 如果需要记录控制器数据，就记录控制器数据的监控信息
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
}

// twentyfive_hz_logging_loop
// should be run at 25hz
// 应以25赫兹的频率运行
void Sub::twentyfive_hz_logging()
{
    // 如果需要记录快速姿态数据，就记录姿态数据
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();   // 记录姿态数据
        ahrs_view.Write_Rate(motors, attitude_control, pos_control);

        // 如果需要记录PID数据，就记录PID数据
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    // 如果需要记录IMU数据，并且尚未以更高的速率记录IMU原始数据，就记录IMU数据
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_IMU();
    }
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update(); // 更新泄漏检测器状态

    failsafe_leak_check();  // 检查泄漏状态

    failsafe_internal_pressure_check(); // 检查内部压力状态

    failsafe_internal_temperature_check(); // 检查内部温度状态

    // check if we've lost contact with the ground station
    // 检查是否失去与地面站的联系
    failsafe_gcs_check();

    // check if we've lost terrain data
    // 检查是否失去地形数据
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    // 检查是否已经触发了电子围栏
    fence_check();
#endif // AC_FENCE_ENABLED

    ServoRelayEvents.update_events();   // 更新伺服继电器事件
}

// one_hz_loop - runs at 1Hz 每秒运行一次
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);  // 执行预飞行检查，并获取是否成功的标志
    ap.pre_arm_check = arm_check; // 更新AP系统的预飞行检查状态
    AP_Notify::flags.pre_arm_check = arm_check; // 更新通知模块的预飞行检查状态
    AP_Notify::flags.pre_arm_gps_check = position_ok(); // 更新通知模块的GPS状态
    AP_Notify::flags.flying = motors.armed(); // 更新通知模块的飞行状态

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);  // 记录AP系统的状态数据
    }

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        // 在初始配置期间，使得可以在运行时更改ahrs方向
        ahrs.update_orientation();

        motors.update_throttle_range(); // 更新油门范围
    }

    // update assigned functions and enable auxiliary servos
    // 更新分配的功能和启用辅助伺服
    SRV_Channels::enable_aux_servos();

    // update position controller alt limits
    // 更新位置控制器的高度限制
    update_poscon_alt_max();

    // log terrain data
    // 记录地形数据
    terrain_logging();

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    // 在飞行器解锁时，需要设置 "likely flying"，以便允许罗盘校准运行
    set_likely_flying(hal.util->get_soft_armed());
}

void Sub::read_AHRS()
{
    // 执行IMU计算并获取姿态信息
    //-----------------------------------------------
    // <true> 告诉AHRS跳过INS更新，因为我们在fast_loop()中已经执行了它
    ahrs.update(true); // 更新AHRS状态
    ahrs_view.update(true); // 更新AHRS视图
}

// read baro and rangefinder altitude at 10hz
// 每秒10次读取气压计和测距仪高度
void Sub::update_altitude()
{
    // 读取气压计高度
    read_barometer(); // 读取气压计数据

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning(); // 记录控制调整数据
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages(); // 记录陀螺仪FFT数据
#else
        write_notch_log_messages(); // 记录陀螺仪Notch滤波数据
#endif
    }
}

bool Sub::control_check_barometer()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // 如果没有深度传感器，则无法保持深度
    if (!ap.depth_sensor_present) { // can't hold depth without a depth sensor
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    } else if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
#endif
    return true;
}

// vehicle specific waypoint info helpers
// 获取与航点距离的帮助函数
bool Sub::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    distance = sub.wp_nav.get_wp_distance_to_destination() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
// 获取与航点方向角的帮助函数
bool Sub::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    bearing = sub.wp_nav.get_wp_bearing_to_destination() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
// 获取与航点横向偏差的帮助函数
bool Sub::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // 没有横向偏差报告 no crosstrack error reported, see GCS_MAVLINK_Sub::send_nav_controller_output()
    xtrack_error = 0;
    return true;
}

AP_HAL_MAIN_CALLBACKS(&sub); // 定义AP_HAL主回调函数
