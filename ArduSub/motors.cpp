#include "Sub.h"

// enable_motor_output() - 启用并输出最低可能值到电机 enable and output lowest possible value to motors
void Sub::enable_motor_output()
{
    motors.output_min();    // 将所有电机输出设置为最小值
}

// motors_output - 发送输出到电机库，该库将进行调整并发送到电调和伺服电机 send output to motors library which will adjust and send to ESCs and servos
void Sub::motors_output()
{
    // 电机检测模式直接控制推进器 Motor detection mode controls the thrusters directly
    if (control_mode == MOTOR_DETECT){
        return;
    }
    // 检查是否正在执行电机测试 check if we are performing the motor test
    if (ap.motor_test) {
        verify_motor_test(); // 验证电机测试
    } else {
        motors.set_interlock(true); // 启用电机输出
        motors.output(); // 发送电机输出信号
    }
}

// Initialize new style motor test
// Perform checks to see if it is ok to begin the motor test
// Returns true if motor test has begun
// 初始化新风格电机测试
// 执行检查以查看是否可以开始电机测试
// 如果电机测试已经开始，则返回true
bool Sub::init_motor_test()
{
    uint32_t tnow = AP_HAL::millis();

    // Ten second cooldown period required with no do_set_motor requests required
    // after failure.
    // 在失败后需要10秒的冷却时间，不需要发出do_set_motor请求
    if (tnow < last_do_motor_test_fail_ms + 10000 && last_do_motor_test_fail_ms > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "10 second cooldown required after motor test");
        return false;
    }

    // check if safety switch has been pushed
    // 检查是否按下了安全开关
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Disarm hardware safety switch before testing motors.");
        return false;
    }

    // Make sure we are on the ground
    // 确保我们在地面上
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }

    // 将所有电机输出设置为零
    enable_motor_output(); // set all motor outputs to zero
    ap.motor_test = true;

    return true;
}

// Verify new style motor test
// The motor test will fail if the interval between received
// MAV_CMD_DO_SET_MOTOR requests exceeds a timeout period
// Returns true if it is ok to proceed with new style motor test
// 验证新风格电机测试
// 如果接收到的MAV_CMD_DO_SET_MOTOR请求之间的时间间隔超过了超时期限，则电机测试将失败
// 如果可以进行新风格电机测试，则返回true
bool Sub::verify_motor_test()
{
    bool pass = true;

    // Require at least 2 Hz incoming do_set_motor requests
    // 需要至少2赫兹的do_set_motor请求
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor test timed out!");
        pass = false;
    }

    if (!pass) {
        ap.motor_test = false;
        AP::arming().disarm(AP_Arming::Method::MOTORTEST);
        last_do_motor_test_fail_ms = AP_HAL::millis();
        return false;
    }

    return true;
}

// 处理do_motor_test命令
bool Sub::handle_do_motor_test(mavlink_command_long_t command) {
    last_do_motor_test_ms = AP_HAL::millis();

    // If we are not already testing motors, initialize test
    // 如果我们还没有在测试电机，请初始化测试
    static uint32_t tLastInitializationFailed = 0;
    if(!ap.motor_test) {
        // Do not allow initializations attempt under 2 seconds
        // If one fails, we need to give the user time to fix the issue
        // instead of spamming error messages
        // 不允许在2秒内初始化尝试
        // 如果一个失败了，我们需要给用户时间来解决问题，而不是不断出错
        if (AP_HAL::millis() > (tLastInitializationFailed + 2000)) {
            if (!init_motor_test()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "motor test initialization failed!");
                tLastInitializationFailed = AP_HAL::millis();
                return false; // init fail  初始化失败
            }
        } else {
            return false;
        }
    }

    float motor_number = command.param1;
    float throttle_type = command.param2;
    float throttle = command.param3;
    // float timeout_s = command.param4; // not used
    // float motor_count = command.param5; // not used
    float test_type = command.param6;

    if (!is_equal(test_type, (float)MOTOR_TEST_ORDER_BOARD)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad test type %0.2f", (double)test_type);
        return false; // test type not supported here 不支持此测试类型
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PILOT)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad throttle type %0.2f", (double)throttle_type);

        return false; // throttle type not supported here 不支持此油门类型
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PWM)) {
        return motors.output_test_num(motor_number, throttle); // 如果设置了电机输出，则返回true
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PERCENT)) {
        throttle = constrain_float(throttle, 0.0f, 100.0f);
        throttle = channel_throttle->get_radio_min() + throttle / 100.0f * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        return motors.output_test_num(motor_number, throttle); // 如果设置了电机输出，则返回true
    }

    return false;
}


// translate wpnav roll/pitch outputs to lateral/forward
// 将wpnav的横滚/俯仰输出转换为横向/前向
void Sub::translate_wpnav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    // 获取以厘米度为单位的横滚和俯仰目标值
    int32_t lateral = wp_nav.get_roll();
    int32_t forward = -wp_nav.get_pitch(); // 输出是反向的 output is reversed

    // constrain target forward/lateral values
    // The outputs of wp_nav.get_roll and get_pitch should already be constrained to these values
    // 限制目标前向/横向值
    // wp_nav.get_roll和get_pitch的输出值应该已经受到这些值的限制
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize 归一化
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// translate wpnav roll/pitch outputs to lateral/forward
// 将circle_nav的横滚/俯仰输出转换为横向/前向
void Sub::translate_circle_nav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    // 以厘米度为单位获取横滚和俯仰目标值
    int32_t lateral = circle_nav.get_roll();
    int32_t forward = -circle_nav.get_pitch(); // 输出是反向的

    // 限制目标前向/横向值 constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize    归一化
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// translate pos_control roll/pitch outputs to lateral/forward
// 将pos_control的横滚/俯仰输出转换为横向/前向
void Sub::translate_pos_control_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    // 以厘米度为单位获取横滚和俯仰目标值
    int32_t lateral = pos_control.get_roll_cd();
    int32_t forward = -pos_control.get_pitch_cd(); // 输出是反向的

    // constrain target forward/lateral values
    // 限制目标前向/横向值
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize    归一化
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}
