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
#include "Sub.h"

// 强制包含 version.h 头文件
#define FORCE_VERSION_H_INCLUDE
#include "version.h"
// 取消强制包含 version.h 头文件
#undef FORCE_VERSION_H_INCLUDE

// 获取 AP_HAL 库的引用
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Sub class
  主要 Sub 类的构造函数
 */
Sub::Sub()
    : logger(g.log_bitmask),    // 初始化日志记录器
          control_mode(MANUAL), // 初始化控制模式为 MANUAL
          motors(MAIN_LOOP_RATE),   // 初始化电机控制器，设置主循环速率
          auto_mode(Auto_WP),       // 初始化自动模式为 Auto_WP
          guided_mode(Guided_WP),   // 初始化引导模式为 Guided_WP
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),  // 初始化自动航向模式
          inertial_nav(ahrs),       // 初始化惯性导航对象，并使用 ahrs 作为参数
          ahrs_view(ahrs, ROTATION_NONE),   // 初始化 AHRS 视图，使用 ahrs 和无旋转
          attitude_control(ahrs_view, aparm, motors, scheduler.get_loop_period_s()),    // 初始化姿态控制器
          pos_control(ahrs_view, inertial_nav, motors, attitude_control, scheduler.get_loop_period_s()),    // 初始化位置控制器
          wp_nav(inertial_nav, ahrs_view, pos_control, attitude_control),   // 初始化航点导航控制器
          loiter_nav(inertial_nav, ahrs_view, pos_control, attitude_control),   // 初始化盘旋导航控制器
          circle_nav(inertial_nav, ahrs_view, pos_control), // 初始化环绕导航控制器
          param_loader(var_info)    // 初始化参数加载器，使用 var_info 作为参数
{
    // init sensor error logging flags
    // 初始化传感器错误日志标志
    sensor_health.baro = true;      // 大气压传感器健康
    sensor_health.compass = true;   // 罗盘传感器健康

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    failsafe.pilot_input = true;    // 如果不是 SITL 模拟环境，启用飞行器控制器的飞行器失控保护（pilot input failsafe）
#endif
}

// 创建 Sub 类的对象
Sub sub;
// 创建对 AP_Vehicle 类的引用，将其指向 sub 对象
AP_Vehicle& vehicle = sub;
