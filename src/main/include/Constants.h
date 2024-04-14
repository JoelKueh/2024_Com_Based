// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Translation2d.h>

typedef units::volt_t ff_stat;
typedef units::unit_t<units::compound_unit<units::volt,
	units::inverse<units::meters_per_second>>> ff_vel;
typedef units::unit_t<units::compound_unit<units::volt,
	units::inverse<units::meters_per_second_squared>>> ff_accel;

typedef units::volt_t ff_rot_stat;
typedef units::unit_t<units::compound_unit<units::volt,
	units::inverse<units::rpm>>> ff_rot_vel;
typedef units::unit_t<units::compound_unit<units::volt,
	units::inverse<units::revolutions_per_minute_squared>>> ff_rot_accel;

namespace Constants {

struct ff_vals {
	ff_stat kS;
	ff_vel kV;
	ff_accel kA;
};

struct pid_vals {
	double kP;
	double kI;
	double kD;
};

namespace Operator {

constexpr int k_driver_port = 0;

}

namespace Drive {

constexpr int k_fl_id = 1;
constexpr int k_fr_id = 2;
constexpr int k_bl_id = 3;
constexpr int k_br_id = 4;

constexpr double k_rot_to_m   = 0.00797964534 / 9.13;
constexpr double k_rpm_to_mps = 0.4787787204 / 9.13;

const struct ff_vals k_drive_ff = {
	ff_stat { 0.30923 },
	ff_vel { 2.3046 },
	ff_accel { 0.32495 }
};

constexpr auto k_fl_pid_kP = 0.70234;
constexpr auto k_fr_pid_kP = 1.3031;
constexpr auto k_bl_pid_kP = 1.6944;
constexpr auto k_br_pid_kP = 0.80585;

const frc::Translation2d k_fl_pos = frc::Translation2d(0.1778_m, 0.2873375_m);
const frc::Translation2d k_fr_pos = frc::Translation2d(0.1778_m, 0.2873375_m);
const frc::Translation2d k_bl_pos = frc::Translation2d(0.1778_m, 0.2873375_m);
const frc::Translation2d k_br_pos = frc::Translation2d(0.1778_m, 0.2873375_m);

constexpr auto k_max_vel_x = 1.0_mps;
constexpr auto k_max_vel_y = 1.0_mps;
constexpr auto k_max_vel_z = (units::radians_per_second_t)1.0;
constexpr auto k_max_accel_x = 1.0_mps_sq;
constexpr auto k_max_accel_y = 1.0_mps_sq;
constexpr auto k_max_accel_z = (units::radians_per_second_squared_t)1.0;

}

namespace Shooter {

constexpr int k_left_id		= 5;
constexpr int k_right_id	= 6;
constexpr int k_intake_upper_id = 7;
constexpr int k_intake_lower_id = 8;
constexpr int k_ele_id		= 9;

constexpr int k_sensor_id = 1;
constexpr int k_limit_id = 0;

constexpr auto k_left_kP = 0.5;
constexpr auto k_left_ff_kS = (ff_rot_stat)0.30923;
constexpr auto k_left_ff_kV = (ff_rot_vel)2.3046;
constexpr auto k_left_ff_kA = (ff_rot_accel)0.32495;

constexpr auto k_right_kP = 0.5;
constexpr auto k_right_ff_kS = (ff_rot_stat)0.30923;
constexpr auto k_right_ff_kV = (ff_rot_vel)2.3046;
constexpr auto k_right_ff_kA = (ff_rot_accel)0.32495;

constexpr auto k_ele_kP = 1.0;

constexpr double k_pickup_setpoint = 1.0;
constexpr double k_high_setpoint = 400.0;

}

namespace Hanger {

constexpr int k_pneumatics_hub_id = 10;
constexpr int k_forward_id	  =  1;
constexpr int k_reverse_id	  =  0;

}

namespace Positions {

constexpr units::meter_t k_red_speaker_x = 8.308467_m;
constexpr units::meter_t k_red_speaker_y = 1.442593_m;

constexpr units::meter_t k_blue_speaker_x = -8.308975_m;
constexpr units::meter_t k_blue_speaker_y = 1.442593_m;

}

}
