// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"
#include "Constants.h"

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

extern bool is_red;

Drive::Drive()
{
	m_fl_motor.SetInverted(false);
	m_fr_motor.SetInverted(true);
	m_bl_motor.SetInverted(false);
	m_br_motor.SetInverted(true);

	m_fl_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_fr_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_bl_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_br_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

	m_fl_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);
	m_fr_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);
	m_bl_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);
	m_br_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);

	m_fl_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
	m_fr_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
	m_bl_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
	m_br_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);

	m_gyro.Calibrate();
	m_fl_encoder.SetPosition(0);
	m_fr_encoder.SetPosition(0);
	m_bl_encoder.SetPosition(0);
	m_br_encoder.SetPosition(0);
}

// This method will be called once per scheduler run
void Drive::Periodic()
{

}

/**
 * This design pattern is called a "command factory" we use arguments to
 * create a commmand that we will use later.
 *
 * This function creates a new command that binds an initial
 * position to a lambda expression that resets the pose estimator.
 * We could then schedule this command to reset our position.
 */
frc2::CommandPtr Drive::init_lps_command(frc::Pose2d init_pos)
{
	return this->RunOnce([this, init_pos]() {
		m_estimator.ResetPosition(
			m_gyro.GetAngle(),
			get_wheel_pos(),
			init_pos
		);
		m_drive_timer.Restart();
	});
}

/**
 * This command factory returns a command that drives our robot.
 * While we are driving, we want to be constantly updating our position
 * estimator, so we run both drive_subcommand and estimate_position_subcommand
 * in parallel.
 */
frc2::CommandPtr Drive::drive_command(
		std::function<double(void)> drive_power,
		std::function<double(void)> strafe_power,
		std::function<double(void)> rot_power)
{
	return frc2::cmd::Parallel(
		drive_subcommand(drive_power, strafe_power, rot_power),
		estimate_position_subcommand()
	);
}

frc2::CommandPtr Drive::track_position_command(frc::Pose2d goal)
{
	return this->Run([this, goal]() {
		track_setpoint(goal);
	});
}

frc2::CommandPtr Drive::track_target_command()
{
	return this->Run([this]() {
		track_target();
	});
}

frc2::CommandPtr Drive::track_angle_command(std::function<units::radian_t()> angle_error)
{
	return this->Run([this, angle_error]() {
		track_angle(angle_error());
	});
};

frc2::FunctionalCommand Drive::wait_for_pos_command()
{
	static auto command = frc2::FunctionalCommand(
		[]() {},
		[]() {},
		[](bool interrupted) {},
		[this]() {
			bool finished = true;
			finished = finished && x_profile.IsFinished(0.0_s);
			finished = finished && y_profile.IsFinished(0.0_s);
			finished = finished && z_profile.IsFinished(0.0_s);
			return finished;
		},
		{this}
	);

	return command;
}

frc2::FunctionalCommand Drive::wait_for_angle_command()
{
	static auto command = frc2::FunctionalCommand(
		[]() {},
		[]() {},
		[](bool interrupted) {},
		[this]() {
			return z_profile.IsFinished(0.0_s);
		},
		{this}
	);

	return command;
}

void Drive::track_setpoint(frc::Pose2d goal)
{
	auto robot_pos = m_estimator.GetEstimatedPosition();
	auto wheel_speeds = get_wheel_speeds();
	auto field_speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
		m_kinematics.ToChassisSpeeds(wheel_speeds),
		robot_pos.Rotation()
	);
	auto angle_error = robot_pos.Rotation().Radians() - goal.Rotation().Radians();

	if (angle_error < -(units::radian_t)M_PI) {
		angle_error = angle_error + (units::radian_t)(2 * M_PI);
	} else if (angle_error > (units::radian_t)M_PI) {
		angle_error = angle_error - (units::radian_t)(2 * M_PI);
	}

	auto x_vel = x_profile.Calculate(
		20_ms,
		frc::TrapezoidProfile<units::meters>::State {
			robot_pos.X(), field_speed.vx
		},
		frc::TrapezoidProfile<units::meters>::State {
			goal.X(), 0.0_mps
		}
	);

	auto y_vel = y_profile.Calculate(
		20_ms,
		frc::TrapezoidProfile<units::meters>::State {
			robot_pos.Y(), field_speed.vy
		},
		frc::TrapezoidProfile<units::meters>::State {
			goal.Y(), 0.0_mps
		}
	);

	auto z_vel = z_profile.Calculate(
		20_ms,
		frc::TrapezoidProfile<units::radians>::State {
			robot_pos.Rotation().Radians(), field_speed.omega
		},
		frc::TrapezoidProfile<units::radians>::State {
			angle_error, 0.0_rad_per_s
		}
	);

	auto new_vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
		x_vel.velocity,
		y_vel.velocity,
		z_vel.velocity,
		robot_pos.Rotation().Radians()
	);

	auto [fl, fr, bl, br] = m_kinematics.ToWheelSpeeds(new_vel);
	set_speeds(fl, fr, bl, br);
}

void Drive::track_target()
{
	auto pos = m_estimator.GetEstimatedPosition();
	units::meter_t x_error, y_error;
	x_error = Constants::Positions::k_red_speaker_x - pos.X();
	y_error = Constants::Positions::k_red_speaker_y - pos.Y();
	auto angle = (units::radian_t)atan(x_error.value() / y_error.value());
	if (!is_red) {
		angle = (units::radian_t)M_PI - angle;
	}

	units::radian_t angle_error;
	angle_error = angle - pos.Rotation().Radians();
	track_angle(angle_error);
}

void Drive::track_angle(units::radian_t angle_error)
{
	auto robot_pos = m_estimator.GetEstimatedPosition();
	auto wheel_speeds = get_wheel_speeds();
	auto robot_speed = m_kinematics.ToChassisSpeeds(wheel_speeds);

	if (angle_error < -(units::radian_t)M_PI) {
		angle_error = angle_error + (units::radian_t)(2 * M_PI);
	} else if (angle_error > (units::radian_t)M_PI) {
		angle_error = angle_error - (units::radian_t)(2 * M_PI);
	}

	auto z_vel = z_profile.Calculate(
		20_ms,
		frc::TrapezoidProfile<units::radians>::State {
			robot_pos.Rotation().Radians(), robot_speed.omega
		},
		frc::TrapezoidProfile<units::radians>::State {
			angle_error, 0.0_rad_per_s
		}
	);

	auto new_vel = frc::ChassisSpeeds(0.0_mps, 0.0_mps, z_vel.velocity);
	auto [fl, fr, bl, br] = m_kinematics.ToWheelSpeeds(new_vel);
	set_speeds(fl, fr, bl, br);
}

void Drive::set_speeds(
	units::meters_per_second_t fl,
	units::meters_per_second_t fr,
	units::meters_per_second_t bl,
	units::meters_per_second_t br)
{
	auto fl_voltage = m_drive_ff.Calculate(fl);
	fl_voltage += (units::volt_t)(m_fl_pid.Calculate(
		m_fl_encoder.GetVelocity(), fl.value())
	);

	auto fr_voltage = m_drive_ff.Calculate(fr);
	fr_voltage += (units::volt_t)(m_fr_pid.Calculate(
		m_fr_encoder.GetVelocity(), fr.value())
	);

	auto bl_voltage = m_drive_ff.Calculate(bl);
	bl_voltage += (units::volt_t)(m_bl_pid.Calculate(
		m_bl_encoder.GetVelocity(), bl.value())
	);

	auto br_voltage = m_drive_ff.Calculate(br);
	br_voltage += (units::volt_t)(m_br_pid.Calculate(
		m_br_encoder.GetVelocity(), br.value())
	);

	m_fl_motor.SetVoltage(fl_voltage);
	m_fr_motor.SetVoltage(fr_voltage);
	m_bl_motor.SetVoltage(bl_voltage);
	m_br_motor.SetVoltage(br_voltage);
}

/**
 * Another example of a command factory. This time we pass in functions
 * to get our directions before we drive.
 *
 * These three lambda expressions are expected to return doubles that we
 * pass into DriveCartesian. We bind these these functions to our command
 * so that we get an updated value on each execution of this command. This
 * is different from the init_lps_command becuase we bind a "constant" value
 * there, that value is then the same for every invocation of the command.
 *
 * The RunEnd command factory is simply defined. While the command runs,
 * the first function is executed every 20ms. When the command ends
 */
frc2::CommandPtr Drive::drive_subcommand(
		std::function<double(void)> drive_power,
		std::function<double(void)> strafe_power,
		std::function<double(void)> rot_power)
{
	return this->RunEnd(
		[this, drive_power, strafe_power, rot_power]() {
			m_mecanums->DriveCartesian(
				drive_power(), strafe_power(), rot_power()
			);
		},
		[this]() { m_mecanums->DriveCartesian(0.0, 0.0, 0.0); }
	);
}

/**
 * Another command factory that returns a command that updates our limelight
 * position estimator.
 */
frc2::CommandPtr Drive::estimate_position_subcommand()
{
	return this->Run(
		[this]() {
			frc::Pose2d pos = get_vision_estimate();
			frc::Rotation2d rot = m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kYaw);
			m_estimator.UpdateWithTime(
				m_drive_timer.Get(),
				rot,
				get_wheel_pos()
			);
			
			if (pos.X().value() == 0.0
			    && pos.Y().value() == 0.0
			    && pos.Rotation().Radians().value() == 0.0) {
				return;
			}

			m_estimator.AddVisionMeasurement(
				pos, m_drive_timer.Get()
			);
		}
	);
}

units::meter_t Drive::get_target_dist()
{
	frc::Pose2d pos = m_estimator.GetEstimatedPosition();
	units::meter_t dist, x, y;

	if (is_red) {
		x = Constants::Positions::k_red_speaker_x - pos.X();
	} else {
		x = Constants::Positions::k_blue_speaker_y - pos.X();
	}
	y = Constants::Positions::k_red_speaker_y - pos.Y();

	dist = (units::meter_t)std::sqrt((x * x + y * y).value());
	return dist;
}
