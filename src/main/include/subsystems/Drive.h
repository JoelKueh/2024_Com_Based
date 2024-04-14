// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "frc2/command/FunctionalCommand.h"

#include <vector>

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Timer.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <networktables/NetworkTableInstance.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>

class Drive : public frc2::SubsystemBase {
public:
	Drive();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;

	frc2::CommandPtr init_lps_command(frc::Pose2d init_pos);
	frc2::CommandPtr drive_command(
			std::function<double(void)> drive_power,
			std::function<double(void)> strafe_power,
			std::function<double(void)> rot_power
	);
	frc2::CommandPtr track_position_command(frc::Pose2d goal);
	frc2::CommandPtr track_target_command();
	frc2::CommandPtr track_angle_command(std::function<units::radian_t()> angle_error);
	frc2::FunctionalCommand wait_for_pos_command();
	frc2::FunctionalCommand wait_for_angle_command();

	units::meter_t get_target_dist();

private:
	frc::MecanumDrive *m_mecanums;

	rev::CANSparkMax m_fl_motor {
		Constants::Drive::k_fl_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_fl_encoder = m_fl_motor.GetEncoder();
	frc::PIDController m_fl_pid {
		Constants::Drive::k_fl_pid_kP, 0.0, 0.0
	};

	rev::CANSparkMax m_fr_motor {
		Constants::Drive::k_fr_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_fr_encoder = m_fr_motor.GetEncoder();
	frc::PIDController m_fr_pid {
		Constants::Drive::k_fr_pid_kP, 0.0, 0.0
	};

	rev::CANSparkMax m_bl_motor {
		Constants::Drive::k_bl_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_bl_encoder = m_bl_motor.GetEncoder();
	frc::PIDController m_bl_pid {
		Constants::Drive::k_bl_pid_kP, 0.0, 0.0
	};

	rev::CANSparkMax m_br_motor {
		Constants::Drive::k_br_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_br_encoder = m_br_motor.GetEncoder();
	frc::PIDController m_br_pid {
		Constants::Drive::k_br_pid_kP, 0.0, 0.0
	};

	frc::ADIS16470_IMU m_gyro;

	frc::SimpleMotorFeedforward<units::meter> m_drive_ff {
		Constants::Drive::k_drive_ff.kS,
		Constants::Drive::k_drive_ff.kV,
		Constants::Drive::k_drive_ff.kA
	};

	frc::MecanumDriveKinematics m_kinematics {
		Constants::Drive::k_fl_pos,
		Constants::Drive::k_fr_pos,
		Constants::Drive::k_bl_pos,
		Constants::Drive::k_br_pos
	};

	frc::MecanumDriveWheelPositions init_wheel_pos { 0.0_m, 0.0_m, 0.0_m, 0.0_m };
	frc::Pose2d init_pos { 0.0_m, 0.0_m, (frc::Rotation2d)0.0_deg };
	frc::Timer m_drive_timer;
	frc::MecanumDrivePoseEstimator m_estimator {
		m_kinematics, 0.0_deg, init_wheel_pos, init_pos
	};

	frc::TrapezoidProfile<units::meter>::Constraints x_constr {
		Constants::Drive::k_max_vel_x,
		Constants::Drive::k_max_accel_x
	};
	frc::TrapezoidProfile<units::meter> x_profile { x_constr };

	frc::TrapezoidProfile<units::meter>::Constraints y_constr {
		Constants::Drive::k_max_vel_y,
		Constants::Drive::k_max_accel_y
	};
	frc::TrapezoidProfile<units::meter> y_profile { y_constr };

	frc::TrapezoidProfile<units::radian>::Constraints z_constr {
		Constants::Drive::k_max_vel_z,
		Constants::Drive::k_max_accel_z
	};
	frc::TrapezoidProfile<units::radian> z_profile { z_constr };

	inline frc::MecanumDriveWheelPositions get_wheel_pos() {
		return frc::MecanumDriveWheelPositions(
			(units::meter_t)m_fl_encoder.GetPosition(),
			(units::meter_t)m_fr_encoder.GetPosition(),
			(units::meter_t)m_bl_encoder.GetPosition(),
			(units::meter_t)m_br_encoder.GetPosition()
		);
	}

	inline frc::MecanumDriveWheelSpeeds get_wheel_speeds() {
		return frc::MecanumDriveWheelSpeeds(
			(units::meters_per_second_t)m_fl_encoder.GetVelocity(),
			(units::meters_per_second_t)m_fr_encoder.GetVelocity(),
			(units::meters_per_second_t)m_bl_encoder.GetVelocity(),
			(units::meters_per_second_t)m_br_encoder.GetVelocity()
		);
	}

	nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();
	inline frc::Pose2d get_vision_estimate() {
		std::vector<double> pos(6);
		pos = nt_inst.GetTable("limelight")->GetNumberArray(
			"botpose",
			std::vector<double>(6)
		);

		return frc::Pose2d(
			(units::meter_t)pos[0],
			(units::meter_t)pos[1],
			(units::degree_t)pos[5]
		);
	}

	frc2::CommandPtr drive_subcommand(
			std::function<double(void)> drive_power,
			std::function<double(void)> strafe_power,
			std::function<double(void)> rot_power
	);

	frc2::CommandPtr estimate_position_subcommand();

	void track_setpoint(frc::Pose2d goal);
	void track_target();
	void track_angle(units::radian_t angle_error);
	void set_speeds(
		units::meters_per_second_t fl,
		units::meters_per_second_t fr,
		units::meters_per_second_t bl,
		units::meters_per_second_t br
	);
};
