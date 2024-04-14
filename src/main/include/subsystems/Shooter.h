// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <units/time.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"
#include "frc2/command/FunctionalCommand.h"

class Shooter : public frc2::SubsystemBase {
public:
	Shooter();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;
	
	inline bool has_note() { return !m_sensor.Get(); }
	inline bool at_setpoint() {
		bool done = true;
		done = done && m_ele_pid.AtSetpoint();
		done = done && m_left_pid.AtSetpoint();
		done = done && m_right_pid.AtSetpoint();
		return done;
	}

	frc2::CommandPtr track_command(std::function<units::meter_t()> dist);
	frc2::CommandPtr track_elevator_command(std::function<double()> setpoint);
	frc2::CommandPtr shoot_command();
	frc2::FunctionalCommand wait_shooter_command();
	frc2::FunctionalCommand zero_command();
	frc2::FunctionalCommand pickup_command();

private:
	rev::CANSparkMax m_left_motor {
		Constants::Shooter::k_left_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_left_encoder = m_left_motor.GetEncoder();
	frc::PIDController m_left_pid { Constants::Shooter::k_left_kP, 0.0, 0.0 };
	frc::SimpleMotorFeedforward<units::radian> m_left_ff {
		Constants::Shooter::k_left_ff_kS,
		Constants::Shooter::k_left_ff_kV,
		Constants::Shooter::k_left_ff_kA
	};

	rev::CANSparkMax m_right_motor {
		Constants::Shooter::k_right_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_right_encoder = m_right_motor.GetEncoder();
	frc::PIDController m_right_pid { Constants::Shooter::k_right_kP, 0.0, 0.0 };
	frc::SimpleMotorFeedforward<units::radian> m_right_ff {
		Constants::Shooter::k_right_ff_kS,
		Constants::Shooter::k_right_ff_kV,
		Constants::Shooter::k_right_ff_kA
	};

	rev::CANSparkMax m_ele_motor {
		Constants::Shooter::k_ele_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkRelativeEncoder m_ele_encoder = m_ele_motor.GetEncoder();
	frc::PIDController m_ele_pid { Constants::Shooter::k_ele_kP, 0.0, 0.0 };

	rev::CANSparkMax m_top_motor {
		Constants::Shooter::k_intake_lower_id,
		rev::CANSparkMax::MotorType::kBrushed
	};
	rev::CANSparkMax m_low_motor {
		Constants::Shooter::k_intake_upper_id,
		rev::CANSparkMax::MotorType::kBrushed
	};

	frc::DigitalInput m_sensor { Constants::Shooter::k_sensor_id };
	frc::DigitalInput m_ele_limit { Constants::Shooter::k_limit_id };
	
	void track(units::meter_t dist);
	double dist_to_setpoint(units::meter_t dist);
};
