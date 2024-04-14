// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include "frc2/command/FunctionalCommand.h"

#include <frc2/command/Commands.h>

Shooter::Shooter()
{
	m_left_motor.SetInverted(false);
	m_right_motor.SetInverted(true);
	m_top_motor.SetInverted(true);
	m_low_motor.SetInverted(false);
	m_ele_motor.SetInverted(true);
	
	m_ele_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
	m_ele_motor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 500.0);
	m_ele_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
	m_ele_motor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0.0);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {}

frc2::CommandPtr Shooter::track_command(std::function<units::meter_t()> dist)
{
	return this->Run(
		[this, dist]() { track(dist()); }
	);
}

frc2::CommandPtr Shooter::track_elevator_command(std::function<double()> setpoint)
{
	return this->Run(
		[this, setpoint]() {
			double ele_power = m_ele_pid.Calculate(
					m_ele_encoder.GetPosition(),
					setpoint()
			);
			m_ele_motor.SetVoltage((units::volt_t)ele_power);
		}
	);
}

frc2::CommandPtr Shooter::shoot_command()
{
	return frc2::cmd::Race(
		this->Run([this]() {
			m_top_motor.Set(1.0);
			m_low_motor.Set(1.0);
		}),
		frc2::cmd::Wait(2.0_s)
	);
}

frc2::FunctionalCommand Shooter::zero_command()
{
	static frc2::FunctionalCommand zero_elevator = frc2::FunctionalCommand(
		[this]() {
			m_ele_motor.Set(-1.0);
			m_ele_motor.EnableSoftLimit(
				rev::CANSparkMax::SoftLimitDirection::kReverse,
				false
			);
		},
		[]() {},
		[this](bool interrupted) {
			m_ele_motor.Set(0.0);
			m_ele_motor.SetSoftLimit(
				rev::CANSparkMax::SoftLimitDirection::kReverse,
				false
			);
		},
		[this]() { return m_ele_limit.Get(); },
		{this}
	);

	return zero_elevator;
}

frc2::FunctionalCommand Shooter::pickup_command()
{
	static frc2::FunctionalCommand pickup = frc2::FunctionalCommand(
		[this]() {
			m_top_motor.Set(1.0);
			m_low_motor.Set(1.0);
		},
		[]() {},
		[this](bool interrupted) {
			m_top_motor.Set(0.0);
			m_low_motor.Set(0.0);
		},
		[this]() { return has_note(); },
		{this}
	);

	return pickup;
}

frc2::FunctionalCommand Shooter::wait_shooter_command()
{
	static frc2::FunctionalCommand command = frc2::FunctionalCommand(
		[]() {},
		[]() {},
		[](bool interrupted) {},
		[this]() {
			bool ready = true;
			ready = ready && m_left_pid.AtSetpoint();
			ready = ready && m_right_pid.AtSetpoint();
			ready = ready && m_ele_pid.AtSetpoint();
			return ready;
		}
	);

	return command;
}

void Shooter::track(units::meter_t dist)
{
	double ele_setpoint = dist_to_setpoint(dist);
	double ele_power = m_ele_pid.Calculate(m_ele_encoder.GetPosition(), ele_setpoint);
	double left_power = m_left_pid.Calculate(m_left_encoder.GetVelocity(), 5000);
	left_power += (double)m_left_ff.Calculate(5000_rpm);
	double right_power = m_right_pid.Calculate(m_right_encoder.GetVelocity(), 5000);
	right_power += (double)m_right_ff.Calculate(5000_rpm);

	m_ele_motor.SetVoltage((units::volt_t)ele_power);
	m_left_motor.SetVoltage((units::volt_t)left_power);
	m_right_motor.SetVoltage((units::volt_t)right_power);
}

double Shooter::dist_to_setpoint(units::meter_t dist)
{
	return dist.value();
}

