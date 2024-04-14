// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	// Essentially this line tells the drive to drive with values from the
	// joysticks if you aren't doing anything else.
	m_drive.SetDefaultCommand(m_drive.drive_command(
		[this]() { return m_driverController.GetLeftX(); },
		[this]() { return -m_driverController.GetLeftY(); },
		[this]() { return m_driverController.GetRightY(); }
	));

	// Tell the elevator to go to 1.0 if it doesn't have a note and 400.0 
	// if it does to prepare for the shot.
	m_shooter.SetDefaultCommand(m_shooter.track_elevator_command(
		[this]() {
			if (m_shooter.has_note()) {
				return Constants::Shooter::k_high_setpoint;
			} else {
				return Constants::Shooter::k_pickup_setpoint;
			}
		}
	));

	// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
	frc2::Trigger([this] {
			return m_subsystem.ExampleCondition();
			}).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

	// Attempt to pick up one note when the X button is held.
	// Shedules once on the button is pressed then terminates either
	// when it is let go or when a note is picked up.
	m_driverController.X().WhileTrue(m_shooter.pickup_command().ToPtr());
	// Schedule shoot routine when we hold down Y.
	m_driverController.Y().WhileTrue(m_shots.teleop_shoot_command());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	// An example command will be run in autonomous
	return autos::ExampleAuto(&m_subsystem);
}
