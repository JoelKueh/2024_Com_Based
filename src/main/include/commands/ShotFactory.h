// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Drive.h"
#include "subsystems/Shooter.h"

class ShotFactory {
public:
	ShotFactory(Drive *m_drive, Shooter *m_shooter);

	frc2::CommandPtr teleop_shoot_command();
	frc2::CommandPtr auto_shoot_command(frc::Pose2d shot_pos);

private:
	Drive *m_drive;
	Shooter *m_shooter;

	frc2::CommandPtr prep_teleop_shot_command();
	frc2::CommandPtr prep_auto_shot_command(frc::Pose2d shot_pos);
};
