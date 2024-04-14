// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShotFactory.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

extern bool is_red;

ShotFactory::ShotFactory(Drive *m_drive, Shooter *m_shooter)
{

}

frc2::CommandPtr ShotFactory::prep_teleop_shot_command()
{
	return frc2::cmd::Race(
		m_drive->track_target_command(),
		m_shooter->track_command([this]() {
			return m_drive->get_target_dist();
		}),
		frc2::cmd::Parallel(
			m_drive->wait_for_angle_command().ToPtr(),
			m_shooter->wait_shooter_command().ToPtr()
		)
	);
}

frc2::CommandPtr ShotFactory::teleop_shoot_command()
{
	return frc2::cmd::Sequence(
		prep_teleop_shot_command(),
		m_shooter->shoot_command()
	);
}

frc2::CommandPtr ShotFactory::prep_auto_shot_command(frc::Pose2d shot_pos)
{
	return frc2::cmd::Race(
		m_drive->track_position_command(shot_pos),
		m_shooter->track_command([shot_pos]() {
			units::meter_t dist, x, y;
			
			if (is_red) {
				x = Constants::Positions::k_red_speaker_x - shot_pos.X();
			} else {
				x = Constants::Positions::k_blue_speaker_y - shot_pos.X();
			}
			y = Constants::Positions::k_red_speaker_y;
			dist = (units::meter_t)std::sqrt(
				(x * x + y * y).value()
			);

			return dist;
		}),
		frc2::cmd::Parallel(
			m_drive->wait_for_pos_command().ToPtr(),
			m_shooter->wait_shooter_command().ToPtr()
		)
	);
}

frc2::CommandPtr ShotFactory::auto_shoot_command(frc::Pose2d shot_pos)
{
	return frc2::cmd::Sequence(
		prep_auto_shot_command(shot_pos),
		m_shooter->shoot_command()
	);
}
