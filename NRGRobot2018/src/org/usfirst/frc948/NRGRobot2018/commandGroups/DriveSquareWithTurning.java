package org.usfirst.frc948.NRGRobot2018.commandGroups;

import org.usfirst.frc948.NRGRobot2018.commands.DriveStraightTimed;
import org.usfirst.frc948.NRGRobot2018.commands.TurnToHeading;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveSquareWithTurning extends CommandGroup {
	public DriveSquareWithTurning() {
		addSequential(new DriveStraightTimed(0.5,2.0));
		addSequential(new TurnToHeading(90));
		addSequential(new DriveStraightTimed(0.5,2.0));
		addSequential(new TurnToHeading(180));
		addSequential(new DriveStraightTimed(0.5,2.0));
		addSequential(new TurnToHeading(270));
		addSequential(new DriveStraightTimed(0.5,2.0));
		addSequential(new TurnToHeading(360));
	} 
}