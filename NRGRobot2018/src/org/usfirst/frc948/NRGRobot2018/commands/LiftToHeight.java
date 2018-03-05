package org.usfirst.frc948.NRGRobot2018.commands;

import java.util.EnumMap;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter;
import org.usfirst.frc948.NRGRobot2018.utilities.LifterLevel;
import org.usfirst.frc948.NRGRobot2018.utilities.PreferenceKeys;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter;

/**
 *
 */
public class LiftToHeight extends Command {
	private final LifterLevel level;

	public LiftToHeight(LifterLevel level) {
		requires(Robot.cubeLifter);
		this.level = level;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.cubeLifter.liftToHeightPIDInit(level.getTicks(), 1);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.cubeLifter.liftToHeightPIDExecute();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.cubeLifter.lifterPIDControllerOnTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeLifter.liftToHeightPIDEnd();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
