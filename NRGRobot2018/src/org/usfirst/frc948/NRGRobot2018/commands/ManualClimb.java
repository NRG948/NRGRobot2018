package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualClimb extends Command {
	double power;


	public ManualClimb(double power) {
		requires(Robot.climber);
		this.power = power;
	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotMap.climberMotor.set(power);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
