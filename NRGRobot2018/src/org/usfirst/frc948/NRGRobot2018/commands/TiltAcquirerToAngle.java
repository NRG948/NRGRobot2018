package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TiltAcquirerToAngle extends Command {
	private final double ticks;

	public TiltAcquirerToAngle(double ticks) {
		requires(Robot.cubeTilter);
		this.ticks = ticks;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.cubeTilter.tiltToAnglePIDIntialize(ticks, 50);
		System.out.println("TiltAcquirerToAngle init: " + ticks);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.cubeTilter.tiltToAnglePIDExecute();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.cubeTilter.tiltToAnglePIDOnTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("TiltAcquirerToAngle end");
		Robot.cubeLifter.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("TiltAcquirerToAngle interrupted");
		end();
	}
}
