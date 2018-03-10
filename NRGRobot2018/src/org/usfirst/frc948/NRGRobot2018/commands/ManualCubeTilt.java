package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeTilter;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualCubeTilt extends Command {

	public ManualCubeTilt() {
		requires(Robot.cubeTilter);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (OI.isXBoxDPadUp()) {
			Robot.cubeTilter.tiltUp();
		} else if (OI.isXBoxDPadDown()) {
			Robot.cubeTilter.tiltDown();
		}else{
			Robot.cubeTilter.stop();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeTilter.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
