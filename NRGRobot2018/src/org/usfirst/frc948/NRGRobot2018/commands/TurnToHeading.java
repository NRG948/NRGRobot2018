package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToHeading extends Command {

	double targetHeading;
	
    public TurnToHeading(double targetHeading) {
    	this.targetHeading = targetHeading;
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.driveHeadingPIDInit(targetHeading, 1.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.driveHeadingPIDExecute(0.0, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drive.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.driveHeadingPIDEnd();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
