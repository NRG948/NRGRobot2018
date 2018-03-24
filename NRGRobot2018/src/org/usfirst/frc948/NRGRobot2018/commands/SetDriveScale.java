package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetDriveScale extends Command {
	
	private double scale;

    public SetDriveScale(double s) {
        requires(Robot.drive);
        scale = s;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("SetDriveScale init");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.setScale(scale);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("SetDriveScale End");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
;