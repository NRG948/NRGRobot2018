package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.utilities.PreferenceKeys;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToXYHeadingPIDTest extends Command {
	private Command command;

    public DriveToXYHeadingPIDTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	double desiredX = Robot.preferences.getDouble(PreferenceKeys.DRIVE_XYH_X, 48.0);
		double desiredY = Robot.preferences.getDouble(PreferenceKeys.DRIVE_XYH_Y, 48.0);
		double desiredHeading = Robot.preferences.getDouble(PreferenceKeys.DRIVE_XYH_H, 0);
		
		command = new DriveToXYHeadingPID(desiredX,desiredY,desiredHeading);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	command.start();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
