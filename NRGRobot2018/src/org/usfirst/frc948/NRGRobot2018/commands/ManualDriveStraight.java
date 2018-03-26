package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualDriveStraight extends Command {
	
	private double currentHeading;

    public ManualDriveStraight() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currentHeading = RobotMap.gyro.getAngle();
    	Robot.drive.tankDriveOnHeadingPIDInit(RobotMap.gyro.getAngle(), 1.0);
    	System.out.println("ManualDriveStraight init, currentHeading:" + currentHeading);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.tankDriveOnHeadingPIDExecute(OI.getRightJoystickY());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.driveHeadingPIDEnd();
    	System.out.println("ManualDriveStraight End");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
