package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StrafeStraightTimed extends Command {
	
	private double power;
	private double timeToDrive;
	private Timer timer = new Timer();
	private double currentHeading;

    public StrafeStraightTimed(double power, double timeToDrive) {
    	requires(Robot.drive);
    	this.power = power;
    	this.timeToDrive = timeToDrive;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currentHeading = RobotMap.gyro.getAngle();
    	Robot.drive.driveHeadingPIDInit(currentHeading, 1.0);
    	timer.start();
    	
    	System.out.println("StrafeStraightTimed int");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.driveHeadingPIDExecute(power, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return timer.hasPeriodPassed(timeToDrive);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.driveHeadingPIDEnd();
    	timer.stop();
    	System.out.println("StrafeStraightTimed End");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
