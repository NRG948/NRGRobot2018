 package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotorAcquire extends Command {
	
	private double aqSpeed;

    public MotorAcquire(double aqSpeed) {
    	this.aqSpeed = aqSpeed;
    	requires(Robot.acquirer);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.aqVictorL.set(-aqSpeed);
    	RobotMap.aqVictorR.set(aqSpeed);
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
