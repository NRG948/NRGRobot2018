package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ResetSensors extends Command {

    public ResetSensors() {
    	requires(Robot.drive);
    	requires(Robot.climber);
    	requires(Robot.cubeAcquirer);
    	requires(Robot.cubeTilter);
    	requires(Robot.cubeLifter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("ResetSensors init");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.navx.reset();
    	RobotMap.xEncoder.reset();
    	RobotMap.yEncoder.reset();
    	Robot.positionTracker.reset();
    	
    	RobotMap.cubeLiftEncoder.reset();
    	RobotMap.cubeTiltEncoder.reset();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return Math.abs(RobotMap.navx.getAngle()) < 0.5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("ResetSensors end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
