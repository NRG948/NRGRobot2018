package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SoftShootCube extends Command {
	final double SOFT_SHOOT_POWER = 0.3;
	
    public SoftShootCube() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cubeAcquirer);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("SoftShootCube init");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.cubeAcquirer.rawAcquire(SOFT_SHOOT_POWER, SOFT_SHOOT_POWER);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.cubeAcquirer.stop();
    	System.out.println("SoftShootCube end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("SoftShootCube interrupted");
    	end();
    }
}
