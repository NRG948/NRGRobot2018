package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TiltAcquirerDown extends Command {
	double delay;
	
	Timer timer;
	
    public TiltAcquirerDown(double delay) {
    	requires(Robot.cubeTilter);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.delay = delay;
    	
    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.start();
    	System.out.println("TiltAcquirerDown init");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.cubeTilter.tiltDown();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.hasPeriodPassed(delay);
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("TiltAcquirerDown end");
    	Robot.cubeTilter.stop();
    	timer.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("TiltAcquirerDown interrupted");
    	end();
    }
}
