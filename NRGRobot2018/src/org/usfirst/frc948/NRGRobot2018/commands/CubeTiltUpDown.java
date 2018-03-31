package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Fallback if tilter PID doesn't work.
 * If tilter is commanded down, command ends after delay.
 * Else, command is run until interrupted and pulsed every other second after the delay
 * This command is super sketch, will need to be tested.
 */
public class CubeTiltUpDown extends Command {
	Timer timer;
	double pulseStartTime;

	double power;
	double delay;
	
	// time in seconds
	public CubeTiltUpDown(double power, double delay) {
		requires(Robot.cubeTilter);
		
		this.power = power;
		this.delay = delay;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	pulseStartTime = Double.MAX_VALUE;
    	
    	timer = new Timer();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (timer.hasPeriodPassed(delay)) {
    		if (pulseStartTime == Double.MAX_VALUE) {
    			pulseStartTime = timer.get();
    		}
    		
			double currTime = timer.get();
			if ((currTime - pulseStartTime) % 2 >= 1) {
				Robot.cubeTilter.rawTilt(power);
			} else {
				Robot.cubeTilter.stop();
			}
    	} else {
    		Robot.cubeTilter.rawTilt(power);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (power <= 0) return timer.hasPeriodPassed(delay);
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
