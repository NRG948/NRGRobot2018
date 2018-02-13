package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualCubeLift extends Command {
	private final double LIFT_POWER_SCALE_UP = 1;
	private final double LIFT_POWER_SCALE_DOWN = 0.3;

    public ManualCubeLift() {
    	requires(Robot.cubeLifter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double upSpeed = OI.getTriggerL() * LIFT_POWER_SCALE_UP;
    	double downSpeed = OI.getTriggerR() * LIFT_POWER_SCALE_DOWN;
    	 
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
