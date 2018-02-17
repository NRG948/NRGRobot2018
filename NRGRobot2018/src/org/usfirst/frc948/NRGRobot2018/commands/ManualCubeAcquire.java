package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualCubeAcquire extends Command {

    public ManualCubeAcquire() {
        requires(Robot.cubeAcquirer);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double leftPower = -OI.xboxController.getY(Hand.kLeft);
    	double rightPower = OI.xboxController.getY(Hand.kRight);
    	Robot.cubeAcquirer.rawAcquire(leftPower, rightPower);
    	// TODO: Left acq motor needs to be calibrated (subtracting power is bad fix). This has happened in the past
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
