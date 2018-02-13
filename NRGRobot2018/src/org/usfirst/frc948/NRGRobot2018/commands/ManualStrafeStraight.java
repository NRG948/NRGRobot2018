package src.org.usfirst.frc948.NRGRobot2018.commands;

import edu.wpi.first.wpilibj.command.Command;
import src.org.usfirst.frc948.NRGRobot2018.OI;
import src.org.usfirst.frc948.NRGRobot2018.Robot;
import src.org.usfirst.frc948.NRGRobot2018.RobotMap;

/**
 *
 */
public class ManualStrafeStraight extends Command {
	
	private double currentHeading;

    public ManualStrafeStraight() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currentHeading = RobotMap.gyro.getAngle();
    	Robot.drive.driveHeadingPIDInit(currentHeading, 1.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.driveHeadingPIDExecute(OI.getX(), 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.driveHeadingPIDEnd();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
