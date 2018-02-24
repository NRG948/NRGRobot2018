package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightDistanceNoPID extends Command {
	
	double distance;
	double currentdistance;
	private double startEncoderLeftFront;
	private double xEncoder;
	private double startEncoderRightFront;
	private double yEncoder;

    public DriveStraightDistanceNoPID(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("DriveStraighDistanceNoPID("+distance+")");
    	xEncoder = RobotMap.xEncoder.getDistance();
		yEncoder = RobotMap.yEncoder.getDistance();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	double distanceTravelledx = RobotMap.xEncoder.getDistance() - xEncoder;
		double distanceTravelledy = RobotMap.yEncoder.getDistance() - yEncoder;
		double yDisplacement = ((distanceTravelledx + distanceTravelledy)) / 2;
		currentdistance = Math.abs(yDisplacement);
    	Robot.drive.rawDriveCartesian(0, 1, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return(currentdistance >= distance);
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("DriveStraighDistanceNoPID.end()");
    	Robot.drive.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
