package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToXYNoPID extends Command {
	final double DISTANCE_TO_SLOW_DOWN = 9.0; // in inches
	final double DISTANCE_TOLERANCE = 1.0;
	final double ANGLE_TO_SLOW_DOWN = 20.0; // in degrees
	final double ANGLE_TOLERANCE = 2.0;
	
	double desiredX;
	double desiredY;
	
	private double distanceToTravel; // in inches
	private double angleToTurn; // in degrees

	public DriveToXYNoPID(double x, double y) {
		requires(Robot.drive);

		desiredX = x;
		desiredY = y;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		distanceToTravel = Double.MAX_VALUE;
		angleToTurn = Double.MAX_VALUE;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double currentX = Robot.positionTracker.getX();
		double currentY = Robot.positionTracker.getY();
		double dX = desiredX - currentX;
		double dY = desiredY - currentY;
		
		distanceToTravel = Math.sqrt((dX * dX) + (dY * dY));
		angleToTurn = Math.toDegrees(Math.atan2(dX, dY)) - RobotMap.gyro.getAngle();

		double drivePower = Math.min(1.0, distanceToTravel / DISTANCE_TO_SLOW_DOWN);
		double turnPower = Math.copySign(Math.min(1.0, Math.abs(angleToTurn) / ANGLE_TO_SLOW_DOWN), angleToTurn);
		
		Robot.drive.rawDriveCartesian(0, drivePower, turnPower);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (distanceToTravel <= DISTANCE_TOLERANCE && Math.abs(angleToTurn) <= ANGLE_TOLERANCE);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
