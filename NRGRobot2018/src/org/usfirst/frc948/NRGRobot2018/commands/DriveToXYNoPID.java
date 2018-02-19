package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.utilities.MathUtil;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveToXYNoPID extends Command {
	final double X_DISTANCE_TO_SLOW_DOWN = 6.0; // in inches
	final double Y_DISTANCE_TO_SLOW_DOWN = 15.0;
	final double DISTANCE_TOLERANCE = 5.0;
	
	final double ANGLE_TO_SLOW_DOWN = 20.0; // in degrees
	final double ANGLE_TOLERANCE = 5.0;
	
	double desiredX;
	double desiredY;
	double desiredHeading;
	
	private double dX; // in inches
	private double dY; // in inches
	private double dHeading; // in inches

	public DriveToXYNoPID(double x, double y, double heading) {
		requires(Robot.drive);

		desiredX = x;
		desiredY = y;
		desiredHeading = heading;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		dX = Double.MAX_VALUE;
		dY = Double.MAX_VALUE;
		dHeading = Double.MAX_VALUE;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double currentX = Robot.positionTracker.getX();
		double currentY = Robot.positionTracker.getY();
		double currentHeading = RobotMap.gyro.getAngle();
		dX = desiredX - currentX;
		dY = desiredY - currentY;
		dHeading = desiredHeading - currentHeading;
		
		SmartDashboard.putNumber("driveToXYHeading/dX", dX);
		SmartDashboard.putNumber("driveToXYHeading/dY", dY);
		SmartDashboard.putNumber("driveToXYHeading/dHeading", dHeading);

		

		double xPower = MathUtil.clamp(dX / X_DISTANCE_TO_SLOW_DOWN, -0.9, 0.9);
		double yPower = MathUtil.clamp(dY / Y_DISTANCE_TO_SLOW_DOWN, -0.6, 0.6);
		double turnPower = MathUtil.clamp(dHeading / ANGLE_TO_SLOW_DOWN, -1.0, 1.0);
		
		Robot.drive.rawDriveCartesian(xPower, yPower, turnPower);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Math.abs(dX) <= DISTANCE_TOLERANCE && 
				Math.abs(dY) <= DISTANCE_TOLERANCE && 
				Math.abs(dHeading) <= ANGLE_TOLERANCE);
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
