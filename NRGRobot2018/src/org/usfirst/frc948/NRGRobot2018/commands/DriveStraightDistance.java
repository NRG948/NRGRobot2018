package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import org.usfirst.frc948.NRGRobot2018.RobotMap;

import org.usfirst.frc948.NRGRobot2018.subsystems.Drive;

import org.usfirst.frc948.NRGRobot2018.utilities.MathUtil;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Units are in inches

public class DriveStraightDistance extends Command {
	private final double desiredDistance; // in inches
	private final double maxPower;
	private final boolean powerInY;
	private double startX;
	private double startY;
	private double distanceTravelled;

	public DriveStraightDistance(double power, double distance, Drive.Direction direction) {
		requires(Robot.drive);
		this.desiredDistance = Math.abs(distance);
		switch (direction) {
		case FORWARD:
			this.maxPower = Math.abs(power);
			powerInY = true;
			break;

		case BACKWARD:
			this.maxPower = -Math.abs(power);
			powerInY = true;
			break;

		case LEFT:
			this.maxPower = -Math.abs(power);
			powerInY = false;
			break;

		case RIGHT:
			this.maxPower = Math.abs(power);
			powerInY = false;
			break;

		default:
			throw new IllegalStateException("invalid direction");

		}
	}

	protected void initialize() {
		startX = Robot.positionTracker.getX();
		startY = Robot.positionTracker.getY();

		distanceTravelled = 0.0;
		Robot.drive.tankDriveOnHeadingPIDInit(maxPower, RobotMap.gyro.getAngle());
		Robot.drive.driveHeadingPIDInit(RobotMap.gyro.getAngle(), 2.0);

		SmartDashboard.putNumber("DriveStraightDistance/startX", startX);
		SmartDashboard.putNumber("DriveStraightDistance/startY", startY);
		System.out.println("DriveStraightDistance");

	}

	protected void execute() {
		distanceTravelled = calculateDistanceTravelled();

		if (powerInY) {
			double remainingDistance = desiredDistance - distanceTravelled;
			// start slowing down at 9in. away from target
			double calculatedPower = maxPower * MathUtil.clamp(remainingDistance / 9.0, 0, 1);

			Robot.drive.driveHeadingPIDExecute(0, calculatedPower);

			SmartDashboard.putNumber("DriveStraightDistance/power", calculatedPower);
		} else {
			Robot.drive.driveHeadingPIDExecute(maxPower, 0);

			SmartDashboard.putNumber("DriveStraightDistance/power", maxPower);
		}

	}

	protected boolean isFinished() {
		return (distanceTravelled >= desiredDistance);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.driveHeadingPIDEnd();

		System.out.println("DriveStraightDistance End");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private double calculateDistanceTravelled() {
		double currX = Robot.positionTracker.getX();
		double currY = Robot.positionTracker.getY();

		double dX = currX - startX;
		double dY = currY - startY;

		double distanceTravelled = Math.sqrt((dY * dY) + (dX * dX));

		SmartDashboard.putNumber("DriveStraightDistance/distance travelled", distanceTravelled);
		SmartDashboard.putNumber("DriveStraightDistance/deltaX", dX);
		SmartDashboard.putNumber("DriveStraightDistance/deltaY", dY);
		SmartDashboard.putNumber("DriveStraightDistance/currentX", currX);
		SmartDashboard.putNumber("DriveStraightDistance/currentY", currY);

		return distanceTravelled;
	}
}