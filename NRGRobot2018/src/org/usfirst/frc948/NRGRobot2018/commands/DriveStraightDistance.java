package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Units are in inches
public class DriveStraightDistance extends Command {
	private boolean powerInY = false;
	private double startX;
	private double startY;
	private double power;
	private double distance;
	private double distanceTravelled;

	public DriveStraightDistance(double power, double distance, Drive.Direction direction) {
		this.distance = Math.abs(distance);

		switch (direction) {
		case FORWARD:
			this.power = Math.abs(power);
			powerInY = true;
			break;
		case BACKWARD:
			this.power = -Math.abs(power);
			powerInY = true;
			break;
		case LEFT:
			this.power = -Math.abs(power);
			powerInY = false;
			break;
		case RIGHT:
			this.power = Math.abs(power);
			powerInY = false;
			break;
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		startX = Robot.positionTracker.getX();
		startY = Robot.positionTracker.getY();

		Robot.drive.driveHeadingPIDInit(RobotMap.gyro.getAngle(), 2.0);
		SmartDashboard.putNumber("startX", startX);
		SmartDashboard.putNumber("startY", startY);
	}

	protected void execute() {
		distanceTravelled = calculateDistanceTravelled();
		if (powerInY) {
			power *= Math.min(1.0, (distance - Math.min(distance, distanceTravelled)) / 9.0);
			Robot.drive.driveHeadingPIDExecute(0, power);
		} else {
			Robot.drive.driveHeadingPIDExecute(power, 0);
		}
	}

	private double calculateDistanceTravelled() {
		double currentX = Robot.positionTracker.getX();
		double currentY = Robot.positionTracker.getY();

		double deltaX = currentX - startX;
		double deltaY = currentY - startY;

		double travel = Math.sqrt((deltaY * deltaY) + (deltaX * deltaX));

		SmartDashboard.putNumber("distance travelled", travel);
		SmartDashboard.putNumber("deltaX", deltaX);
		SmartDashboard.putNumber("deltaY", deltaY);
		SmartDashboard.putNumber("currentX", currentX);
		SmartDashboard.putNumber("currentY", currentY);

		return travel;
	}

	protected boolean isFinished() {
		return (distanceTravelled >= distance);
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
