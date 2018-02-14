package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Units are in inches
public class DriveStraightDistance extends Command {
	private boolean powerInY = false;
	private double startEncoderLeftFront;
	private double startX;
	private double startY;
	private double startEncoderRightBack;
	private double distanceTravelled = 0.0;

	private double power;
	private double distance;
	private Drive.Direction direction;
	private double currentHeading;

	public DriveStraightDistance(double power, double distance, Drive.Direction direction) {
		this.direction = direction;
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
		currentHeading = RobotMap.gyro.getAngle();
		distanceTravelled = 0.0;
		startX = RobotMap.xEncoder.getDistance();
		startY = RobotMap.yEncoder.getDistance();
		Robot.drive.driveHeadingPIDInit(RobotMap.gyro.getAngle(), 2.0);
		System.out.println(String.format("DriveStraightDistance()", power, distance, direction));
	}

	protected void execute() {
		double deltaX = RobotMap.xEncoder.getDistance() - startX;
		double deltaY = RobotMap.yEncoder.getDistance() - startY;
		double distanceTraveled = Math.sqrt((deltaY * deltaY) + (deltaX * deltaX));

		// double rot = ((distanceTravelledLF + distanceTravelledLB) -
		// (distanceTravelledRF + distanceTravelledRB)) / 4;
		SmartDashboard.putNumber("x displacement", distanceTraveled);
		if (powerInY) {
			Robot.drive.driveHeadingPIDExecute(0, power);
		} else {
			Robot.drive.driveHeadingPIDExecute(power, 0);
		}
		SmartDashboard.putNumber("distance travelled", distanceTravelled);
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
