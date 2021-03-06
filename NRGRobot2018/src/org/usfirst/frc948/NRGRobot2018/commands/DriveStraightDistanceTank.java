package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.utilities.PositionTracker;
import org.usfirst.frc948.NRGRobot2018.utilities.SimplePIDController;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightDistanceTank extends Command {
	private final static double DISTANCE_P = 0.03;
	private final static double DISTANCE_I = 0.0125;
	private final static double DISTANCE_D = 0.0047;

	public double distance;
	public double power;
	private double desiredHeading;
	private double distanceTolerance;
	private double initialY;

	private SimplePIDController distancePIDController;
	private int cyclesOnTarget;

	public DriveStraightDistanceTank(double power, double distance, double distanceTolerance) {
		requires(Robot.drive);

		this.power = power;
		this.distance = distance;
		this.distanceTolerance = distanceTolerance;
	}

	public DriveStraightDistanceTank(double power, double distance) {
		this(power, distance, 2.0);
	}

	@Override
	protected void initialize() {
		System.out.println("DriveStraightDistanceTank init");
		desiredHeading = Robot.drive.getDesiredHeading();
		Robot.drive.tankDriveOnHeadingPIDInit(desiredHeading, power);

		distancePIDController = new SimplePIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D).setOutputRange(-1, 1)
				.setAbsoluteTolerance(distanceTolerance).setSetpoint(distance).start();

		initialY = PositionTracker.getMechY();
//		initialY = RobotMap.yEncoder.getDistance();
	}

	@Override
	protected void execute() {
		double currY = PositionTracker.getMechY();
//		double currY = RobotMap.yEncoder.getDistance();
		double dY = currY - initialY;

		double distancePIDOutput = distancePIDController.update(dY);

		double revisedPower = power * distancePIDOutput;
		double error = distancePIDController.getError();

		if (Math.abs(error) < distanceTolerance) {
			revisedPower = 0.0;
		} 
		else if (Math.abs(error) < 9.0) {
			revisedPower = 0.25 * Math.signum(error);
		}

		Robot.drive.tankDriveOnHeadingPIDExecute(revisedPower);

		SmartDashboard.putNumber("DriveStraightDistanceTank/Revised Power", revisedPower);
		SmartDashboard.putNumber("DriveStraightDistanceTank/Distance PID ERROR", error);
	}

	// Finishes the command if the target distance has been exceeded
	protected boolean isFinished() {
		if (Math.abs(distancePIDController.getError()) < distanceTolerance) {
			cyclesOnTarget++;
		} else {
			cyclesOnTarget = 0;
		}

		SmartDashboard.putNumber("DriveStraightDistanceTank/Cycles On Target", cyclesOnTarget);
		return cyclesOnTarget >= 7;
	}

	protected void end() {
		Robot.drive.stop();
		System.out.println("DriveStraightDistanceTank End");

		double lf = RobotMap.leftFrontEncoder.getDistance();
		double lr = RobotMap.leftRearEncoder.getDistance();
		double rf = RobotMap.rightFrontEncoder.getDistance();
		double rr = RobotMap.rightRearEncoder.getDistance();
		
		double min = Math.min(Math.min(lf, lr), Math.min(rr, rf));
		
		if (min == 0) {
			System.out.println("Broken encoder");
		} else {
			System.out.println("LF: " + min/lf + ", RF: " + min/rf + " LR: " + min/lr + " RR: " + min/rr);
		}
	}

	protected void interrupted() {
		end();
	}
}
