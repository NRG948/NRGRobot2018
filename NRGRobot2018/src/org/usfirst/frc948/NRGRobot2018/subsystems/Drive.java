// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualDrive;
import org.usfirst.frc948.NRGRobot2018.utilities.MathUtil;
import org.usfirst.frc948.NRGRobot2018.utilities.PreferenceKeys;
import org.usfirst.frc948.NRGRobot2018.utilities.SimplePIDController;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sun.java2d.DestSurfaceProvider;

/**
 *
 */
public class Drive extends Subsystem implements PIDOutput {
	public enum DriveDirection {
		FORWARD, BACKWARD, LEFT, RIGHT
	}

	// private PIDController drivePIDController;
	private SimplePIDController xPIDController;
	private SimplePIDController yPIDController;
	private SimplePIDController turnPIDController;
	private SimplePIDController drivePIDController;
	private SimplePIDController driveStraightOnHeadingPIDController;
	
	private int cyclesOnTarget;
	private volatile double drivePIDOutput = 0;
	private static final int REQUIRED_CYCLES_ON_TARGET = 3;

	public final double DRIVE_STRAIGHT_ON_HEADING_P = 0.02;
	public final double DRIVE_STRAIGHT_ON_HEADING_I = 0.005 * 20;
	public final double DRIVE_STRAIGHT_ON_HEADING_D = 0.02 / 20;
	private final double PID_MIN_OUTPUT = 0.08;
	private final double PID_MAX_OUTPUT = 1;
	
	public final static double DEFAULT_TURN_P = 0.09;
	public final static double DEFAULT_TURN_I = 0.6 / 20;
	public final static double DEFAULT_TURN_D = 0.0135 * 20;

	public final static double DEFAULT_DRIVE_Y_P = 0.24;
	public final static double DEFAULT_DRIVE_Y_I = 0.6;
	public final static double DEFAULT_DRIVE_Y_D = 0.055;

	public final static double DEFAULT_DRIVE_X_P = 0.5;
	public final static double DEFAULT_DRIVE_X_I = 0.5;
	public final static double DEFAULT_DRIVE_X_D = 0;

	public final static double DEFAULT_DRIVE_X_POWER = 1.0;
	public final static double DEFAULT_DRIVE_Y_POWER = 0.8;
	public final static double DEFAULT_DRIVE_TURN_POWER = 1.0;

	public final static double SCALE_HIGH = 1.0;
	public final static double SCALE_LOW = 0.5;
	private double scale = SCALE_HIGH;

	private double maxDriveAccel = 1.0;
	public static final double DEF_TELEOP_DRIVE_ACCEL_MAX_LIFT_HEIGHT = 0.045;
	public static final double DEF_AUTO_MAX_DRIVE_ACCEL = 0.03;

	private double lastVelX = 0.0;
	private double lastVelY = 0.0;
	
	private double desiredHeading = 0.0;
	private double lastPower = 0;

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}

	private SimplePIDController createPIDController(double setpoint, double tolerance, double xP, double xI, double xD,
			double xMaxPower) {
		return new SimplePIDController(xP, xI, xD).setOutputRange(-xMaxPower, xMaxPower).setAbsoluteTolerance(tolerance)
				.setSetpoint(setpoint).start();
	}

	public void drivePIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
		drivePIDController = new SimplePIDController(p, i, d, false);

		drivePIDController.setOutputRange(-0.5, 0.5);
		drivePIDController.setAbsoluteTolerance(tolerance);
		drivePIDController.setSetpoint(setpoint);

		drivePIDController.start();
		cyclesOnTarget = 0;
	}

	public void xPIDControllerInit(double setpoint, double tolerance) {
		double xP = Robot.preferences.getDouble(PreferenceKeys.DRIVE_X_P, DEFAULT_DRIVE_X_P);
		double xI = Robot.preferences.getDouble(PreferenceKeys.DRIVE_X_I, DEFAULT_DRIVE_X_I);
		double xD = Robot.preferences.getDouble(PreferenceKeys.DRIVE_X_D, DEFAULT_DRIVE_X_D);
		double xMaxPower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_X_MAX_POWER, DEFAULT_DRIVE_X_POWER);

		xPIDController = createPIDController(setpoint, tolerance, xP, xI, xD, xMaxPower);

		cyclesOnTarget = 0;
	}

	public void yPIDControllerInit(double setpoint, double tolerance) {
		double yP = Robot.preferences.getDouble(PreferenceKeys.DRIVE_Y_P, DEFAULT_DRIVE_Y_P);
		double yI = Robot.preferences.getDouble(PreferenceKeys.DRIVE_Y_I, DEFAULT_DRIVE_Y_I);
		double yD = Robot.preferences.getDouble(PreferenceKeys.DRIVE_Y_D, DEFAULT_DRIVE_Y_D);
		double yMaxPower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_Y_MAX_POWER, DEFAULT_DRIVE_Y_POWER);
		yPIDController = createPIDController(setpoint, tolerance, yP, yI, yD, yMaxPower);
	}

	public void turnPIDControllerInit(double setpoint, double tolerance) {
		double turnP = Robot.preferences.getDouble(PreferenceKeys.TURN_P_TERM, DEFAULT_TURN_P);
		double turnI = Robot.preferences.getDouble(PreferenceKeys.TURN_I_TERM, DEFAULT_TURN_I);
		double turnD = Robot.preferences.getDouble(PreferenceKeys.TURN_D_TERM, DEFAULT_TURN_D);
		double turnMaxPower = Robot.preferences.getDouble(PreferenceKeys.DRIVE_TURN_MAX_POWER,
				DEFAULT_DRIVE_TURN_POWER);

		turnPIDController = createPIDController(setpoint, tolerance, turnP, turnI, turnD, turnMaxPower);
		desiredHeading = setpoint;
	}

	public void driveHeadingPIDInit(double desiredHeading, double tolerance) {
		drivePIDControllerInit(Robot.preferences.getDouble(PreferenceKeys.TURN_P_TERM, DEFAULT_TURN_P),
				Robot.preferences.getDouble(PreferenceKeys.TURN_I_TERM, DEFAULT_TURN_I),
				Robot.preferences.getDouble(PreferenceKeys.TURN_D_TERM, DEFAULT_TURN_D), desiredHeading, tolerance);
	}

	public double turnPIDControllerExecute(double dTurn) {
		return turnPIDController.update(dTurn);
	}

	public double xPIDControllerExecute(double dX) {
		return xPIDController.update(dX);
	}

	public double yPIDControllerExecute(double dY) {
		return yPIDController.update(dY);
	}

	public void driveHeadingPIDExecute(double velX, double velY) {
		double pidOutput = drivePIDController.update(RobotMap.gyro.getAngle());

		SmartDashboard.putNumber("Turn To Heading PID Error", drivePIDController.getError());
		SmartDashboard.putNumber("Turn To Heading PID Output", pidOutput);

		rawDriveCartesian(velX, velY, pidOutput);
	}

	public void driveHeadingPIDEnd() {
		drivePIDController = null;
		stop();
		SmartDashboard.putNumber("DriveOnHeadingEnd gyro", RobotMap.gyro.getAngle());
	}

	public boolean drivePIDControllerOnTarget() {
		if (drivePIDController.onTarget()) {
			cyclesOnTarget++;
		} else {
			cyclesOnTarget = 0;
		}
		return cyclesOnTarget >= REQUIRED_CYCLES_ON_TARGET;
	}

	public boolean xPIDControllerOnTarget() {
		return xPIDController.onTarget();
	}

	public boolean yPIDControllerOnTarget() {
		return yPIDController.onTarget();
	}

	public boolean turnPIDControllerOnTarget() {
		return turnPIDController.onTarget();
	}

	public boolean allControllersOnTarget(boolean isFinalWaypoint) {
		if (xPIDControllerOnTarget() && yPIDControllerOnTarget() && turnPIDControllerOnTarget()) {
			cyclesOnTarget++;
		} else {
			cyclesOnTarget = 0;
		}
		return cyclesOnTarget >= (isFinalWaypoint ? REQUIRED_CYCLES_ON_TARGET : 1);
	}

	public void driveCartesian(double currVelX, double currVelY, double currRot) {
		rawDriveCartesian(currVelX, currVelY, currRot);
	}

	public void rawDriveCartesian(double velX, double velY, double rot) {
		velY *= scale;
		rot *= scale;

		double xAccel = velX - lastVelX;
		double yAccel = velY - lastVelY;

		if (Math.abs(xAccel) > maxDriveAccel) {
			velX = lastVelX + Math.copySign(maxDriveAccel, xAccel);
		}
		if (Math.abs(yAccel) > maxDriveAccel) {
			velY = lastVelY + Math.copySign(maxDriveAccel, yAccel);
		}

		RobotMap.driveMecanumDrive.driveCartesian(velX, velY, rot);
		
		SmartDashboard.putNumber("velY", velY);
		SmartDashboard.putNumber("velX", velX);
		
		lastVelX = velX;
		lastVelY = velY;
	}

	public void setMaxAccel(double accel) {
		maxDriveAccel = accel;
	}

	public void tankDrive(double pL, double pR) {
		RobotMap.driveLeftFrontMotor.set(pL * 1);
		RobotMap.driveLeftRearMotor.set(pL * 1);

		RobotMap.driveRightFrontMotor.set(-pR * 1);
		RobotMap.driveRightRearMotor.set(-pR * 1);
	}
	
	public void tankDriveOnHeadingPIDInit(double desiredHeading, double maxDrivePower) {
		driveStraightOnHeadingPIDController = createPIDController(desiredHeading, 
				1.0, 
				DRIVE_STRAIGHT_ON_HEADING_P, 
				DRIVE_STRAIGHT_ON_HEADING_I, 
				DRIVE_STRAIGHT_ON_HEADING_D, 
				maxDrivePower);
		
	}
	
	public void tankDriveOnHeadingPIDExecute(double power) {
		double accel = power - lastPower;
		if (Math.abs(accel) > maxDriveAccel) {
			power = lastPower  + Math.copySign(maxDriveAccel, accel);
		}
		double leftPower = power;
		double rightPower = power;

		double currHeading = RobotMap.gyro.getAngle();
		double error = driveStraightOnHeadingPIDController.getSetpoint() - currHeading;
		
		// output range for turn adjustment power is based on error
		// in this case, error >= 15deg corresponds to [-1,1]
		// while 0deg <= error < 15deg corresponds to a smaller range
		double calculatedOutputRange = MathUtil.clamp(PID_MIN_OUTPUT
				+ (Math.abs(error) / 15.0) * (PID_MAX_OUTPUT - PID_MIN_OUTPUT),
				0, PID_MAX_OUTPUT);
		driveStraightOnHeadingPIDController.setOutputRange(-calculatedOutputRange, calculatedOutputRange);

		// this power is added/subtracted from desired power to reach desired heading
		double turnAdjustmentPower = driveStraightOnHeadingPIDController.update(currHeading);

		if (power > 0) {
			if (turnAdjustmentPower > 0) {
				rightPower -= turnAdjustmentPower;
			} else {
				leftPower += turnAdjustmentPower;
			}
		} else {
			if (turnAdjustmentPower > 0) {
				leftPower += turnAdjustmentPower;
			} else {
				rightPower -= turnAdjustmentPower;
			}
		}
		
		tankDrive(leftPower, rightPower);
		lastPower = power;

		SmartDashboard.putNumber("drive on heading turn adjustment power", turnAdjustmentPower);	
	}

	
	public void stop() {
		lastVelX = 0;
		lastVelY = 0;

		RobotMap.driveMecanumDrive.stopMotor();
	}

	public void setScale(double s) {
		scale = s;
	}

	@Override
	public void pidWrite(double output) {
		drivePIDOutput = output;
	}

	public double getXError() {
		return xPIDController.getError();
	}

	public double getYError() {
		return yPIDController.getError();
	}

	public double getTurnError() {
		return turnPIDController.getError();
	}
	
	public double getDesiredHeading() {
		return desiredHeading;
	}
	
	public void setDesiredHeading(double desiredHeading) {
		this.desiredHeading = desiredHeading;
	}
}
