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
import org.usfirst.frc948.NRGRobot2018.utilities.PreferenceKeys;
import org.usfirst.frc948.NRGRobot2018.utilities.SimplePIDController;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Drive extends Subsystem implements PIDOutput {
	
	public enum Direction {
		FORWARD, BACKWARD, LEFT, RIGHT
	}

	//	private PIDController drivePIDController;
	private SimplePIDController drivePIDController;
	private volatile double PIDOutput = 0;
	
	public final static double DEFAULT_TURN_P = 0.02;
	public final static double DEFAULT_TURN_I = 0.0;
	public final static double DEFAULT_TURN_D = 0.0;
	
	public final static double SCALE_HIGH = 1.0;
	public final static double SCALE_LOW = 0.5;
	public double scale = SCALE_LOW;
	public static final double DEF_MAX_VEL_CHANGE = 0.1;
	
	private double lastVelX = 0.0;
	private double lastVelY = 0.0;

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}
	
	public void drivePIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
		drivePIDController = new SimplePIDController(p, i, d, false, RobotMap.gyro, this);
		
		drivePIDController.setOutputRange(-1, 1);
		drivePIDController.setInputRange(-1, 1);
		drivePIDController.setSetpoint(setpoint);
		drivePIDController.setAbsoluteTolerance(tolerance);
		
		drivePIDController.start();
	}
	
	public void driveHeadingPIDInit(double desiredHeading, double tolerance) {
		drivePIDControllerInit(Robot.preferences.getDouble(PreferenceKeys.TURN_P_TERM, DEFAULT_TURN_P),
				Robot.preferences.getDouble(PreferenceKeys.TURN_I_TERM, DEFAULT_TURN_I),
				Robot.preferences.getDouble(PreferenceKeys.TURN_D_TERM, DEFAULT_TURN_D),
				desiredHeading,
				tolerance);
	}
	
	public void driveHeadingPIDExecute(double velX, double velY) {
		drivePIDController.update();
		double currentPIDOutput = PIDOutput;

		SmartDashboard.putNumber("Turn To Heading PID Error", drivePIDController.getError());
		SmartDashboard.putNumber("Turn To Heading PID Output", currentPIDOutput);
		
		rawDriveCartesian(velX, velY, currentPIDOutput);
	}
	
	public void driveHeadingPIDEnd() {
		drivePIDController = null;
		stop();
	}
	
	public void driveCartesian(double currVelX, double currVelY, double currRot) {
		double maxVelDifference = Robot.preferences.getDouble(PreferenceKeys.MAX_VEL_CHANGE, DEF_MAX_VEL_CHANGE);
		double velXChange = currVelX - lastVelX;
		double velYChange = currVelY - lastVelY;		
		
		if (Math.abs(velXChange) > maxVelDifference) {
			currVelX = lastVelX + Math.copySign(maxVelDifference, velXChange);
		}
		
		if (Math.abs(velYChange) > maxVelDifference) {
			currVelY = lastVelY + Math.copySign(maxVelDifference, velYChange);
		}
		
		rawDriveCartesian(currVelX, currVelY, currRot);
	}
	
	public void rawDriveCartesian(double velX, double velY, double rot) {
		lastVelX = velX;
		lastVelY = velY;
		
		velX *= scale;
		velY *= scale;
		rot *= scale;
		RobotMap.driveMecanumDrive.driveCartesian(velX, velY, rot);
		SmartDashboard.putNumber("velY", velY);
		SmartDashboard.putNumber("velX", velX);
	}

	public void stop() {
		lastVelX = 0;
		lastVelY = 0;
		
		RobotMap.driveMecanumDrive.stopMotor();
	}
	
	public void setScale(double s) {
		scale = s;
	}

	public boolean onTarget() {
		return drivePIDController.onTarget();
	}
	
	@Override
	public void periodic() {

	}

	@Override
	public void pidWrite(double output) {
		PIDOutput = output;
	}
}
