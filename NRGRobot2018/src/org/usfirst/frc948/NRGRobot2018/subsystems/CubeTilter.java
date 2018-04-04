package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeTiltPIDAssist;
import org.usfirst.frc948.NRGRobot2018.utilities.SimplePIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * CubeTilter: subsystem for controlling cube acquirer tilt motor.
 * 
 * Positive power is for tilting up, negative power is for tilting down
 * 
 * Held upright by mechanical arm before auto period is 0deg, all the way down is 86deg
 */ 
public class CubeTilter extends Subsystem {
	public static final double TILT_UP_POWER = 1.0;
	public static final double TILT_DOWN_POWER = -0.1;

	public static final double DEFAULT_TILT_P = 1 / 100.0;
	public static final double DEFAULT_TILT_I = 0;
	public static final double DEFAULT_TILT_D = 0;
	
	public static final double TILTER_UP = 0;
	public static final double TILTER_DOWN = -256;
	
	private double desiredAngle;
	private double tiltPIDTolerance;

    public void initDefaultCommand() {
        setDefaultCommand(new ManualCubeTiltPIDAssist());
    }
    
    public void tiltToAnglePIDIntialize(double desiredAngle, double tolerance) {
    	this.desiredAngle = desiredAngle;
    	tiltPIDTolerance = tolerance;
    	
    	RobotMap.cubeTilterMotor.selectProfileSlot(0, 0);
    	RobotMap.cubeTilterMotor.config_kP(0, DEFAULT_TILT_P, 0);
    	RobotMap.cubeTilterMotor.config_kI(0, DEFAULT_TILT_I, 0);
    	RobotMap.cubeTilterMotor.config_kD(0, DEFAULT_TILT_D, 0);
    	RobotMap.cubeTilterMotor.configClosedLoopPeakOutput(0, TILT_UP_POWER, 0);
    }
    
    public void tiltToAnglePIDExecute() {
    	RobotMap.cubeTilterMotor.set(ControlMode.Position, desiredAngle);
    	SmartDashboard.putNumber("TiltPID/computedPower", RobotMap.cubeTilterMotor.getMotorOutputPercent());
    }

	public boolean tiltToAnglePIDOnTarget() {
    	return Math.abs(RobotMap.cubeTilterMotor.getClosedLoopError(0)) <= tiltPIDTolerance;
    }
    
    public void rawTilt(double power) {
    	RobotMap.cubeTilterMotor.set(ControlMode.PercentOutput, power);
    }
    
    public double getDesiredAngle() {
    	return desiredAngle;
    }
    
    public double getTiltPIDTolerance() {
    	return tiltPIDTolerance;
    }
    
    public void tiltDown() {
    	rawTilt(TILT_DOWN_POWER);
    }
    
    public void tiltUp() {
    	rawTilt(TILT_UP_POWER);
    }
    
    public void stop() {
    	RobotMap.cubeTilterMotor.stopMotor();
    }
}

