package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeTiltPIDAssist;
import org.usfirst.frc948.NRGRobot2018.utilities.SimplePIDController;

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
	public static final double TILT_UP_POWER = 0.5;
	public static final double TILT_DOWN_POWER = -0.1;

	public static final double DEFAULT_TILT_P = 1 / 100.0;
	public static final double DEFAULT_TILT_I = 0;
	public static final double DEFAULT_TILT_D = 0;
	
	public static final double TILTER_UP = 0;
	public static final double TILTER_DOWN = -1200;
	
	public static final double TILTER_RANGE_MIN = -2300;
	public static final double TILTER_RANGE_MAX = 0;
	public static final double TILTER_PID_CUTOFF = -2000;
	
	private static SimplePIDController tiltPIDController;

    public void initDefaultCommand() {
        setDefaultCommand(new ManualCubeTiltPIDAssist());
    }
    
    private void createTiltPIDController(double setpoint, double tolerance){
    	tiltPIDController = new SimplePIDController(DEFAULT_TILT_P, DEFAULT_TILT_I, DEFAULT_TILT_D, true)
    			.setAbsoluteTolerance(tolerance)
    			.setInputRange(TILTER_RANGE_MIN, TILTER_RANGE_MAX)
    			.setOutputRange(TILT_DOWN_POWER, TILT_UP_POWER)
    			.setSetpoint(setpoint)
    			.start();
    }
    
    public void tiltToAnglePIDIntialize(double setpoint, double tolerance){
    	createTiltPIDController(setpoint, tolerance);
    }
    
    public void tiltToAnglePIDExecute(){
    	double computedPower = tiltPIDController.update(RobotMap.cubeTiltEncoder.getDistance());
    	SmartDashboard.putNumber("computedPower", computedPower);
    	rawTilt(computedPower);
    }
    
    public boolean tiltToAnglePIDOnTarget(){
    	return tiltPIDController.onTarget();
    }
    
    public void rawTilt(double power) {
    	RobotMap.cubeTilterMotor.set(power);
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

