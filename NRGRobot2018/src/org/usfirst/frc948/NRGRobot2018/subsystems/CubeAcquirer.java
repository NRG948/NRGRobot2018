package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeAcquire;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * CubeAcquirer subsystem: controls wheels to acquire/eject power-cube.
 * 
 * Positive power is for ejecting, negative power is for acquiring.
 */

public class CubeAcquirer extends Subsystem {
	public enum Direction {
		ACQUIRE, EJECT;
	}
	
	public boolean lastCubeSwitchState;
	private double acquireScale = 1.0;

	public CubeAcquirer() {
		lastCubeSwitchState = isCubeIn();
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new ManualCubeAcquire());
	}

	public void acquire(double power, Direction direction) {
		power = Math.abs(power);
		
		if (direction == Direction.ACQUIRE) {
			rawAcquire(-power, -power);
		} else {
			rawAcquire(power, power);
		}
	}
	
	public void rawAcquire(double leftPower, double rightPower) {
		RobotMap.acquirerLeftMotor.set(leftPower * acquireScale);
		RobotMap.acquirerRightMotor.set(rightPower * acquireScale);
	}
	
	public boolean isCubeIn(){
		return !RobotMap.cubeDetectSwitch.get();
	}
	
	public double getAcquireScale() {
		return acquireScale;
	}
	
	public void setAcquireScale(double acquireMaxPower) {
		this.acquireScale = acquireMaxPower;
	}
	
	public void stop() {
		RobotMap.acquirerLeftMotor.stopMotor();
		RobotMap.acquirerRightMotor.stopMotor();
	}
}
