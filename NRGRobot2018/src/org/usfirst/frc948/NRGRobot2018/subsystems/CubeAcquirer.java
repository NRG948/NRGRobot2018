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
		RobotMap.acquirerLeftMotor.set(leftPower);
		RobotMap.acquirerRightMotor.set(rightPower);
	}
	
	public boolean isCubeIn(){
		return !RobotMap.cubeDetectSwitch.get();
	}
	
	public void stop() {
		RobotMap.acquirerLeftMotor.stopMotor();
		RobotMap.acquirerRightMotor.stopMotor();
	}
}
