package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeAcquire;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeAcquirer extends Subsystem {

	public enum Direction {
		ACQUIRE, EJECT;
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new ManualCubeAcquire());
	}

	public void acquire(double power, Direction direction) {
		//TODO: directions need to be tested
		power = Math.abs(power);
		
		if (direction == Direction.ACQUIRE) {
			rawAcquire(power, -power);
		} else {
			rawAcquire(-power, power);
		}
	}
	
	public void rawAcquire(double leftPower, double rightPower) {
		RobotMap.acquirerLeftMotor.set(leftPower);
		RobotMap.acquirerRightMotor.set(rightPower);
	}

	public void periodic() {
//		SmartDashboard.putData("acquire Servo", RobotMap.acquireServo);
	}
}
