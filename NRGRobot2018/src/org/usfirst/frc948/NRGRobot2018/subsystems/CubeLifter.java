package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeLift;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * CubeLifter subsystem: controls cube lifter motor.
 * 
 * Positive power is for raising lifter, negative power is for lowering lifter.
 */
public class CubeLifter extends Subsystem {
	private static final double LIFT_POWER_SCALE_UP = 0.5;
	private static final double LIFT_POWER_SCALE_DOWN = 0.3;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ManualCubeLift());
	}

	public void manualLift(double power) {
		if (power > 0) {
			RobotMap.cubeLifterMotor.set(power * LIFT_POWER_SCALE_UP);
		} else {
			RobotMap.cubeLifterMotor.set(power * LIFT_POWER_SCALE_DOWN);
		}
	}

	public void rawLift(double power) {
		RobotMap.cubeLifterMotor.set(power);
	}
	
	public void stop() {
		RobotMap.cubeLifterMotor.stopMotor();
	}

	public void periodic() {

	}
}
