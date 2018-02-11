package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeAcquire;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CubeAcquirer extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ManualCubeAcquire());
	}

	public void periodic() {
//		SmartDashboard.putData("acquire Servo", RobotMap.acquireServo);
	}
}
