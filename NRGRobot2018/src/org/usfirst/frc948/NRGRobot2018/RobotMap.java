// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc948.NRGRobot2018;

import org.usfirst.frc948.NRGRobot2018.utilities.ContinuousGyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	public static Victor driveLeftFrontMotor;
	public static Victor driveLeftRearMotor;
	public static Victor driveRightFrontMotor;
	public static Victor driveRightRearMotor;
	public static MecanumDrive driveMecanumDrive;
	
	public static AHRS navx;
	public static ContinuousGyro gyro;

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	public static void init() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
		driveLeftFrontMotor = new Victor(1);
		driveLeftRearMotor = new Victor(3);
		driveRightFrontMotor = new Victor(0);
		driveRightRearMotor = new Victor(2);
		
		driveMecanumDrive = new MecanumDrive(driveLeftFrontMotor, driveLeftRearMotor, driveRightFrontMotor,
				driveRightRearMotor);
		driveMecanumDrive.setSafetyEnabled(false);
		driveMecanumDrive.setExpiration(0.1);
		driveMecanumDrive.setMaxOutput(1.0);

		navx = new AHRS(SPI.Port.kMXP);
		gyro = new ContinuousGyro(navx);
		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	}
}
