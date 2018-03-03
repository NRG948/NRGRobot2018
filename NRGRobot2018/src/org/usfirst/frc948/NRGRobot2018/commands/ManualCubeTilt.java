package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeTilter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualCubeTilt extends Command {
//	private boolean wasDPadPressed; // whether d-pad pressed in the previous cycle
//	private boolean notCommandedAtAll; // whether or not d-pad was pressed AT ALL since the last initialize() call
	
	public ManualCubeTilt() {
		requires(Robot.cubeTilter);
	}

	protected void initialize() {
//		wasDPadPressed = false;
//		notCommandedAtAll = true;
	}

	protected void execute() {
		if (OI.isXBoxDPadUp()) {
//			wasDPadPressed = true;
//			notCommandedAtAll = false;
			
			Robot.cubeTilter.tiltUp();
		} else if (OI.isXBoxDPadDown()) {
//			wasDPadPressed = true;
//			notCommandedAtAll = false;

			Robot.cubeTilter.tiltDown();
		} else {
//			if (DriverStation.getInstance().isAutonomous() || notCommandedAtAll) {
			Robot.cubeTilter.stop();
//			} else {
//				if (wasDPadPressed) {
//					wasDPadPressed = false;
//					Robot.cubeTilter.tiltToAnglePIDIntialize(RobotMap.cubeTiltEncoder.getDistance(), 1.0);
//				}
//				Robot.cubeTilter.tiltToAnglePIDExecute();
//			}
		}
	}
	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		Robot.cubeTilter.stop();
	}

	protected void interrupted() {
		end();
	}
}
