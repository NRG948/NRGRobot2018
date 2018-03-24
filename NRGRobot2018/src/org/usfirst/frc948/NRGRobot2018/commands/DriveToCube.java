package org.usfirst.frc948.NRGRobot2018.commands;

import java.util.ArrayList;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.utilities.CubeCalculations;
import org.usfirst.frc948.NRGRobot2018.utilities.MathUtil;
import org.usfirst.frc948.NRGRobot2018.vision.PixyCam.Block;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveToCube extends Command {
	private final double DISTANCE_TO_SLOW = 50;
	private final int DISTANCE_TO_STOP = 31;

	ArrayList<Block> currFrame;
	double distanceToCube; // in inches
	boolean driveUntilCubeAcquired;

	public DriveToCube(boolean driveUntilCubeAcquired) {
		requires(Robot.drive);
		this.driveUntilCubeAcquired = driveUntilCubeAcquired;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		distanceToCube = Double.MAX_VALUE;
		currFrame = null;
		System.out.println("DriveToCube init");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		currFrame = RobotMap.pixy.getPixyFrameData();
		Block currBlock;

		SmartDashboard.putNumber("DriveToCube/current frame size", currFrame.size());
		if (currFrame.size() > 0) {
			currBlock = currFrame.get(0);
			System.out.println("Detected Block " + currBlock);
			distanceToCube = CubeCalculations.getDistanceFromWidth(currBlock);

			double cubeNormalized = CubeCalculations.getDistanceToCenterNormalized(currBlock);
			double turnPower = MathUtil.clampNegativePositive(cubeNormalized, 0.15, 1.0);

			double drivePower;
			
			if (driveUntilCubeAcquired) {
				drivePower = MathUtil.clamp(Math.abs(distanceToCube / DISTANCE_TO_SLOW), 0.12, 0.35);
			} else {
				drivePower = Math.min(1.0, (distanceToCube - DISTANCE_TO_STOP) / (DISTANCE_TO_SLOW - DISTANCE_TO_STOP));
			}

			Robot.drive.rawDriveCartesian(0, drivePower, turnPower);
			SmartDashboard.putNumber("DriveToCube/distance", distanceToCube);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return driveUntilCubeAcquired ? Robot.cubeAcquirer.isCubeIn() : distanceToCube <= DISTANCE_TO_STOP;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("DriveToCube end");
		Robot.drive.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
