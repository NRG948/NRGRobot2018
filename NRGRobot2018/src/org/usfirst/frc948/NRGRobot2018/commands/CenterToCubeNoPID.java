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

// Adapting most of DriveToCubeNoPID into this
public class CenterToCubeNoPID extends Command {
	private final double ERROR_TOLERANCE = 0.05;

	private double error;

	public CenterToCubeNoPID() {
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		error = Double.MAX_VALUE;
		System.out.println("CenterToCube init");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		ArrayList<Block> currentFrame = RobotMap.pixy.getPixyFrameData();
		SmartDashboard.putNumber("CenterToCube/number of objects", currentFrame.size());

		if (currentFrame.size() > 0) {
			Block currentBlock = currentFrame.get(0);
			
			double error = CubeCalculations.getDistanceToCenterNormalized(currentBlock);
			// minimum turn power is .15 to prevent stalling out
			double strafePower = Math.copySign(MathUtil.clamp(Math.abs(error), 0.15, 1), error);
			
			Robot.drive.rawDriveCartesian(0, strafePower, 0);
			SmartDashboard.putNumber("Center To Cube/strafe power", strafePower);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(error) <= ERROR_TOLERANCE;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("CenterToCube end");
		Robot.drive.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
