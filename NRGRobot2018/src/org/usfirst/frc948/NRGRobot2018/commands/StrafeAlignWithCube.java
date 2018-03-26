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
public class StrafeAlignWithCube extends Command {
	private final double ALIGN_ERROR_TOLERANCE = 0.05;

	private double alignError;

	public StrafeAlignWithCube() {
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		alignError = Double.MAX_VALUE;
		
		Robot.drive.turnPIDControllerInit(RobotMap.gyro.getAngle(), 1.0);
		System.out.println("CenterToCube init");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		ArrayList<Block> currentFrame = RobotMap.pixy.getPixyFrameData();
		SmartDashboard.putNumber("CenterToCube/number of objects", currentFrame.size());

		if (currentFrame.size() > 0) {
			Block currentBlock = currentFrame.get(0);
			
			alignError = CubeCalculations.getDistanceToCenterNormalized(currentBlock);
			
			// minimum strafe power is .15 to prevent stalling out
			double strafePower = MathUtil.clampNegativePositive(alignError, 0.5, 1.0);
			double calculatedTurnPower = Robot.drive.turnPIDControllerExecute(RobotMap.gyro.getAngle());
			
			Robot.drive.rawDriveCartesian(strafePower, 0, calculatedTurnPower);
			SmartDashboard.putNumber("CenterToCube/strafe power", strafePower);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(alignError) <= ALIGN_ERROR_TOLERANCE;
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
