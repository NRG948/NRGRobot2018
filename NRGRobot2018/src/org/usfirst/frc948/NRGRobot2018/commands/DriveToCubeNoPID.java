package org.usfirst.frc948.NRGRobot2018.commands;

import java.util.ArrayList;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.utilities.CubeCalculations;
import org.usfirst.frc948.NRGRobot2018.utilities.MathUtil;
import org.usfirst.frc948.NRGRobot2018.vision.PixyCam.Block;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToCubeNoPID extends Command {
	ArrayList<Block> currFrame;
	double distanceToCube; // in inches
	final double DISTANCE_TO_SLOW = 25;
	
    public DriveToCubeNoPID() {
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	distanceToCube = Double.MAX_VALUE;
    	currFrame = null;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	currFrame = RobotMap.pixy.getPixyFrameData();
    	Block currBlock;
    	
    	if (currFrame.size() > 0) {
    		currBlock = currFrame.get(0);
    		distanceToCube = CubeCalculations.getDistanceFromWidth(currBlock);
    		
    		double cubeNormalized = CubeCalculations.getDistanceToCenterNormalized(currBlock);
    		double turnPower = Math.copySign(MathUtil.clamp(Math.abs(cubeNormalized), 0.15, 1), cubeNormalized);
    		double drivePower = Math.min(1.0, distanceToCube / DISTANCE_TO_SLOW);

			Robot.drive.rawDriveCartesian(0, drivePower, turnPower);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return distanceToCube <= 21;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
