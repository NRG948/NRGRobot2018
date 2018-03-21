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
public class CenterToCube extends Command {
	//Adapting most of DriveToCubeNoPID into this
	ArrayList<Block> currentFrame;
	boolean centeredToCube;
	
    public CenterToCube() {
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	currentFrame = null;
    	System.out.println("CenterToCube init");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	currentFrame = RobotMap.pixy.getPixyFrameData();
    	Block currentBlock;
    	SmartDashboard.putNumber("CenterToCube/current frame size", currentFrame.size());
    	if(currentFrame.size() > 0) {
    		currentBlock = currentFrame.get(0);
    		//determines where the cube's center is.
    		double cubeCentered = CubeCalculations.getDistanceToCenterNormalized(currentBlock);
			//determines the turning power of the robot in order to turn enough so that it is centered.
    		double turnPower = Math.copySign(MathUtil.clamp(Math.abs(cubeCentered), 0.15, 1), cubeCentered);
    		Robot.drive.rawDriveCartesian(0, 0, turnPower);
			SmartDashboard.putNumber("Center To Cube/turning power", turnPower);
			centeredToCube = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return centeredToCube;
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
