package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.vision.PixyCam.Block;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestPixyData extends Command {

    public TestPixyData() {
        // Use requires() here to declare subsystem dependencies
         requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("testpixy initialize");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("test execute");
    	RobotMap.pixy.getBlocksLoop();
		System.out.println("Dumping pixy blocks");
		
		for (Block block : RobotMap.pixy.getBlocks()) {
			System.out.println(block);
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("testpixy isfinished");
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("testpixy end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    	System.out.println("testpixy interrupted");
    }
}
