package org.usfirst.frc948.NRGRobot2018.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightDistance extends Command {
	
    // According to RobotDrive.mecanumDrive_Cartesian in WPILib:
    //
    // LF =  x + y + rot    RF = -x + y - rot
    // LR = -x + y + rot    RR =  x + y - rot
    //
    // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
    // => (LF + RR) - (RF + LR) = 4x
    // => x = ((LF + RR) - (RF + LR))/4
    //
    // LF + RF + LR + RR = 4y
    // => y = (LF + RF + LR + RR)/4
    //
    // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
    // => (LF + LR) - (RF + RR) = 4rot
    // => rot = ((LF + LR) - (RF + RR))/4
    
//	private double power;
//	protected double distance;
//	private Direction direction;
//	
//	public DriveStraightDistance (double power, double distance, Drive.Direction direction) {
//		this.direction = direction;
//		this.distance = Math.abs(distance);
//		if (direction == Drive.Direction.FORWARD || direction == Drive.Direction,BACKWARD) {
//			this.power = (direction == Drive.Direction.FORWARD) ? Math.abs(power) : -Math.abs(power);
//		}
//		else {
//			
//		}
//	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		
	}

}
