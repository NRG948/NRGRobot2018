package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.utilities.PreferenceKeys;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveToXY extends Command {

    public DriveToXY() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

		double x2 = Robot.preferences.getDouble(PreferenceKeys.X2, 0.0);
		double y2 = Robot.preferences.getDouble(PreferenceKeys.Y2, 0.0);
		double x1 = Robot.positionTracker.getX();
		double y1 = Robot.positionTracker.getY();
		double theta = Math.atan2(x2 - x1, y2 - y1) * 180 / Math.PI;
		double distance = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
		SmartDashboard.putString("DriveToXY: heading, distance", String.format("%.2f, %.2f", theta, distance));
		new CommandGroup() {
			{
				addSequential(new TurnToHeading(theta));
				addSequential(new DriveStraightDistance(0.5, distance, Drive.Direction.FORWARD));
			}
		}.start();
	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
