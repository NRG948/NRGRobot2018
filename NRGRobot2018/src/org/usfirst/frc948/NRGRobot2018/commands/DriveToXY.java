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
		double currentX = Robot.positionTracker.getX();
		double currentY = Robot.positionTracker.getY();
		
		double desiredX = Robot.preferences.getDouble(PreferenceKeys.X2, 0.0);
		double desiredY = Robot.preferences.getDouble(PreferenceKeys.Y2, 0.0);
		double desiredHeading = Math.atan2(desiredX - currentX, desiredY - currentY) * 180 / Math.PI;
		double distanceToTravel = Math
				.sqrt((desiredX - currentX) * (desiredX - currentX) + (desiredY - currentY) * (desiredY - currentY));

		SmartDashboard.putString("DriveToXY: heading, distance",
				String.format("%.2f, %.2f", desiredHeading, distanceToTravel));

		new CommandGroup() {
			{
				addSequential(new TurnToHeading(desiredHeading));
				addSequential(new DriveStraightDistance(0.5, distanceToTravel, Drive.Direction.FORWARD));
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
