package org.usfirst.frc948.NRGRobot2018.commandGroups;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.commands.DriveToXYHeadingPID;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class FollowWaypoints extends CommandGroup {

	public FollowWaypoints(double startX, double startY, Waypoint waypoints[]) {
		double previousX = startX;
		double previousY = startY;

		for (int i = 0; i < waypoints.length; ++i) {
			// Making sure all waypoints are absolute coordinates
			Waypoint waypoint = waypoints[i].toAbsolute(previousX, previousY);
			
			// Robot will stop if it's the last waypoint in the path
			addSequential(new DriveToXYHeadingPID(waypoint, i == (waypoints.length) - 1));
			
			previousX = waypoint.x;
			previousY = waypoint.y;
		}
	}
}
