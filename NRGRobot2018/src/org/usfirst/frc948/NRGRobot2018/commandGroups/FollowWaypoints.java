package org.usfirst.frc948.NRGRobot2018.commandGroups;

import org.usfirst.frc948.NRGRobot2018.commands.DriveToXYHeadingPID;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class FollowWaypoints extends CommandGroup {

	public FollowWaypoints(Waypoint waypoints[]) {
		double previousX = 0.0;
		double previousY = 0.0;

		for (int i = 0; i < waypoints.length; ++i) {
			Waypoint waypoint = waypoints[i];
			boolean isAbsolute = waypoint.coordinateType == CoordinateType.ABSOLUTE;
			double x = isAbsolute ? waypoint.x : previousX + waypoint.x;
			double y = isAbsolute ? waypoint.y : previousY + waypoint.y;

			addSequential(new DriveToXYHeadingPID(x, y, waypoint.heading, waypoint.getPredicate(),
					i == (waypoints.length) - 1));

			previousX = x;
			previousY = y;
		}
	}
}
