package org.usfirst.frc948.NRGRobot2018.commandGroups;

import java.util.ArrayList;

import org.usfirst.frc948.NRGRobot2018.commands.DriveToXYHeadingPID;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class FollowWaypoints extends CommandGroup {

    public FollowWaypoints(ArrayList<Waypoint> waypoints) {
        double previousX = 0.0;
        double previousY = 0.0;

        for(int i = 0; i<waypoints.size(); ++i) {
           Waypoint waypoint = waypoints.get(i);
           boolean isAbsolute = waypoint.coordinateType == CoordinateType.ABSOLUTE;
           double x = isAbsolute? waypoint.x : previousX + waypoint.x;
           double y = isAbsolute? waypoint.y : previousY + waypoint.y;
           
           addSequential(new DriveToXYHeadingPID(x, y, waypoint.heading, waypoint.getPredicate(), i == (waypoints.size()-1)));
           
           previousX = x;
           previousY = y;
       }
    }
}
