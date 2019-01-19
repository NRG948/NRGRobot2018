package org.usfirst.frc948.NRGRobot2018.commandGroups;

import java.time.temporal.IsoFields;

import org.usfirst.frc948.NRGRobot2018.commands.AcquireUntilCubeDetected;
import org.usfirst.frc948.NRGRobot2018.commands.DriveToCube;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveToCubeAndGrab extends CommandGroup {
    public DriveToCubeAndGrab() {
    	System.out.println("DriveToCubeAndGrab start");
    	addParallel(new DriveToCube(true));
    	addSequential(new AcquireUntilCubeDetected(0.75));
    	System.out.println("DriveToCubeAndGrab end");
    }
}
