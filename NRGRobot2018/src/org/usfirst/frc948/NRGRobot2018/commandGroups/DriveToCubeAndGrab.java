package org.usfirst.frc948.NRGRobot2018.commandGroups;

import java.time.temporal.IsoFields;

import org.usfirst.frc948.NRGRobot2018.commands.AcquireUntilCubeDetected;
import org.usfirst.frc948.NRGRobot2018.commands.DriveToCubeNoPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveToCubeAndGrab extends CommandGroup {
    public DriveToCubeAndGrab() {
    	addParallel(new DriveToCubeNoPID(true));
    	addSequential(new AcquireUntilCubeDetected(0.5));
    }
}
