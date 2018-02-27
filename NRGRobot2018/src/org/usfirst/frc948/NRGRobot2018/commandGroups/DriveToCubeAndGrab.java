package org.usfirst.frc948.NRGRobot2018.commandGroups;

import org.usfirst.frc948.NRGRobot2018.commands.AcquireUntilCubeDetected;
import org.usfirst.frc948.NRGRobot2018.commands.DriveToCubeNoPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveToCubeAndGrab extends CommandGroup {

    public DriveToCubeAndGrab() {
    	addSequential(new DriveToCubeNoPID());
    	addSequential(new AcquireUntilCubeDetected(0.5));
    }
}
