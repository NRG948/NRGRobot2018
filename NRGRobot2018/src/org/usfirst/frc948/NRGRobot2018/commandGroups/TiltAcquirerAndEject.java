package org.usfirst.frc948.NRGRobot2018.commandGroups;

import org.usfirst.frc948.NRGRobot2018.commands.EjectUntilCubeOut;
import org.usfirst.frc948.NRGRobot2018.commands.TiltAcquirerToAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TiltAcquirerAndEject extends CommandGroup {
    public TiltAcquirerAndEject(double tilterAngle, double acquirerPower, double acquirerDelay) {
    	addSequential(new TiltAcquirerToAngle(tilterAngle));
    	addSequential(new EjectUntilCubeOut(acquirerPower, acquirerDelay));
    }
}
