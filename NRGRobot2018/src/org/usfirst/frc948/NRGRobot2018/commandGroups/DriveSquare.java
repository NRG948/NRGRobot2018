package src.org.usfirst.frc948.NRGRobot2018.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import src.org.usfirst.frc948.NRGRobot2018.commands.DelaySeconds;
import src.org.usfirst.frc948.NRGRobot2018.commands.DriveStraightTimed;
import src.org.usfirst.frc948.NRGRobot2018.commands.StrafeStraightTimed;

/**
 *
 */
public class DriveSquare extends CommandGroup {

    public DriveSquare() {
    	addSequential(new DriveStraightTimed(0.5, 2));
    	addSequential(new DelaySeconds(1));
    	addSequential(new StrafeStraightTimed(0.7, 3));
    	addSequential(new DelaySeconds(1));
    	addSequential(new DriveStraightTimed(-0.5, 2));
    	addSequential(new DelaySeconds(1));
    	addSequential(new StrafeStraightTimed(-0.7, 3));
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
