package src.org.usfirst.frc948.NRGRobot2018.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import src.org.usfirst.frc948.NRGRobot2018.commands.ManualClimb;

/**
 *
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    	setDefaultCommand(new ManualClimb(0.5));//the power is set to 0.5 for now
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

