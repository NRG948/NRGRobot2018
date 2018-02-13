package src.org.usfirst.frc948.NRGRobot2018.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import src.org.usfirst.frc948.NRGRobot2018.RobotMap;
import src.org.usfirst.frc948.NRGRobot2018.commands.ManualCubeLift;

/**
 *
 */
public class CubeLifter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ManualCubeLift());
    }
    
    public void periodic() {
    	SmartDashboard.putData("LiftVictor", RobotMap.cubeLifter);
    }
}

