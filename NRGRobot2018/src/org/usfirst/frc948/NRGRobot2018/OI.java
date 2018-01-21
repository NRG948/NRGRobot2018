// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc948.NRGRobot2018;

import org.usfirst.frc948.NRGRobot2018.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc948.NRGRobot2018.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public Joystick leftJoystick;
    public Joystick rightJoystick;
    public Joystick xboxController;
    
    public JoystickButton shiftGears;
    
    public OI() {
    	shiftGears = new JoystickButton(rightJoystick, 0);
        xboxController = new Joystick(2);
        
        rightJoystick = new Joystick(1);
        
        leftJoystick = new Joystick(0);
        
        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("ManualDrive", new ManualDrive());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    
    public void buttonInit() {
    	shiftGears.whenPressed(new SetDriveScale(Drive.SCALE_HIGH));
    	shiftGears.whenReleased(new SetDriveScale(Drive.SCALE_LOW));
    }
    public Joystick getLeftJoystick() {
        return leftJoystick;
    }

    public Joystick getRightJoystick() {
        return rightJoystick;
    }

    public Joystick getXboxController() {
        return xboxController;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

