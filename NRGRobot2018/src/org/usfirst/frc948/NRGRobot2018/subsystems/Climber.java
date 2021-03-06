package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualClimb;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Climber subsystem for climber arm.
 * 
 * Positive power for raising climber up and negative power for down.
 * 
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    	// Set the default command for a subsystem here.
    	setDefaultCommand(new ManualClimb(0));
    }
    public void rawClimb(double power){
    	RobotMap.climberMotor.set(power);
    }
    
    public void stop(){
    	RobotMap.climberMotor.stopMotor();
    }
}

