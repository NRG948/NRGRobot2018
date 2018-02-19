package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeTilt;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * CubeTilter: subsystem for controlling cube acquirer tilt motor.
 * Positive power is for tilting up, negative power is for tilting down
 */ 
public class CubeTilter extends Subsystem {
	public static final double TILT_UP_POWER = 0.5;
	public static final double TILT_DOWN_POWER = 0.1;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
//        setDefaultCommand(new ManualCubeTilt());
    }
    
    public void rawTilt(double power) {
    	RobotMap.cubeTitlerMotor.set(power);
    }
    
    public void tiltDown() {
    	rawTilt(TILT_DOWN_POWER);
    }
    
    public void tiltUp() {
    	rawTilt(TILT_UP_POWER);
    }
    
    public void stop() {
    	RobotMap.cubeTitlerMotor.stopMotor();
    }
}

