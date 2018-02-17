package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeLift;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CubeLifter extends Subsystem {
	private final double LIFT_POWER_SCALE_UP = 0.5;
	private final double LIFT_POWER_SCALE_DOWN = 0.3;
	
	public enum Direction {
		UP, DOWN;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ManualCubeLift());
    }
    
    public void lift(double power, Direction direction) {
    	power = Math.abs(power);
    	
    	if (direction == Direction.UP) {
    		rawLift(power);
    	} else {
    		rawLift(-power);
    	}
    }
    
    public void rawLift(double power) {
    	if (power > 0) {
    		RobotMap.cubeLifterMotor.set(power * LIFT_POWER_SCALE_UP);
    	} else {
    		RobotMap.cubeLifterMotor.set(power * LIFT_POWER_SCALE_DOWN);
    	}
    }
    
    public void periodic() {
    	
    }
}
