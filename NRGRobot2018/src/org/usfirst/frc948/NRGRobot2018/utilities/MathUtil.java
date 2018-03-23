package org.usfirst.frc948.NRGRobot2018.utilities;

import edu.wpi.first.wpilibj.Victor;

public class MathUtil {
	public static double deadband(double input, double range){
		return Math.abs(input) < range ? 0 : input;
	}
	
	public static double clamp(double input, double min, double max){
		return input > max ? max : input < min ? min : input;
	}
	
	public static double clampNegativePositive(double input, double min, double max) {
		min = Math.abs(min);
		max = Math.abs(max);
		
		return Math.copySign(MathUtil.clamp(Math.abs(input), min, max), input);
	}
}
