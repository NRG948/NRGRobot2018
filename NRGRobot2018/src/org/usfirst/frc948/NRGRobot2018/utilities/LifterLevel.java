package org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter;

public class LifterLevel {
	public final String preferenceKey;
	public final int defaultTicks;
	
	public LifterLevel(String prefKey, int defaultTicks) {
		this.preferenceKey = prefKey;
		this.defaultTicks = defaultTicks;
	}
	
	public int getTicks() {
		return Robot.preferences.getInt(preferenceKey, defaultTicks);
	}
	public boolean needsPID() {
		
		return defaultTicks != CubeLifter.DEFAULT_STOWED_TICKS;
	}
}
