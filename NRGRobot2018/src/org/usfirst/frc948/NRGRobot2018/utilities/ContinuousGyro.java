package org.usfirst.frc948.NRGRobot2018.utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class ContinuousGyro implements PIDSource {
	private AHRS navx;
	private double headingOffset = 0.0;
	
	public ContinuousGyro(AHRS navx) {
		this.navx = navx;
	}
	
	public double getAngle() {
		return navx.getAngle() + headingOffset;
	}
	
	public double getHeadingOffset() {
		return headingOffset;
	}
	
	public void setHeadingOffset(double headingOffset) {
		this.headingOffset = headingOffset;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return navx.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		return navx.getAngle();
	}
}
