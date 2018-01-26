package org.usfirst.frc948.NRGRobot2018.utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class ContinuousGyro implements PIDSource {
	AHRS navx;
	private double headingOffset;
	
	public ContinuousGyro(AHRS gyro) {
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
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return navx.getAngle();
	}
}
