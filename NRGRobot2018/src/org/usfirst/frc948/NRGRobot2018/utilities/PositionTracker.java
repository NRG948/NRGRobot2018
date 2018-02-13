package org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.RobotMap;

public class PositionTracker {
	private double x;
	private double y;
	private double prevXEncoder;
	private double prevYEncoder;

	public PositionTracker(double x, double y) {
		// already converted to inches
		this.x = x;
		this.y = y;

		prevXEncoder = RobotMap.xEncoder.getDistance();
		prevYEncoder = RobotMap.yEncoder.getDistance();
	}

	public void updatePosition() {
		double currXEncoder = RobotMap.xEncoder.getDistance();
		double currYEncoder = RobotMap.yEncoder.getDistance();
		double currHeading = RobotMap.gyro.getAngle();

		// in robot reference frame
		double xDelta = currXEncoder - prevXEncoder;
		double yDelta = currYEncoder - prevYEncoder;
		double distanceTraveled = Math.sqrt(xDelta * xDelta + yDelta * yDelta);

		// converting to field reference frame
		double robotToFieldHeading = currHeading + Math.toDegrees(Math.atan(xDelta / yDelta));
		x += distanceTraveled * Math.cos(robotToFieldHeading);
		y += distanceTraveled * Math.sin(robotToFieldHeading);

		prevXEncoder = currXEncoder;
		prevYEncoder = currYEncoder;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}
}
