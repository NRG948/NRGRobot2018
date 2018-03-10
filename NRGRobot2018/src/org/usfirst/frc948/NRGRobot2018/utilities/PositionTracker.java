package org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.RobotMap;

public class PositionTracker {
	private double x;
	private double y;
	private double xGoal;
	private double yGoal;
	private double prevXEncoder;
	private double prevYEncoder;


	public PositionTracker(double x, double y) {
		// parameters are in inches
		reset(x,y);
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
		double robotToFieldRadians = Math.toRadians(90 - (currHeading + Math.toDegrees(Math.atan2(xDelta, yDelta))));
		x += distanceTraveled * Math.cos(robotToFieldRadians);
		y += distanceTraveled * Math.sin(robotToFieldRadians);

		prevXEncoder = currXEncoder;
		prevYEncoder = currYEncoder;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void setXY(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double getXGoal() {
		return xGoal;
	}
	public double getYGoal() {
		return yGoal;
	}
	public void setXYGoal(double xGoal, double yGoal) {
		this.xGoal = xGoal;
		this.yGoal = yGoal;
	}

	public void reset(double x, double y) {
		// parameters are in inches
		setXY(x,y);
		setXYGoal(x,y);
		prevXEncoder = RobotMap.xEncoder.getDistance();
		prevYEncoder = RobotMap.yEncoder.getDistance();
	}
	public void reset() {
		reset(0,0);
	}
}
