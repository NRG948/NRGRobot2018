package src.org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.RobotMap;

public class PositionTracker {
	private static double x, y;
	private static double prevXEncoder;
	private static double prevYEncoder;

	public static void initialize(double x, double y) {
		// already converted to inches
		this.x = x;
		this.y = y;

		prevXEncoder = RobotMap.xEncoder.getDistance();
		prevYEncoder = RobotMap.yEncoder.getDistance();
	}

	public static void updatePosition() {
		double currXEncoder = RobotMap.xEncoder.getDistance();
		double currYEncoder = RobotMap.yEncoder.getDistance();
		double currHeading = RobotMap.gyro.getAngle();

		// in robot reference frame
		double xDelta = currXEncoder - prevXEncoder;
		double yDelta = currYEncoder - prevYEncoder;
		double distanceTraveled = Math.sqrt(xDelta * xDelta + yDelta * yDelta);

		// converting to field reference frame
		double robotToFieldHeading = currHeading + Math.toDegrees(Math.atan(xDelta / yDelta));
		x += distanceTraveled * Math.cos(robotToFieldAngle);
		y += distanceTraveled * Math.sin(robotToFieldAngle);

		prevXEncoder = currXEncoder;
		prevYEncoder = currYEncoder;
	}

	public static double getX() {
		return x;
	}

	public static double getY() {
		return y;
	}

	public static void setX(double x) {
		PositionTracker.x = x;
	}

	public static void setY(double y) {
		PositionTracker.y = y;
	}
}
