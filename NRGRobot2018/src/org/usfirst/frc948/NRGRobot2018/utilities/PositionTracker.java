package org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.RobotMap;

import com.sun.glass.ui.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PositionTracker {
	private double omniX;
	private double omniY;
	private double xGoal;
	private double yGoal;
	private double prevXEncoder;
	private double prevYEncoder; 
	
	private double mechX;
	private double mechY;
	/*private double prevLeftFrontEncoder;
	private double prevLeftRearEncoder;
	private double prevRightFrontEncoder;
	private double prevRightRearEncoder;*/


	public PositionTracker(double x, double y) {
		// parameters are in inches
		reset(x,y);
	}

	public void updatePosition() {
//		updatePositionFourEncoders();
		updatePositionTwoEncoders();
		SmartDashboard.putNumber("Position Tracker/mechX", mechX);
		SmartDashboard.putNumber("Position Tracker/mechY", mechY);
		SmartDashboard.putNumber("Position Tracker/omniX", omniX);
		SmartDashboard.putNumber("Position Tracker/omniY", omniY);
	}
	
	public void updatePositionTwoEncoders() {
		double currXEncoder = RobotMap.xEncoder.getDistance();
		double currYEncoder = RobotMap.yEncoder.getDistance();
		double currHeading = RobotMap.gyro.getAngle();

		// in robot reference frame
		double xDelta = currXEncoder - prevXEncoder;
		double yDelta = currYEncoder - prevYEncoder;
		double distanceTraveled = Math.sqrt(xDelta * xDelta + yDelta * yDelta);

		// converting to field reference frame
		double robotToFieldRadians = Math.toRadians(90 - (currHeading + Math.toDegrees(Math.atan2(xDelta, yDelta))));
		omniX += distanceTraveled * Math.cos(robotToFieldRadians);
		omniY += distanceTraveled * Math.sin(robotToFieldRadians);

		prevXEncoder = currXEncoder;
		prevYEncoder = currYEncoder;
	}
	
	public void updatePositionFourEncoders() {
		double currLeftFrontEncoder = RobotMap.leftFrontEncoder.getDistance();
		double currLeftRearEncoder = RobotMap.leftRearEncoder.getDistance();
		double currRightFrontEncoder = RobotMap.rightFrontEncoder.getDistance();
		double currRightRearEncoder = RobotMap.rightRearEncoder.getDistance();
		
		double xPos = ((currLeftFrontEncoder + currRightRearEncoder) - (currRightFrontEncoder + currLeftRearEncoder)) / 4.0;
		double yPos = (currLeftFrontEncoder + currLeftRearEncoder + currRightFrontEncoder + currRightRearEncoder) / 4.0;
		
		this.omniX = xPos;
		this.omniY = yPos;
	}
	
	public double getX() {
		boolean usingFourEncoders = org.usfirst.frc948.NRGRobot2018.Robot.preferences.getBoolean(PreferenceKeys.USE_FOUR_ENCODERS, false);
		if(usingFourEncoders) {
			return mechX;
		}
		return omniX;
	}

	public double getY() {
		boolean usingFourEncoders = org.usfirst.frc948.NRGRobot2018.Robot.preferences.getBoolean(PreferenceKeys.USE_FOUR_ENCODERS, false);
		if(usingFourEncoders) {
			return mechY;
		}
		return omniY;
	}

	public void setXY(double x, double y) {
		this.mechX = x;
		this.mechY = y;

		this.omniX = x;
		this.omniY = y;
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

	@Override
	public String toString() {
		return String.format("Position: (%.1f, %.1f)", omniX, omniY);
	}
}
