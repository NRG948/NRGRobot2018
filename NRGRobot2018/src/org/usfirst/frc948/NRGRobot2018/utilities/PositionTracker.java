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
	
	private double prevLF;
	private double prevLR;
	private double prevRF;
	private double prevRR;
	
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
		updatePositionFourEncoders();
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
		double dxRobot = currXEncoder - prevXEncoder;
		double dyRobot = currYEncoder - prevYEncoder;
		double distanceTraveled = Math.sqrt(dxRobot * dxRobot + dyRobot * dyRobot);

		// converting to field reference frame
		double robotToFieldRadians = robotToFieldRadians(currHeading, dxRobot, dyRobot);
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
		double currHeading = RobotMap.gyro.getAngle();
		
		// in robot reference frame
		double dxRobot = ((currLeftFrontEncoder + currRightRearEncoder) - (currRightFrontEncoder + currLeftRearEncoder)) / 4.0;
		double dyRobot = (currLeftFrontEncoder + currLeftRearEncoder + currRightFrontEncoder + currRightRearEncoder) / 4.0;
		double distanceTraveled = Math.sqrt(dxRobot * dxRobot + dyRobot * dyRobot);
		
		// converting to field reference frame
		double robotToFieldRadians = robotToFieldRadians(currHeading, dxRobot, dyRobot);
		mechX += distanceTraveled * Math.cos(robotToFieldRadians);
		mechY += distanceTraveled * Math.sin(robotToFieldRadians);
		
		prevLF = currLeftFrontEncoder;
		prevLR = currLeftRearEncoder;
		prevRF = currRightFrontEncoder;
		prevRR = currRightRearEncoder;
	}

	private double robotToFieldRadians(double currHeading, double dxRobot, double dyRobot) {
		return Math.toRadians(90 - (currHeading + Math.toDegrees(Math.atan2(dxRobot, dyRobot))));
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
