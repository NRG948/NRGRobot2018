package org.usfirst.frc948.NRGRobot2018.subsystems;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeLift;
import org.usfirst.frc948.NRGRobot2018.utilities.PreferenceKeys;
import org.usfirst.frc948.NRGRobot2018.utilities.SimplePIDController;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * CubeLifter subsystem: controls cube lifter motor.
 * 
 * Positive power is for raising lifter, negative power is for lowering lifter.
 */
public class CubeLifter extends Subsystem {
	private SimplePIDController lifterPIDController;

	public static final double LIFT_POWER_SCALE_UP = 0.75;
	public static final double LIFT_POWER_SCALE_DOWN = 0.35;

	public final static double DEFAULT_LIFT_P = 0.46;
	public final static double DEFAULT_LIFT_I = 0.0;
	public final static double DEFAULT_LIFT_D = 0.0;

	public enum LifterLevel {
		SWITCH(20.0),
		
		SCALE_LOW(56.5),
		SCALE_MEDIUM(64.5),
		SCALE_HIGH(76.5);
		
		double height;

		private LifterLevel(double height) {
			this.height = height;
		}
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ManualCubeLift());
	}
	
	public void lifterPIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
		double maxPowerUp = Robot.preferences.getDouble(PreferenceKeys.LIFT_UP_MAX_POWER, LIFT_POWER_SCALE_UP);
		double maxPowerDown = Robot.preferences.getDouble(PreferenceKeys.LIFT_DOWN_MAX_POWER, LIFT_POWER_SCALE_DOWN);
		lifterPIDController = new SimplePIDController(p, i, d);
		lifterPIDController.setInputRange(0, 75.5);
		lifterPIDController.setOutputRange(-maxPowerDown, maxPowerUp);
		lifterPIDController.setAbsoluteTolerance(tolerance);
		lifterPIDController.setSetpoint(setpoint);

		lifterPIDController.start();
	}

	public void liftToHeightPIDInit(double setpoint, double tolerance) {
		double p = Robot.preferences.getDouble(PreferenceKeys.LIFT_P_TERM, DEFAULT_LIFT_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.LIFT_I_TERM, DEFAULT_LIFT_I);
		double d = Robot.preferences.getDouble(PreferenceKeys.LIFT_D_TERM, DEFAULT_LIFT_D);
		lifterPIDControllerInit(p, i, d, setpoint, tolerance);
	}

	public void liftToHeightPIDExecute() {
		double currentPIDOutput = lifterPIDController.update(RobotMap.cubeLiftEncoder.getDistance());

		SmartDashboard.putNumber("Lift To Height PID Error", lifterPIDController.getError());
		SmartDashboard.putNumber("Lift To Height PID Output", currentPIDOutput);

		rawLift(currentPIDOutput);
	}

	public void liftToHeightPIDEnd() {
		lifterPIDController = null;
		stop();
	}
	
	public boolean lifterPIDControllerOnTarget() {
		return lifterPIDController.onTarget();
	}

	public void manualLift(double power) {
		if (power > 0) {
			rawLift(power * LIFT_POWER_SCALE_UP);
		} else {
			rawLift(power * LIFT_POWER_SCALE_DOWN);
		}
	}

	public void rawLift(double power) {
		if (!hasReachedLowerLimit() && power < 0) {
			RobotMap.cubeLifterMotor.set(power);
		} else if (!hasReachedUpperLimit() && power > 0) {
			RobotMap.cubeLifterMotor.set(power);
		} else {
			stop();
		}
	}

	public void stop() {
		RobotMap.cubeLifterMotor.stopMotor();
	}
	
	public boolean hasReachedUpperLimit() {
		return RobotMap.lifterUpperLimitSwitch.get();
	}

	//Reads the Hall Effect sensor which is true when open and false when closed
	public boolean hasReachedLowerLimit() {
		return !RobotMap.lifterLowerLimitSwitch.get();
	}
}
