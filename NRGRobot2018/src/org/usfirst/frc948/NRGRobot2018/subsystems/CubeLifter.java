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
public class CubeLifter extends Subsystem implements PIDOutput {
	private SimplePIDController lifterPIDController;
	private volatile double lifterPIDOutput;

	private static final double LIFT_POWER_SCALE_UP = 0.5;
	private static final double LIFT_POWER_SCALE_DOWN = 0.3;

	public final static double DEFAULT_LIFT_P = 0.02;
	public final static double DEFAULT_LIFT_I = 0.0;
	public final static double DEFAULT_LIFT_D = 0.0;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ManualCubeLift());
	}

	public void lifterPIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
		lifterPIDController = new SimplePIDController(p, i, d, false, RobotMap.cubeLiftEncoder, this);

		lifterPIDController.setInputRange(-1, 1);
		lifterPIDController.setOutputRange(-1, 1);
		lifterPIDController.setAbsoluteTolerance(tolerance);
		lifterPIDController.setSetpoint(setpoint);

		lifterPIDController.start();
	}

	public void liftToHeightPIDInit(double setpoint, double tolerance) {
		lifterPIDControllerInit(Robot.preferences.getDouble(PreferenceKeys.LIFT_P_TERM, DEFAULT_LIFT_P),
				Robot.preferences.getDouble(PreferenceKeys.LIFT_I_TERM, DEFAULT_LIFT_I),
				Robot.preferences.getDouble(PreferenceKeys.LIFT_D_TERM, DEFAULT_LIFT_D), setpoint, tolerance);
	}

	public void liftToHeightPIDExecute() {
		lifterPIDController.update();
		double currentPIDOutput = lifterPIDOutput;

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
		return !RobotMap.lifterUpperLimitSwitch.get();
	}

	public boolean hasReachedLowerLimit() {
		return !RobotMap.lifterLowerLimitSwitch.get();
	}
	
	@Override
	public void pidWrite(double output) {
		lifterPIDOutput = output;
	}
}
