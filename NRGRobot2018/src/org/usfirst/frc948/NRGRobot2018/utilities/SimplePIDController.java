package org.usfirst.frc948.NRGRobot2018.utilities;

import edu.wpi.first.wpilibj.PIDController.Tolerance;

public class SimplePIDController {
	private double kP; // factor for "proportional" control
	private double kI; // factor for "integral" control
	private double kD; // factor for "derivative" control
	private double maximumOutput = 1.0; // maximum output|
	private double minimumOutput = -1.0; // minimum output
	private double maximumInput = 0.0; // maximum input - limit setpoint to this
	private double minimumInput = 0.0; // minimum input - limit setpoint to this
	private double inputRange = 0.0; // input range - difference between maximum and minimum
	private boolean wasPIDReset = false; // is the pid controller enabled

	// the sum of the errors for use in the integral calc
	private double integral = 0.0;
	// the tolerance object used to check if on target
	private Tolerance tolerance;
	private double setpoint = 0.0;
	private double error = 0.0;
	private double result = 0.0;
	private double prevError = 0.0; // the prior error (used to compute derivative of error)
	private double prevTime;
	private boolean isIntegralNeededToHoldPosition;

	public SimplePIDController(double p, double i, double d, Tolerance tolerance,
			boolean isIntegralNeededToHoldPosition) {
		kP = p;
		kD = d;
		kI = i;
		this.tolerance = tolerance;
		this.isIntegralNeededToHoldPosition = isIntegralNeededToHoldPosition;
	}

	public void start() {
		prevTime = System.nanoTime() / 1.0e9;
		this.wasPIDReset = true;
		integral = 0.0;
	}

	public double update(double sensorValue) {
		double time = System.nanoTime() / 1.0e9;
		error = setpoint - sensorValue;
		if (wasPIDReset) {
			prevError = error;
			wasPIDReset = false;
		}
		// integral is reset if sensor value overshoots the setpoint
		if (Math.signum(error) != Math.signum(prevError)) {
			integral = 0;
		}
		// TODO: clamp integral
		integral += (error + prevError) * 0.5 * (time - prevTime);

		double derivative = (error - prevError) / (time - prevTime);
		result = kP * error + kI * integral + kD * derivative;
		prevTime = time;
		prevError = error;
		// TODO: clamp result
		return result;
	}

	public SimplePIDController setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		wasPIDReset = true;
		return this;
	}

	public boolean onTarget() {
		return tolerance.onTarget();
	}

}
