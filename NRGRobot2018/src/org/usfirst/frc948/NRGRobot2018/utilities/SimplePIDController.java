package org.usfirst.frc948.NRGRobot2018.utilities;

import edu.wpi.first.wpilibj.PIDController.Tolerance;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDController.NullTolerance;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class SimplePIDController {
	private double kP; // factor for "proportional" control
	private double kI; // factor for "integral" control
	private double kD; // factor for "derivative" control

	private double minimumOutput = -1.0; // minimum output
	private double maximumOutput = 1.0; // maximum output
	private double minimumInput = -Double.MAX_VALUE; // minimum input - limit setpoint to this
	private double maximumInput = Double.MAX_VALUE; // maximum input - limit setpoint to this

	private PIDSource source;
	private PIDOutput output;
	private boolean wasPIDReset = false; // is the pid controller enabled

	// the sum of the errors for use in the integral calc
	private double integral = 0.0;
	// the tolerance object used to check if on target
	private Tolerance tolerance;
	private double setpoint = 0.0;

	private double result = 0.0;
	private double prevInput;

	private double prevError = 0.0; // the prior error (used to compute derivative of error)
	private double prevTime;
	private boolean isIntegralNeededToHoldPosition;

	public SimplePIDController(double p, double i, double d, boolean isIntegralNeededToHoldPosition, PIDSource source,
			PIDOutput output) {
		this(p, i, d, isIntegralNeededToHoldPosition);

		this.source = source;
		this.output = output;
	}

	public SimplePIDController(double p, double i, double d, boolean isIntegralNeededToHoldPosition) {
		kP = p;
		kD = d;
		kI = i;
		this.tolerance = new Tolerance() {

			@Override
			public boolean onTarget() {
				throw new RuntimeException("tolerance needs to be specified explicitly");
			}
		};
		this.isIntegralNeededToHoldPosition = isIntegralNeededToHoldPosition;
	}

	public SimplePIDController(double p, double i, double d) {
		this(p, i, d, false);
	}

	public SimplePIDController start() {
		prevTime = System.nanoTime() / 1.0e9;
		this.wasPIDReset = true;
		integral = 0.0;
		return this;
	}

	public double update(double input, double setpoint) {
		this.setpoint = setpoint;
		return update(input);
	}

	public double update(double input) {
		double currTime = System.nanoTime() / 1.0e9;
		double deltaTime = currTime - prevTime;
		double error = setpoint - input;

		input = MathUtil.clamp(input, minimumInput, maximumInput);

		if (wasPIDReset) {
			prevError = error;
			wasPIDReset = false;
		}

		double derivative = (error - prevError) / deltaTime;
		double pdTerms = kP * error + kD * derivative;

		// only integrate when close to setpoint
		if (pdTerms >= maximumOutput) {
			result = maximumOutput;
			integral = 0;
		} else if (pdTerms <= minimumOutput) {
			result = minimumOutput;
			integral = 0;
		} else {
			// integral is reset if sensor value overshoots the setpoint
			if (!isIntegralNeededToHoldPosition && Math.signum(error) != Math.signum(prevError)) {
				integral = 0;
			}
			integral += kI * (error + prevError) * 0.5 * deltaTime;
			result = MathUtil.clamp(pdTerms + integral, minimumOutput, maximumOutput);
		}

		prevTime = currTime;
		prevError = error;
		prevInput = input;

		return result;
	}

	public void update() {
		double input = source.pidGet();
		double result = update(input);
		output.pidWrite(result);
	}

	public SimplePIDController setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		wasPIDReset = true;
		return this;
	}

	public SimplePIDController setInputRange(double minimumInput, double maximumInput) {
		this.minimumInput = minimumInput;
		this.maximumInput = maximumInput;
		return this;
	}

	public SimplePIDController setOutputRange(double minimumOutput, double maximumOutput) {
		this.minimumOutput = minimumOutput;
		this.maximumOutput = maximumOutput;
		return this;
	}

	public SimplePIDController setAbsoluteTolerance(final double absoluteTolerance) {
		tolerance = new Tolerance() {
			private double tolerance = Math.abs(absoluteTolerance);

			@Override
			public boolean onTarget() {
				return Math.abs(setpoint - prevInput) <= tolerance;
			}
		};
		return this;
	}

	public double getError() {
		return prevError;
	}

	public boolean onTarget() {
		return tolerance.onTarget();
	}
}
