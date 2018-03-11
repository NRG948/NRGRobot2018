package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualCubeLift extends Command {
	private double prevFinalPower;

	public ManualCubeLift() {
		requires(Robot.cubeLifter);
	}

	protected void initialize() {
		// dummy value other than 0, in case first execute cycle of ManualLift
		// calculates final power to be 0
		prevFinalPower = Double.MAX_VALUE;
		System.out.println("ManualCubeLift init");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double upSpeed = OI.getXBoxTriggerR();
		double downSpeed = OI.getXBoxTriggerL();
		double finalPower = Robot.cubeLifter.hasReachedUpperLimit() || Robot.cubeLifter.hasReachedLowerLimit() ? 0
				: upSpeed - downSpeed;

		// if finalPower is 0, use lift PID controller to maintain current height
		if (finalPower == 0) {
			// if prev execute cycle sent power to motor, initialize PID controller with
			// current height
			if (prevFinalPower != 0) {
				double setPoint = RobotMap.cubeLiftEncoder.getDistance();
				System.out.println("Entering pid mode for setpoint:" + setPoint);
				Robot.cubeLifter.liftToHeightPIDInit(setPoint, 100);
			}
			// send PID-calculated power to motor
			Robot.cubeLifter.liftToHeightPIDExecute();
		} else {
			if (prevFinalPower == 0) {
				System.out.println("Exiting pid mode");
			}
			Robot.cubeLifter.manualLift(finalPower);
		}

		prevFinalPower = finalPower;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeLifter.stop();
		System.out.println("ManualCubeLift end");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
