package org.usfirst.frc948.NRGRobot2018.commands;

import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeAcquirer.Direction;

import edu.wpi.first.wpilibj.command.Command;

/**
 * AcquireUntilCubeDetected: Run acquirer wheels inward until cube sensor activated.
 */
public class EjectUntilCubeOut extends Command {
	private final double power, delay; // Delay (seconds) is to make sure cube has been completely ejected
	private final Timer timer;

	public EjectUntilCubeOut(double power, double delay) {
		requires(Robot.cubeAcquirer);
		
		this.power = power;
		this.delay = delay;
		
		timer = new Timer();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.cubeAcquirer.acquire(power, Direction.EJECT);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return !Robot.cubeAcquirer.isCubeIn() && timer.hasPeriodPassed(delay);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeAcquirer.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
