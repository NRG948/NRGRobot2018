package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeAcquirer.Direction;

import edu.wpi.first.wpilibj.command.Command;

/**
 * AcquireUntilCubeDetected: Run acquirer wheels inward until cube sensor activated.
 */
public class AcquireUntilCubeDetected extends Command {
	private final double power;

	public AcquireUntilCubeDetected(double power) {
		requires(Robot.cubeAcquirer);
		
		this.power = power;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.cubeAcquirer.acquire(power, Direction.ACQUIRE);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.cubeAcquirer.isCubeIn();
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
