package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeAcquirer.Direction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TiltAcquirerAndEject extends Command {
	private final double desiredTilterAngle; // in degrees
	private final double acquirerDelay; // in seconds
	private final double acquirerPower;
	
	private final Timer timer;
	private boolean timerStarted;

	public TiltAcquirerAndEject(double tilterAngle, double delayToEject, double acquirerPower) {
		requires(Robot.cubeTilter);
		requires(Robot.cubeAcquirer);
		
		desiredTilterAngle = tilterAngle;
		acquirerDelay = delayToEject;
		this.acquirerPower = acquirerPower;
		
		timer = new Timer();
		timerStarted = false;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.cubeTilter.tiltToAnglePIDIntialize(desiredTilterAngle, 1.0);
		timerStarted = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.cubeTilter.tiltToAnglePIDExecute();
		
		if (Robot.cubeTilter.tiltToAnglePIDOnTarget() && !timerStarted) {
			timer.start();
			timerStarted = true;
		}
		
		if (timerStarted) {
			Robot.cubeAcquirer.acquire(acquirerPower, Direction.EJECT);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return !Robot.cubeAcquirer.isCubeIn() && timer.hasPeriodPassed(acquirerDelay);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeTilter.stop();
		Robot.cubeAcquirer.stop();
		
		timer.reset();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
