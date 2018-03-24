package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *Interrupts the command running on every subsystem
 */
public class InterruptCommands extends Command {

    public InterruptCommands() {
        requires(Robot.drive);
        requires(Robot.climber);
        requires(Robot.cubeAcquirer);
        requires(Robot.cubeLifter);
        requires(Robot.cubeTilter);
    }
    
    public InterruptCommands(Subsystem... subsystems)
	{
		for (Subsystem subsystem : subsystems)
		{
			requires(subsystem);
		}
	}
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("InterruptCommands");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("InterruptCommands End");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
