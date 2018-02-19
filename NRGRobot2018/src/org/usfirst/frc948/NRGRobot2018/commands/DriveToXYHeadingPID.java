package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * DriveToXYHeadingNoPID: Drives/strafes to target x and y (inches), and heading
 * (degrees)
 */
public class DriveToXYHeadingPID extends Command {
    final double DISTANCE_TOLERANCE = 5.0;
    final double ANGLE_TOLERANCE = 5.0;
    
    final double desiredX; // desired x
    final double desiredY; // desired y
    final double desiredHeading; // desired heading
    final private Waypoint.Predicate predicate;
    final private boolean stopAtEnd;
    
    private double dXFieldFrame; // (desired x) - (current robot x position)
    private double dYFieldFrame; // (desired y) - (current robot y position)


    public DriveToXYHeadingPID(double x, double y, double heading) {
        this(x, y, heading, new Waypoint.DefaultPredicate(), true);
    }

    public DriveToXYHeadingPID(double x, double y, double heading, Waypoint.Predicate predicate, boolean stopAtEnd) {
        requires(Robot.drive);
        desiredX = x;
        desiredY = y;
        desiredHeading = heading;
        this.predicate = predicate;
        this.stopAtEnd = stopAtEnd;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        // The goal is to drive dx and dy to zero and point toward the desired heading
        Robot.drive.xPIDControllerInit(0, DISTANCE_TOLERANCE);
        Robot.drive.yPIDControllerInit(0, DISTANCE_TOLERANCE);
        Robot.drive.turnPIDControllerInit(desiredHeading, ANGLE_TOLERANCE);

        dXFieldFrame = Double.MAX_VALUE;
        dYFieldFrame = Double.MAX_VALUE;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        // used for calculating powers in x and y directions
        double currX = Robot.positionTracker.getX();
        double currY = Robot.positionTracker.getY();
        double currHeading = RobotMap.gyro.getAngle();

        dXFieldFrame = desiredX - currX;
        dYFieldFrame = desiredY - currY;
        double distanceToTarget = Math.sqrt(dXFieldFrame * dXFieldFrame + dYFieldFrame * dYFieldFrame);

        double headingRobotFrame = Math.atan2(dXFieldFrame, dYFieldFrame) - Math.toRadians(currHeading);
        double dXRobotFrame = distanceToTarget * Math.sin(headingRobotFrame); // desired x in robot coordinate frame
        double dYRobotFrame = distanceToTarget * Math.cos(headingRobotFrame); // desired y in robot coordinate frame

        // calculating powers
        double xPower = Robot.drive.xPIDControllerExecute(dXRobotFrame);
        double yPower = Robot.drive.yPIDControllerExecute(dYRobotFrame);
        double turnPower = Robot.drive.turnPIDControllerExecute(currHeading);

        // sending calculated powers
        Robot.drive.rawDriveCartesian(xPower, yPower, turnPower);

        SmartDashboard.putNumber("driveToXYHeading/dXFieldFrame", dXFieldFrame);
        SmartDashboard.putNumber("driveToXYHeading/dYFieldFrame", dYFieldFrame);
        SmartDashboard.putNumber("driveToXYHeading/dHeadingToTarget", desiredHeading - currHeading);
        SmartDashboard.putNumber("driveToXYHeading/dXRobotFrame", dXRobotFrame);
        SmartDashboard.putNumber("driveToXYHeading/dYRobotFrame", dYRobotFrame);
        SmartDashboard.putNumber("driveToXYHeading/dHeadingToXY", headingRobotFrame);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return predicate.isAtWaypoint() || (Robot.drive.xPIDControllerOnTarget() && Robot.drive.yPIDControllerOnTarget()
                && Robot.drive.turnPIDControllerOnTarget());
    }

    // Called once after isFinished returns true
    protected void end() {
        if (stopAtEnd) {
            Robot.drive.stop();
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.drive.stop();
    }
}
