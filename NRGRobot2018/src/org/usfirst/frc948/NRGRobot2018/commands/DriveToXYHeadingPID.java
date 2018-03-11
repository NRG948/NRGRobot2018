package org.usfirst.frc948.NRGRobot2018.commands;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.RobotMap;
import org.usfirst.frc948.NRGRobot2018.utilities.PositionTracker;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.WaypointPredicate;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * DriveToXYHeadingNoPID: Drives/strafes to target x and y (inches), and heading
 * (degrees)
 */
public class DriveToXYHeadingPID extends Command {
    final double DISTANCE_TOLERANCE = 1.0;
    final double ANGLE_TOLERANCE = 1.0;
    
    final private Waypoint waypoint;
    final private WaypointPredicate predicate;
    final private boolean isFinalWaypoint;

    final double desiredX; // desired x
    final double desiredY; // desired y
    final double desiredHeading; // desired heading
    
    
    private double dXFieldFrame; // (desired x) - (current robot x position)
    private double dYFieldFrame; // (desired y) - (current robot y position)

    public DriveToXYHeadingPID(double x, double y, double heading) {
        this(new Waypoint(CoordinateType.ABSOLUTE, x, y, heading, Waypoint.USE_PID), true);
    }

    public DriveToXYHeadingPID(Waypoint waypoint, boolean isFinalWaypoint) {
        requires(Robot.drive);
        
        this.waypoint = waypoint;
        this.predicate = waypoint.waypointPredicate;
        this.isFinalWaypoint = isFinalWaypoint;
        
        desiredX = waypoint.x;
        desiredY = waypoint.y;
        Robot.positionTracker.setXYGoal(desiredX, desiredY);
        desiredHeading = waypoint.heading;
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        // The goal is to drive dx and dy to zero and point toward the desired heading
        Robot.drive.xPIDControllerInit(0, DISTANCE_TOLERANCE);
        Robot.drive.yPIDControllerInit(0, DISTANCE_TOLERANCE);
        Robot.drive.turnPIDControllerInit(desiredHeading, ANGLE_TOLERANCE);

        dXFieldFrame = Double.MAX_VALUE;
        dYFieldFrame = Double.MAX_VALUE;
        
        SmartDashboard.putNumber("currentWaypointX:", waypoint.x);
        SmartDashboard.putNumber("currentWaypointY:", waypoint.y);
        SmartDashboard.putNumber("currentWaypointH:", waypoint.heading);
        
        System.out.println(waypoint);
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

        double headingRobotFrame = Math.atan2(dXFieldFrame, dYFieldFrame) - Math.toRadians(currHeading); // all in radians
        double dXRobotFrame = -distanceToTarget * Math.sin(headingRobotFrame); // x-displacement from (0, 0) in robot coordinate frame
        double dYRobotFrame = -distanceToTarget * Math.cos(headingRobotFrame); // y-displacement from (0, 0) in robot coordinate frame

        // calculating powers
        double xPower = Robot.drive.xPIDControllerExecute(dXRobotFrame);
        double yPower = Robot.drive.yPIDControllerExecute(dYRobotFrame);
        double turnPower = Robot.drive.turnPIDControllerExecute(currHeading);

        // sending calculated powers
        Robot.drive.rawDriveCartesian(xPower, yPower, turnPower);

        SmartDashboard.putNumber("driveToXYHeading/Xpower", xPower);
        SmartDashboard.putNumber("driveToXYHeading/Xerror", Robot.drive.getXError());
        SmartDashboard.putNumber("driveToXYHeading/Ypower", yPower);
        SmartDashboard.putNumber("driveToXYHeading/Yerror", Robot.drive.getYError());
        SmartDashboard.putNumber("driveToXYHeading/turnPower", turnPower);
        SmartDashboard.putNumber("driveToXYHeading/turnError", Robot.drive.getTurnError());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return predicate.isFinished(waypoint) || Robot.drive.allControllersOnTarget(isFinalWaypoint);
    }

    // Called once after isFinished returns true
    protected void end() {
        if (isFinalWaypoint) {
            Robot.drive.stop();
            SmartDashboard.putNumber("DriveToXYHeading/gyro", RobotMap.gyro.getAngle());
        }
        System.out.println("DriveToXYHeading " + Robot.positionTracker);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.drive.stop();
    }
}
