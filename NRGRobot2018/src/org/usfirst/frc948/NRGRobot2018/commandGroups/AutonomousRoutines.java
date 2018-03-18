package org.usfirst.frc948.NRGRobot2018.commandGroups;

import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SCALE_LOW;
import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SWITCH_LEVEL;
import static org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.USE_PID;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.OI.PlateLocation;
import org.usfirst.frc948.NRGRobot2018.Robot.AutoMovement;
import org.usfirst.frc948.NRGRobot2018.Robot.AutoStartingPosition;
import org.usfirst.frc948.NRGRobot2018.commands.DriveStraightDistance;
import org.usfirst.frc948.NRGRobot2018.commands.DriveToXYHeadingPID;
import org.usfirst.frc948.NRGRobot2018.commands.EjectUntilCubeOut;
import org.usfirst.frc948.NRGRobot2018.commands.LiftToHeight;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeLift;
import org.usfirst.frc948.NRGRobot2018.commands.ResetSensors;
import org.usfirst.frc948.NRGRobot2018.commands.SetDriveScale;
import org.usfirst.frc948.NRGRobot2018.commands.TiltAcquirerToAngle;
import org.usfirst.frc948.NRGRobot2018.commands.TurnToHeading;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeTilter;
import org.usfirst.frc948.NRGRobot2018.subsystems.Drive;
import org.usfirst.frc948.NRGRobot2018.utilities.LifterLevel;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.WithinInches;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Selects the auto routine to run based on the SmartDashboard chooser.
 */
public class AutonomousRoutines extends CommandGroup {
	private AutoMovement autoMovement;
    private AutoStartingPosition autoStartingPosition;

    public static final int FIELD_WIDTH_INCHES = 27 * 12;

    public AutonomousRoutines() {
        addSequential(new ResetSensors());
        // addSequential(new SetDriveScale(Drive.SCALE_LOW));

        autoMovement = OI.getAutoMovement();
        autoStartingPosition = OI.getAutoStartingPosition();
        System.out.println("Auto Movement is : " + autoMovement);
        System.out.println("Auto Position is : " + autoStartingPosition);

        switch (autoMovement) {
        case SWITCH:
            System.out.println("Switch side: " + OI.getAllianceSwitchSide());
            if (OI.getAllianceSwitchSide() == PlateLocation.LEFT) {
                if (autoStartingPosition == AutoStartingPosition.LEFT) {
                    addSequential(new LeftToLeftSwitch());
                } else if (autoStartingPosition == AutoStartingPosition.CENTER) {
                    addSequential(new MiddleToLeftSwitch());
                } else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
//                    addSequential(new RightToLeftSwitch());
                	addSequential(new DriveToXYHeadingPID(0, 165, 0));
                }
            } else if (OI.getAllianceSwitchSide() == PlateLocation.RIGHT) {
                if (autoStartingPosition == AutoStartingPosition.LEFT) {
//                    addSequential(new LeftToRightSwitch());
                	addSequential(new DriveToXYHeadingPID(0, 165, 0));
                } else if (autoStartingPosition == AutoStartingPosition.CENTER) {
                    addSequential(new MiddleToRightSwitch());
                } else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
                    addSequential(new RightToRightSwitch());
                }
            }
            break;

        case SCALE:
            System.out.println("Scale side: " + OI.getScaleSide());
            if (OI.getScaleSide() == PlateLocation.LEFT) {
                if (autoStartingPosition == AutoStartingPosition.CENTER) {
                    addSequential(new MiddleToLeftScale());
                } else if (autoStartingPosition == AutoStartingPosition.LEFT) {
                    addSequential(new LeftToLeftScale());
                } else if (autoStartingPosition == AutoStartingPosition.RIGHT){
                	addSequential(new DriveToXYHeadingPID(0, 140, 0));
                }
            } else if (OI.getScaleSide() == PlateLocation.RIGHT) {
                if (autoStartingPosition == AutoStartingPosition.CENTER) {
                    addSequential(new MiddleToRightScale());
                } else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
                    addSequential(new RightToRightScale());
                } else if (autoStartingPosition == AutoStartingPosition.LEFT){
                	addSequential(new DriveToXYHeadingPID(0, 140, 0));
                }
            }
            break;

        case FORWARD:
            addSequential(new DriveToXYHeadingPID(0, 140, 0));
        }
    }
    
    public class DriveAndEject extends CommandGroup {
    	public DriveAndEject(double startX, double startY, Waypoint[] path, double timeout) { // timeout needs to parametrized
    		addSequential(new FollowWaypoints(startX, startY, path), timeout);
    		addSequential(new EjectUntilCubeOut(0.5, 1.0));
    		// so the acquirer doesnt hit the scale/switch when disabled
//            addSequential(new DriveStraightDistance(0.3, 24, Direction.BACKWARD));
    	}
    }
    
    public class LiftToHeightAndHold extends CommandGroup {
    	public LiftToHeightAndHold(LifterLevel level) {
    		addSequential(new LiftToHeight(level));
    		addSequential(new ManualCubeLift());
    	}
    }

//    // Left is default starting position - waypoints are converted if starting
//    // position is right
//    private Waypoint[] convertPath(Waypoint[] path) {
//        Waypoint[] convertedPath = new Waypoint[path.length];
//
//        if (autoStartingPosition == AutoStartingPosition.RIGHT) {
//            for (int i = 0; i < path.length; i++) {
//                Waypoint currWaypoint = path[i];
//                double newX = (currWaypoint.coordinateType == CoordinateType.ABSOLUTE)
//                        ? FIELD_WIDTH_INCHES - currWaypoint.x
//                        : -currWaypoint.x;
//                /*
//                 * if we're thinking of using GreaterThan/LessThan predicates, which we
//                 * currently aren't, add getters for their fields so those can be converted too
//                 */
//
//                convertedPath[i] = new Waypoint(currWaypoint.coordinateType, newX, currWaypoint.y,
//                        -currWaypoint.heading, currWaypoint.waypointPredicate);
//            }
//        }
//
//        return convertedPath;
//    }
//
//    private static final Waypoint SAME_SIDE_SWITCH_PATH[] = {
//            new Waypoint(CoordinateType.RELATIVE, 0.0, 146, 0, new Waypoint.WithinInches(44)),
//            new Waypoint(CoordinateType.RELATIVE, 19, 0, 90, USE_PID) };
//
//    public class SameSideSwitch extends CommandGroup {
//        public SameSideSwitch() {
//            Waypoint[] convertedPath = convertPath(SAME_SIDE_SWITCH_PATH);
//
//            addParallel(new DriveAndEject(0, 0, convertedPath));
//            addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
//            addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
//        }
//    }
//
//    private static final Waypoint OPPOSITE_SIDE_SWITCH_PATH[] = {
//            new Waypoint(CoordinateType.RELATIVE, 0.0, 212, 0, new WithinInches(15)),
//            new Waypoint(CoordinateType.RELATIVE, 242, 0.0, -90, new WithinInches(2)),
//            new Waypoint(CoordinateType.RELATIVE, 0, 0.0, -129, USE_PID),
//            new Waypoint(CoordinateType.RELATIVE, -31.5, -25.5, -135, new WithinInches(2)) };
//
//    public class OppositeSideSwitch extends CommandGroup {
//        public OppositeSideSwitch() {
//            Waypoint[] convertedPath = convertPath(OPPOSITE_SIDE_SWITCH_PATH);
//
//            addParallel(new DriveAndEject(0, 0, convertedPath));
//            addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
//            addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
//        }
//    }

    // Switch auto routines
    private static final Waypoint LEFT_LEFT_SWITCH_PATH[] = {
            new Waypoint(CoordinateType.RELATIVE, 0.0, 146, 0, new Waypoint.WithinInches(44)),
            new Waypoint(CoordinateType.RELATIVE, 19, 0, 90, USE_PID) };

    public class LeftToLeftSwitch extends CommandGroup {
        public LeftToLeftSwitch() {
            addParallel(new DriveAndEject(0, 0, LEFT_LEFT_SWITCH_PATH, 6.0));
            addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
            addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
        }
    }
    
    private static final Waypoint LEFT_RIGHT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, -10.0, 216, 0, new WithinInches(6)),
    		new Waypoint(CoordinateType.RELATIVE, 260, 0.0, -90, new WithinInches(2)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 0.0, -129, USE_PID),
    		new Waypoint(CoordinateType.RELATIVE, -39.5, -29.5, -143, new WithinInches(2)) };
    
    public class LeftToRightSwitch extends CommandGroup {
    	public LeftToRightSwitch() {
    		addParallel(new DriveAndEject(0, 0, LEFT_RIGHT_SWITCH_PATH, 12.0));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    	}
    }
    
    private static final Waypoint MIDDLE_LEFT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, -62, 50, -51, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 45, 0, USE_PID) };
    
    public class MiddleToLeftSwitch extends CommandGroup {
    	public MiddleToLeftSwitch() {
    		addParallel(new DriveAndEject(0, 0, MIDDLE_LEFT_SWITCH_PATH, 6.0));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    	}
    }
    
    private static final Waypoint MIDDLE_RIGHT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, 45, 50, 45, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 45, 0, USE_PID) };
    
    public class MiddleToRightSwitch extends CommandGroup {
    	public MiddleToRightSwitch() {
    		addParallel(new DriveAndEject(0, 0, MIDDLE_RIGHT_SWITCH_PATH, 6.0));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    	}
    }
    
    private static final Waypoint RIGHT_LEFT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, 0.0, 212, 0, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, -242, 0.0, 90, new WithinInches(2)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 0.0, 129, USE_PID),
    		new Waypoint(CoordinateType.RELATIVE, 31.5, -25.5, 141, new WithinInches(2))
    };
    
    public class RightToLeftSwitch extends CommandGroup {
    	public RightToLeftSwitch() {
    		addParallel(new DriveAndEject(0, 0, RIGHT_LEFT_SWITCH_PATH, 12.0));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    	}
    }
    
    private static final Waypoint RIGHT_RIGHT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, 0.0, 146, 0, new Waypoint.WithinInches(44)),
    		new Waypoint(CoordinateType.RELATIVE, -19, 0, -90, USE_PID) };
    
    public class RightToRightSwitch extends CommandGroup {
    	public RightToRightSwitch() {
    		addParallel(new DriveAndEject(0, 0, RIGHT_RIGHT_SWITCH_PATH, 6.0));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    	}
    }

    // Scale auto routines
    public class LeftToLeftScale extends CommandGroup {
        public LeftToLeftScale() {
            addSequential(new SetDriveScale(0.6));
            addSequential(new DriveStraightDistance(1.0, 305.5, Drive.Direction.FORWARD));
            addSequential(new TurnToHeading(90));
            addSequential(new DriveStraightDistance(1.0, 20.875, Drive.Direction.FORWARD));
        }
    }
    
    private static final Waypoint MIDDLE_LEFT_SCALE_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, -118, 112.6, -51.6, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 130, 0, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 37, 17, 45, USE_PID) };
    
    public class MiddleToLeftScale extends CommandGroup {
    	public MiddleToLeftScale() {
    		addParallel(new DriveAndEject(0, 0, MIDDLE_LEFT_SCALE_PATH, 12.0));
    		addParallel(new LiftToHeightAndHold(SCALE_LOW));
    		addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    	}
    }

    private static final Waypoint MIDDLE_RIGHT_SCALE_PATH[] = {
            new Waypoint(CoordinateType.RELATIVE, 120, 112.6, 60, new WithinInches(15)),
            new Waypoint(CoordinateType.RELATIVE, 0, 130, 0, new WithinInches(15)),
            new Waypoint(CoordinateType.RELATIVE, -37, 17, -45, USE_PID) };
    
    public class MiddleToRightScale extends CommandGroup {
        public MiddleToRightScale() {
            addParallel(new DriveAndEject(0, 0, MIDDLE_RIGHT_SCALE_PATH, 12.0));
            addParallel(new LiftToHeightAndHold(SCALE_LOW));
            addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
        }
    }

    private static final Waypoint RIGHT_RIGHT_SCALE_PATH[] = {
            new Waypoint(CoordinateType.RELATIVE, 0.0, 295, 0, new Waypoint.GreaterThanY(280)),
            new Waypoint(CoordinateType.RELATIVE, -21, 0, -90, USE_PID) };

    public class RightToRightScale extends CommandGroup {
        public RightToRightScale() {
            addParallel(new DriveAndEject(0, 0, RIGHT_RIGHT_SCALE_PATH, 12.0));
            addParallel(new LiftToHeightAndHold(SCALE_LOW));
            addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
        }
    }
}
