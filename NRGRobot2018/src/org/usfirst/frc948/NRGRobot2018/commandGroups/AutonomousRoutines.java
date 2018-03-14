package org.usfirst.frc948.NRGRobot2018.commandGroups;

import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SCALE_LOW;
import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SWITCH_LEVEL;
import static org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.USE_PID;
import static org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.WITHIN_EIGHTEEN_INCHES;
import static org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.WITHIN_TWO_FEET;
import static org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType.ABSOLUTE;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.OI.PlateLocation;
import org.usfirst.frc948.NRGRobot2018.Robot;
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
import org.usfirst.frc948.NRGRobot2018.subsystems.Drive.Direction;
import org.usfirst.frc948.NRGRobot2018.utilities.LifterLevel;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.WithinInches;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Selects the auto routine to run based on the SmartDashboard chooser.
 */
public class AutonomousRoutines extends CommandGroup {

	// Add Commands here:
	// e.g. addSequential(new Command1());
	// addSequential(new Command2());
	// these will run in order.

	// To run multiple commands at the same time,
	// use addParallel()
	// e.g. addParallel(new Command1());
	// addSequential(new Command2());
	// Command1 and Command2 will run in parallel.

	// A command group will require all of the subsystems that each member
	// would require.
	// e.g. if Command1 requires chassis, and Command2 requires arm,
	// a CommandGroup containing them would require both the chassis and the
	// arm.

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
					addSequential(new RedLeftToLeftSwitch());
				} else if (autoStartingPosition == AutoStartingPosition.CENTER) {
					addSequential(new RedMiddleToLeftSwitch());
				} else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
					addSequential(new RedRightToLeftSwitch());
				}
			} else if (OI.getAllianceSwitchSide() == PlateLocation.RIGHT) {
				if (autoStartingPosition == AutoStartingPosition.LEFT) {
					addSequential(new RedLeftToRightSwitch());
				} else if (autoStartingPosition == AutoStartingPosition.CENTER) {
					addSequential(new RedMiddleToRightSwitch());
				} else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
					addSequential(new RedRightToRightSwitch());
				}
			}
			break;

		case SCALE:
			System.out.println("Scale side: " + OI.getScaleSide());
			if (OI.getScaleSide() == PlateLocation.LEFT) {
				if (autoStartingPosition == AutoStartingPosition.CENTER) {
					addSequential(new RedMiddleToLeftScale());
				} else if (autoStartingPosition == AutoStartingPosition.LEFT) {
					addSequential(new RedLeftToLeftScale());
				}
			} else if (OI.getScaleSide() == PlateLocation.RIGHT) {
				if (autoStartingPosition == AutoStartingPosition.CENTER) {
					addSequential(new RedMiddleToRightScale());
				} else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
					addSequential(new RedRightToRightScale());
				}
			}
			break;

		case FORWARD:
			addSequential(new DriveToXYHeadingPID(0, 60, 0));
		}
	}
	
	// Left is default starting position - waypoints are converted if starting position is right
	private Waypoint[] convertPath(Waypoint[] path) {
		Waypoint[] convertedPath = new Waypoint[path.length];
		
		if (autoStartingPosition == AutoStartingPosition.RIGHT) {
			for (int i = 0; i < path.length; i++) {
				Waypoint currWaypoint = path[i];
				double newX = (currWaypoint.coordinateType == CoordinateType.ABSOLUTE) ? FIELD_WIDTH_INCHES - currWaypoint.x : -currWaypoint.x;
				/*
				 * if we're thinking of using GreaterThan/LessThan predicates, which we currently aren't,
				 * add getters for their fields so those can be converted too
				 */

				convertedPath[i] = new Waypoint(currWaypoint.coordinateType, 
												newX, 
												currWaypoint.y, 
												currWaypoint.heading,
												currWaypoint.waypointPredicate);
			}
		}
		
		return convertedPath;
	}
	
	private static final Waypoint SAME_SIDE_SWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 146, 0, new Waypoint.WithinInches(44)),
			new Waypoint(CoordinateType.RELATIVE, 19, 0, 90, USE_PID) };
	
	public class SameSideSwitch extends CommandGroup {
		public SameSideSwitch() {
			Waypoint[] convertedPath = convertPath(SAME_SIDE_SWITCH_PATH);
			
			addParallel(new DriveAndEject(0, 0, convertedPath));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}
	
	private static final Waypoint OPPOSITE_SIDE_SWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 212, 0, new WithinInches(15)),
			new Waypoint(CoordinateType.RELATIVE, 242, 0.0, -90, new WithinInches(2)),
			new Waypoint(CoordinateType.RELATIVE, 0, 0.0, -129, USE_PID),
			new Waypoint(CoordinateType.RELATIVE, -31.5, -25.5, -135, new WithinInches(2)) };
	
	public class OppositeSideSwitch extends CommandGroup {
		public OppositeSideSwitch() {
			Waypoint[] convertedPath = convertPath(OPPOSITE_SIDE_SWITCH_PATH);
			
			addParallel(new DriveAndEject(0, 0, convertedPath));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint RLLS_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 146, 0, new Waypoint.WithinInches(44)),
			new Waypoint(CoordinateType.RELATIVE, 19, 0, 90, USE_PID) };

	public class RedLeftToLeftSwitch extends CommandGroup {
		public RedLeftToLeftSwitch() {
			addParallel(new DriveAndEject(0, 0, RLLS_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	public class RedLeftToLeftScale extends CommandGroup {
		public RedLeftToLeftScale() {
			addSequential(new SetDriveScale(0.6));
			addSequential(new DriveStraightDistance(1.0, 305.5, Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0, 20.875, Drive.Direction.FORWARD));
		}
	}

	private static final Waypoint RLRSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 212, 0, new WithinInches(15)),
			new Waypoint(CoordinateType.RELATIVE, 242, 0.0, -90, new WithinInches(2)),
			new Waypoint(CoordinateType.RELATIVE, 0, 0.0, -129, USE_PID),
			new Waypoint(CoordinateType.RELATIVE, -31.5, -25.5, -135, new WithinInches(2)) };

	public class RedLeftToRightSwitch extends CommandGroup {
		public RedLeftToRightSwitch() {
			addParallel(new DriveAndEject(0, 0, RLRSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	public class RedMiddleToRightSwitch extends CommandGroup {
		public RedMiddleToRightSwitch() {
			addSequential(new DriveStraightDistance(1.0, 108.75, Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0, 149.5, Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0, 20, Drive.Direction.FORWARD));
		}
	}

	public class RedMiddleToLeftSwitch extends CommandGroup {
		public RedMiddleToLeftSwitch() {
			addSequential(new DriveStraightDistance(1.0, 84.75, Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0, 148.5, Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0, 20, Drive.Direction.FORWARD));
		}
	}

	public class RedMiddleToRightScale extends CommandGroup {
		public RedMiddleToRightScale() {

		}
	}

	public class RedMiddleToLeftScale extends CommandGroup {
		public RedMiddleToLeftScale() {

		}
	}

	private static final Waypoint RRRS_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 146, 0, new Waypoint.WithinInches(44)),
			new Waypoint(CoordinateType.RELATIVE, -19, 0, -90, USE_PID) };

	public class RedRightToRightSwitch extends CommandGroup {
		public RedRightToRightSwitch() {
			addParallel(new DriveAndEject(0, 0, RRRS_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	public class DriveAndEject extends CommandGroup {
		public DriveAndEject(double startX, double startY, Waypoint[] path) {
			addSequential(new FollowWaypoints(startX, startY, path));
			addSequential(new EjectUntilCubeOut(0.5, 1.0));
			// so the acquirer doesnt hit the scale/switch when disabled
			addSequential(new DriveStraightDistance(0.3, 24, Direction.BACKWARD));
		}
	}

	public class LiftToHeightAndHold extends CommandGroup {
		public LiftToHeightAndHold(LifterLevel level) {
			addSequential(new LiftToHeight(level));
			addSequential(new ManualCubeLift());
		}
	}

	private static final Waypoint RRRSCALE_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 309, 0, new Waypoint.GreaterThanY(280)),
			new Waypoint(CoordinateType.RELATIVE, -21, 0, -90, USE_PID) };

	public class RedRightToRightScale extends CommandGroup {
		public RedRightToRightScale() {
			addParallel(new DriveAndEject(0, 0, RRRSCALE_PATH));
			addParallel(new LiftToHeightAndHold(SCALE_LOW));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint RRLSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0.0, 212, 0, new WithinInches(15)),
			new Waypoint(CoordinateType.RELATIVE, -242, 0.0, 90, new WithinInches(2)),
			new Waypoint(CoordinateType.RELATIVE, 0, 0.0, 129, USE_PID),
			new Waypoint(CoordinateType.RELATIVE, 31.5, -25.5, 135, new WithinInches(2))

	};

	public class RedRightToLeftSwitch extends CommandGroup {
		public RedRightToLeftSwitch() {
			addParallel(new DriveAndEject(0, 0, RRLSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BLLSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 149.5, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, -20.875, 0, -90, USE_PID) };

	public class BlueLeftToLeftSwitch extends CommandGroup {
		public BlueLeftToLeftSwitch() {
			addParallel(new DriveAndEject(0, 0, BLLSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BLLSCALE_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 305.5, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, -20.875, 0, -90, USE_PID) };

	public class BlueLeftToLeftScale extends CommandGroup {
		public BlueLeftToLeftScale() {
			addParallel(new DriveAndEject(0, 0, BLLSCALE_PATH));
			addParallel(new LiftToHeightAndHold(SCALE_LOW));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BLRSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 207, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 212.75, 0, 90, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 0, -45, 180, WITHIN_EIGHTEEN_INCHES),
			new Waypoint(CoordinateType.RELATIVE, -20, 0, 270, USE_PID) };

	public class BlueLeftToRightSwitch extends CommandGroup {
		public BlueLeftToRightSwitch() {
			addParallel(new DriveAndEject(0, 0, BLRSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BMRSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 108.75, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 149.5, 0, 90, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 0, 20, 0, USE_PID) };

	public class BlueMiddleToRightSwitch extends CommandGroup {
		public BlueMiddleToRightSwitch() {
			addParallel(new DriveAndEject(0, 0, BMRSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BMLSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 84.75, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, -148.5, 0, -90, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 0, 20, 0, USE_PID) };

	public class BlueMiddleToLeftSwitch extends CommandGroup {
		public BlueMiddleToLeftSwitch() {
			addParallel(new DriveAndEject(0, 0, BMLSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BMRSCALE_PATH[] = {

	};

	public class BlueMiddleToRightScale extends CommandGroup {
		public BlueMiddleToRightScale() {
			addParallel(new DriveAndEject(0, 0, BMRSCALE_PATH));
			addParallel(new LiftToHeightAndHold(SCALE_LOW));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BMLSCALE_PATH[] = {

	};

	public class BlueMiddleToLeftScale extends CommandGroup {
		public BlueMiddleToLeftScale() {
			addParallel(new DriveAndEject(0, 0, BMLSCALE_PATH));
			addParallel(new LiftToHeightAndHold(SCALE_LOW));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BRRSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 149.5, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, -20.875, 0, -90, USE_PID) };

	public class BlueRightToRightSwitch extends CommandGroup {
		public BlueRightToRightSwitch() {
			addParallel(new DriveAndEject(0, 0, BRRSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BRRSCALE_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 305.5, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 7.68, 0, 90, USE_PID) };

	public class BlueRightToRightScale extends CommandGroup {
		public BlueRightToRightScale() {
			addParallel(new DriveAndEject(0, 0, BRRSCALE_PATH));
			addParallel(new LiftToHeightAndHold(SCALE_LOW));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}

	private static final Waypoint BRLSWITCH_PATH[] = {
			new Waypoint(CoordinateType.RELATIVE, 0, 207, 0, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, -212.75, 0, -90, WITHIN_TWO_FEET),
			new Waypoint(CoordinateType.RELATIVE, 0, -45, -180, WITHIN_EIGHTEEN_INCHES),
			new Waypoint(CoordinateType.RELATIVE, 20, 0, -270, USE_PID) };

	public class BlueRightToLeftSwitch extends CommandGroup {
		public BlueRightToLeftSwitch() {
			addParallel(new DriveAndEject(0, 0, BRLSWITCH_PATH));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
		}
	}
}
