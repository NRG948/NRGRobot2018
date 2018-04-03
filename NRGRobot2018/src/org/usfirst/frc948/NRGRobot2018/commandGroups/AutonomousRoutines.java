package org.usfirst.frc948.NRGRobot2018.commandGroups;

import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SCALE_LOW;
import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SCALE_MEDIUM;
import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SCALE_HIGH;
import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.STOWED;
import static org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter.SWITCH_LEVEL;
import static org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.USE_PID;

import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.OI.PlateLocation;
import org.usfirst.frc948.NRGRobot2018.Robot.AutoMovement;
import org.usfirst.frc948.NRGRobot2018.Robot.AutoStartingPosition;
import org.usfirst.frc948.NRGRobot2018.commands.DelaySeconds;
import org.usfirst.frc948.NRGRobot2018.commands.DriveStraightDistance;
import org.usfirst.frc948.NRGRobot2018.commands.DriveStraightDistanceTank;
import org.usfirst.frc948.NRGRobot2018.commands.EjectUntilCubeOut;
import org.usfirst.frc948.NRGRobot2018.commands.LiftToHeight;
import org.usfirst.frc948.NRGRobot2018.commands.ManualCubeLift;
import org.usfirst.frc948.NRGRobot2018.commands.ResetSensors;
import org.usfirst.frc948.NRGRobot2018.commands.SetDriveScale;
import org.usfirst.frc948.NRGRobot2018.commands.TiltAcquirerDown;
import org.usfirst.frc948.NRGRobot2018.commands.TiltAcquirerUp;
import org.usfirst.frc948.NRGRobot2018.commands.TurnToHeading;
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
	public static final int FIELD_LENGTH_INCHES = 54 * 12;
	public static final int FIELD_WIDTH_INCHES = 27 * 12;

	public static final double BUMPER_THICKNESS = 3.0;
	public static final double ROBOT_HALF_LENGTH = 39.0 / 2;
	public static final double ROBOT_HALF_WIDTH = 34.0 / 2;
	public static final double HALF_LENGTH_AND_BUMPER = ROBOT_HALF_LENGTH + BUMPER_THICKNESS;
	public static final double HALF_WIDTH_AND_BUMPER = ROBOT_HALF_WIDTH + BUMPER_THICKNESS;
	public static final double ACQUIRER_EXTRA_LENGTH = 13.0;
	private static final double TANK_POWER = 0.7;

	private AutoMovement autoMovement;
    private AutoStartingPosition autoStartingPosition;

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
//						 addSequential(new RightToLeftSwitch());
						//addSequential(new DriveStraightDistanceTank(TANK_POWER, 140));
						addSequential(new RightToLeftSwitchDriveStraight());
					}
				} else if (OI.getAllianceSwitchSide() == PlateLocation.RIGHT) {
					if (autoStartingPosition == AutoStartingPosition.LEFT) {
//						addSequential(new LeftToRightSwitch());
						 //addSequential(new DriveStraightDistanceTank(TANK_POWER, 140));
						addSequential(new LeftToRightSwitchDriveStraight());
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
						addSequential(new LeftToLeftScaleDriveStraight());
					} else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
						// addSequential(new DriveToXYHeadingPID(0, 140, 0));
						addSequential(new RightToLeftScaleDriveStraight());
					}
				} else if (OI.getScaleSide() == PlateLocation.RIGHT) {
					if (autoStartingPosition == AutoStartingPosition.CENTER) {
						addSequential(new MiddleToRightScale());
					} else if (autoStartingPosition == AutoStartingPosition.RIGHT) {
						addSequential(new RightToRightScaleDriveStraight());
					} else if (autoStartingPosition == AutoStartingPosition.LEFT) {
						// addSequential(new DriveToXYHeadingPID(0, 140, 0));
						// addSequential(new DriveStraightDistanceTank(TANK_POWER, 140));
						addSequential(new LeftToRightScaleDriveStraight());
					}
				}
				break;
	
			case FORWARD:
				addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
				addSequential(new DriveStraightDistanceTank(TANK_POWER, 200), 4);
				addSequential(new LiftToHeightAndHold(SCALE_MEDIUM));
		}
    }

    /*
     * Helper command groups
     */
    public class TurnAndDriveToCube extends CommandGroup {
    	public TurnAndDriveToCube(double desiredHeading) {
    		addSequential(new TurnToHeading(desiredHeading));
    		addSequential(new DriveToCubeAndGrab());
    	}
    }
    
    public class DriveAndEject extends CommandGroup {
    	public DriveAndEject(double startX, double startY, Waypoint[] path, double timeout) { // timeout needs to parametrized
    		addSequential(new FollowWaypoints(startX, startY, path), timeout);
    		addSequential(new EjectUntilCubeOut(0.5, 1));
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

    public class DelayThenLift extends CommandGroup {
    	public DelayThenLift(double delaySeconds, LifterLevel level) {
    		addSequential(new DelaySeconds(delaySeconds));
    		addSequential(new LiftToHeightAndHold(level));
    	}
    }

    /*
     *  Switch auto routines
     */
    private static final Waypoint LEFT_LEFT_SWITCH_PATH[] = {
            new Waypoint(CoordinateType.RELATIVE, 0, 146, 0, new Waypoint.WithinInches(44)),
            new Waypoint(CoordinateType.RELATIVE, 19, 0, 90, USE_PID) };

    public class LeftToLeftSwitch extends CommandGroup {
        public LeftToLeftSwitch() {
            addParallel(new DriveAndEject(0, 0, LEFT_LEFT_SWITCH_PATH, 6));
            addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
            //addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
            addSequential(new TiltAcquirerDown(1));
        }
    }
    
    private static final Waypoint LEFT_RIGHT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, -10, 216, 0, new WithinInches(6)),
    		new Waypoint(CoordinateType.RELATIVE, 260, 0, -90, new WithinInches(2)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 0, -129, USE_PID),
    		new Waypoint(CoordinateType.RELATIVE, -39.5, -29.5, -143, new WithinInches(2)) };
    
//    private static final Waypoint LEFT_RIGHT_SWITCH_FIRST_WAYPOINT = 
//    		new Waypoint(CoordinateType.RELATIVE, -10, 216, 0, USE_PID);
//    
//    private static final Waypoint LEFT_RIGHT_SWITCH_PATH_LAST_SECTION[] = {
//    		new Waypoint(CoordinateType.RELATIVE, 0, 0, -129, USE_PID),
//    		new Waypoint(CoordinateType.RELATIVE, -39.5, -29.5, -143, new WithinInches(2)) };    		
    
    public class LeftToRightSwitch extends CommandGroup {
    	public LeftToRightSwitch() {
    		addParallel(new DriveAndEject(0, 0, LEFT_RIGHT_SWITCH_PATH, 15));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		//addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    		 addSequential(new TiltAcquirerDown(1));
    		 
//    		addParallel(new DriveToXYHeadingPID(LEFT_RIGHT_SWITCH_FIRST_WAYPOINT, true));
//    		addParallel(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
//    		addSequential(new LiftToHeightAndHold(SWITCH_LEVEL));
//    		addSequential(new TurnToHeading(-90));
//    		addSequential(new DriveStraightDistance(1, 260, Direction.BACKWARD));
//    		addSequential(new DriveAndEject(0, 0, LEFT_RIGHT_SWITCH_PATH_LAST_SECTION, 2.5));
    	}
    }
    
    private static final Waypoint MIDDLE_LEFT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, -62, 50, -51, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 52, 0, new WithinInches(3)) };
    
    public class MiddleToLeftSwitch extends CommandGroup {
    	public MiddleToLeftSwitch() {
    		addParallel(new DriveAndEject(0, 0, MIDDLE_LEFT_SWITCH_PATH, 6));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		//addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    		 addSequential(new TiltAcquirerDown(1));
    	}
    }
    
    private static final Waypoint MIDDLE_RIGHT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, 45, 50, 45, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 52, 0, new WithinInches(3)) };
    
    public class MiddleToRightSwitch extends CommandGroup {
    	public MiddleToRightSwitch() {
    		addParallel(new DriveAndEject(0, 0, MIDDLE_RIGHT_SWITCH_PATH, 6));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		//addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    		 addSequential(new TiltAcquirerDown(1));
    	}
    }
    
    private static final Waypoint RIGHT_LEFT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, 10, 216, 0, new WithinInches(6)),
    		new Waypoint(CoordinateType.RELATIVE, -260, 0, 90, new WithinInches(2)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 0, 129, USE_PID),
    		new Waypoint(CoordinateType.RELATIVE, 39.5, -29.5, 143, new WithinInches(2))
    };
    
    
    
    public class RightToLeftSwitch extends CommandGroup {
    	public RightToLeftSwitch() {
    		addParallel(new DriveAndEject(0, 0, RIGHT_LEFT_SWITCH_PATH, 12));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		//addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    		 addSequential(new TiltAcquirerDown(1));
    	}
    }
    
    private static final Waypoint RIGHT_RIGHT_SWITCH_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, 0, 146, 0, new Waypoint.WithinInches(44)),
    		new Waypoint(CoordinateType.RELATIVE, -19, 0, -90, USE_PID) };
    
    public class RightToRightSwitch extends CommandGroup {
    	public RightToRightSwitch() {
    		addParallel(new DriveAndEject(0, 0, RIGHT_RIGHT_SWITCH_PATH, 6));
    		addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
    		//addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    		 addSequential(new TiltAcquirerDown(1));
    	}
    }

    /*
     *  Scale auto routines
     */
    public class LeftToLeftScaleDriveStraight extends CommandGroup {
        public LeftToLeftScaleDriveStraight() {
            addSequential(new SetDriveScale(0.6));
            addSequential(new DriveStraightDistanceTank(TANK_POWER, 305.5));
            addSequential(new TurnToHeading(90));
            addSequential(new DriveStraightDistanceTank(0.6, 20.875));
        }
    }
    
    private static final Waypoint MIDDLE_LEFT_SCALE_PATH[] = {
    		new Waypoint(CoordinateType.RELATIVE, -118, 112.6, -51.6, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 0, 130, 0, new WithinInches(15)),
    		new Waypoint(CoordinateType.RELATIVE, 37, 17, 45, USE_PID) };
    
    public class MiddleToLeftScale extends CommandGroup {
    	public MiddleToLeftScale() {
    		addParallel(new DriveAndEject(0, 0, MIDDLE_LEFT_SCALE_PATH, 12));
    		addParallel(new LiftToHeightAndHold(SCALE_LOW));
    		//addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
    		 addSequential(new TiltAcquirerDown(1));
    	}
    }

    private static final Waypoint MIDDLE_RIGHT_SCALE_PATH[] = {
            new Waypoint(CoordinateType.RELATIVE, 120, 112.6, 60, new WithinInches(15)),
            new Waypoint(CoordinateType.RELATIVE, 0, 130, 0, new WithinInches(15)),
            new Waypoint(CoordinateType.RELATIVE, -37, 17, -45, USE_PID) };
    
    public class MiddleToRightScale extends CommandGroup {
        public MiddleToRightScale() {
            addParallel(new DriveAndEject(0, 0, MIDDLE_RIGHT_SCALE_PATH, 12));
            addParallel(new LiftToHeightAndHold(SCALE_LOW));
            //addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
            addSequential(new TiltAcquirerDown(1));
        }
    }

    private static final Waypoint RIGHT_RIGHT_SCALE_PATH[] = {
            new Waypoint(CoordinateType.RELATIVE, 0, 295, 0, new Waypoint.GreaterThanY(280)),
            new Waypoint(CoordinateType.RELATIVE, -21, 0, -90, USE_PID) };

    public class RightToRightScale extends CommandGroup {
        public RightToRightScale() {
            addParallel(new DriveAndEject(0, 0, RIGHT_RIGHT_SCALE_PATH, 12));
            addParallel(new LiftToHeightAndHold(SCALE_LOW));
            //addSequential(new TiltAcquirerToAngle(CubeTilter.TILTER_DOWN));
            addSequential(new TiltAcquirerDown(1));
        }
    }
   
    
    /*
     *  DriveStraightDistanceTank backup routines
     */
	
	public class LeftToLeftSwitchDriveStraight extends CommandGroup {
		public LeftToLeftSwitchDriveStraight() {
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addParallel(new DriveStraightDistanceTank(TANK_POWER, 168 - HALF_LENGTH_AND_BUMPER));
			
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistanceTank(0.6, 15), 2);
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
	}
	
	public class RightToLeftSwitchDriveStraight extends CommandGroup {
		public RightToLeftSwitchDriveStraight() {
			addParallel(new TiltAcquirerDown(1));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, 236 - HALF_LENGTH_AND_BUMPER));
			
			addSequential(new TurnToHeading(90));
			
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, -246));
			
			addSequential(new TurnToHeading(127));
			addSequential(new DriveStraightDistanceTank(0.6, 44), 3.0);
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
	}
	
	public class LeftToRightSwitchDriveStraight extends CommandGroup {
		public LeftToRightSwitchDriveStraight() {
			addParallel(new TiltAcquirerDown(1));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, 236 - HALF_LENGTH_AND_BUMPER));
			
			addSequential(new TurnToHeading(-90));
			
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, -246));
			
			addSequential(new TurnToHeading(-127));
			addSequential(new DriveStraightDistanceTank(0.6, 44), 3.0);
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
	}
	
	public class MiddleToRightSwitchDriveStraight extends CommandGroup {
		public MiddleToRightSwitchDriveStraight() {
			addParallel(new TiltAcquirerDown(1));
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new DriveStraightDistanceTank(0.6, 37 - HALF_LENGTH_AND_BUMPER));
			
			addSequential(new TurnToHeading(45));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, 67));
			addSequential(new TurnToHeading(0));
			addSequential(new DriveStraightDistanceTank(0.6, 56 - HALF_LENGTH_AND_BUMPER), 3.0);
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
	}
	
	public class RightToRightSwitchDriveStraight extends CommandGroup {
		public RightToRightSwitchDriveStraight() {
			addParallel(new LiftToHeightAndHold(SWITCH_LEVEL));
			addParallel(new DriveStraightDistanceTank(TANK_POWER, 168 - HALF_LENGTH_AND_BUMPER));
			
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistanceTank(0.6, 15), 2);
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
	}   
	
	public class LeftToRightScaleDriveStraight extends CommandGroup {
		public LeftToRightScaleDriveStraight() {
			addParallel(new TiltAcquirerDown(1));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, 236 - HALF_LENGTH_AND_BUMPER));
			
			addSequential(new TurnToHeading(90));
			
			addParallel(new LiftToHeightAndHold(SCALE_LOW));
			addSequential(new DriveStraightDistanceTank(TANK_POWER, 206));
			
			addSequential(new TurnToHeading(0));
			addSequential(new DriveStraightDistanceTank(0.6, 17));
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
	}
    
    public class RightToRightScaleDriveStraight extends CommandGroup {
        public RightToRightScaleDriveStraight() {
        	addParallel(new LiftToHeight(SCALE_MEDIUM));
        	addParallel(new TiltAcquirerDown(1));
            addSequential(new DriveStraightDistanceTank(TANK_POWER, 300));
            
            addParallel(new LiftToHeight(SCALE_HIGH));
            addSequential(new TurnToHeading(-90));
            
            addSequential(new TiltAcquirerUp(0.75));
            addSequential(new DriveStraightDistanceTank(0.4, 10), 1);
            addSequential(new EjectUntilCubeOut(0.5, 1));
        }
    }
    
    public class RightToLeftScaleDriveStraight extends CommandGroup {
    	public RightToLeftScaleDriveStraight() {
    		addParallel(new TiltAcquirerDown(1));
    		addSequential(new DriveStraightDistanceTank(TANK_POWER, 236 - HALF_LENGTH_AND_BUMPER));
    		
    		addSequential(new TurnToHeading(-90));
    		
    		addParallel(new LiftToHeightAndHold(SCALE_LOW));
    		addSequential(new DriveStraightDistanceTank(TANK_POWER, 206));
    		
    		addSequential(new TurnToHeading(0));
    		addSequential(new DriveStraightDistanceTank(0.6, 17));
    		addSequential(new EjectUntilCubeOut(0.5, 1));
    	}
    }
    
    /*
     * Two-cube routines: currently just scale, then switch, both on the same side
     */
    public class LeftToLeftTwoCube extends CommandGroup {
    	public LeftToLeftTwoCube() {
    		addParallel(new TiltAcquirerDown(1));
    		addParallel(new LiftToHeightAndHold(SCALE_LOW)); // lift to low height to prevent tipping
    		addSequential(new DriveStraightDistanceTank(TANK_POWER, 236 - HALF_LENGTH_AND_BUMPER));
    		
    		addParallel(new LiftToHeightAndHold(SCALE_MEDIUM)); // lift to medium scale height
    		addSequential(new TurnToHeading(25));
    		
    		addSequential(new DriveStraightDistanceTank(TANK_POWER, 26));
    		addSequential(new EjectUntilCubeOut(0.5, 1));
    		
    		addParallel(new DelayThenLift(0.75, STOWED));
    		addSequential(new TurnAndDriveToCube(150)); // estimated heading to get cube into pixy frame
    		
    		addSequential(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new EjectUntilCubeOut(0.5, 1));
    	}
    }
    
    public class RightToRightTwoCube extends CommandGroup {
    	public RightToRightTwoCube() {
    		addParallel(new TiltAcquirerDown(1));
    		addParallel(new LiftToHeightAndHold(SCALE_LOW)); // lift to low height to prevent tipping
    		addSequential(new DriveStraightDistanceTank(TANK_POWER, 236 - HALF_LENGTH_AND_BUMPER));
    		
    		addParallel(new LiftToHeightAndHold(SCALE_MEDIUM)); // lift to medium scale height
    		addSequential(new TurnToHeading(-25));
    		
    		addSequential(new DriveStraightDistanceTank(TANK_POWER, 26));
    		addSequential(new EjectUntilCubeOut(0.5, 1));

    		addParallel(new DelayThenLift(0.75, STOWED));
    		addSequential(new TurnAndDriveToCube(150)); // estimated heading to get cube into pixy frame
    		
    		addSequential(new LiftToHeightAndHold(SWITCH_LEVEL));
    		addSequential(new EjectUntilCubeOut(0.5, 1));
    	}
    }
    
    public class LeftToRightTwoCube extends CommandGroup {
    	// normal LeftToRightScale routine, then follow drivetocube example above
		public LeftToRightTwoCube() {
			addSequential(new LeftToRightScaleDriveStraight());

			addParallel(new DelayThenLift(0.75, STOWED));
			addSequential(new TurnAndDriveToCube(150)); // estimated heading to get cube into pixy frame

			addSequential(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new EjectUntilCubeOut(0.5, 1));
		}
    }
    
    public class RightToLeftTwoCube extends CommandGroup {
    	public RightToLeftTwoCube() {
    		addSequential(new RightToLeftScaleDriveStraight());

			addParallel(new DelayThenLift(0.75, STOWED));
			addSequential(new TurnAndDriveToCube(-150)); // estimated heading to get cube into pixy frame

			addSequential(new LiftToHeightAndHold(SWITCH_LEVEL));
			addSequential(new EjectUntilCubeOut(0.5, 1));
    	}
    }
}
