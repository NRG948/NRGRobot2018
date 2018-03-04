package org.usfirst.frc948.NRGRobot2018.commandGroups;

import static org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition.RED_LEFT;
import static org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition.RED_RIGHT;

import java.util.ArrayList;
import java.util.Arrays;

import static org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition.RED_CENTER;
import static org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition.BLUE_LEFT;
import static org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition.BLUE_RIGHT;
import static org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition.BLUE_CENTER;



import org.usfirst.frc948.NRGRobot2018.OI;
import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.Robot.AutoMovement;
import org.usfirst.frc948.NRGRobot2018.Robot.AutoPosition;
import org.usfirst.frc948.NRGRobot2018.OI.Side;
import org.usfirst.frc948.NRGRobot2018.commands.DriveStraightDistance;
import org.usfirst.frc948.NRGRobot2018.commands.EjectUntilCubeOut;
import org.usfirst.frc948.NRGRobot2018.commands.LiftToHeight;
import org.usfirst.frc948.NRGRobot2018.commands.ResetSensors;
import org.usfirst.frc948.NRGRobot2018.commands.SetDriveScale;
import org.usfirst.frc948.NRGRobot2018.commands.TurnToHeading;
import org.usfirst.frc948.NRGRobot2018.subsystems.CubeLifter;
import org.usfirst.frc948.NRGRobot2018.subsystems.Drive;
import org.usfirst.frc948.NRGRobot2018.utilities.LifterLevel;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.CoordinateType;
import org.usfirst.frc948.NRGRobot2018.utilities.Waypoint.PredicateType;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Selects the auto routine to run based on the SmartDashboard chooser. 
 */
public class AutonomousRoutine extends CommandGroup {
	private AutoMovement autoMovement;
	private AutoPosition autoPosition;
	

    public AutonomousRoutine() {
    	addSequential(new ResetSensors());
    	autoMovement = Robot.autoMovementChooser.getSelected();
    	autoPosition = Robot.autoPositionChooser.getSelected();
    	
		System.out.println("Auto Movement is : " + autoMovement);
		System.out.println("Auto Position is : " + autoPosition);
        
		switch (autoMovement) {
		case LEFT_SWITCH:
		if (autoPosition == AutoPosition.RED_LEFT) {
			addSequential(new RedLeftToLeftSwitch());
		}	else if (autoPosition == BLUE_LEFT) {
			addSequential(new BlueLeftToLeftSwitch());
		} else if (autoPosition == RED_CENTER) {
			addSequential(new RedMiddleToLeftSwitch());
		} else if (autoPosition == BLUE_CENTER) {
			addSequential(new BlueMiddleToLeftSwitch());
		} else if (autoPosition == RED_RIGHT) {
			addSequential(new RedRightToLeftSwitch());
		} else if (autoPosition == BLUE_RIGHT) {
			addSequential(new BlueRightToLeftSwitch());
		}
		break;
		case RIGHT_SWITCH:
			if (autoPosition == RED_RIGHT) {
				addSequential(new RedRightToRightSwitch());
			} else if (autoPosition == BLUE_RIGHT) {
				addSequential(new BlueRightToRightSwitch());
			} else if (autoPosition == RED_CENTER) {
				addSequential(new RedMiddleToRightSwitch());
			} else if (autoPosition == BLUE_CENTER) {
				addSequential(new BlueMiddleToRightSwitch());
			} else if (autoPosition == RED_LEFT) {
				addSequential(new RedLeftToRightSwitch());
			} else if (autoPosition == BLUE_LEFT) {
				addSequential(new BlueLeftToRightSwitch());
			}
			break;
		case RIGHT_SCALE:
			if (autoPosition == RED_RIGHT) {
				addSequential(new RedRightToRightScale());
			} else if (autoPosition == BLUE_RIGHT) {
				addSequential(new BlueRightToRightScale());
			} else if (autoPosition == RED_CENTER) {
				addSequential(new RedMiddleToRightScale());
			} else if (autoPosition == BLUE_CENTER) {
				addSequential(new BlueMiddleToRightScale());
			}
			break;
		case LEFT_SCALE:
			if (autoPosition==AutoPosition.BLUE_LEFT) {
				addSequential(new BlueLeftToLeftScale());
			} else if (autoPosition == RED_CENTER) {
				addSequential(new RedMiddleToLeftScale());
			} else if (autoPosition == BLUE_CENTER) {
				addSequential(new BlueMiddleToLeftScale());
			} else if (autoPosition == RED_LEFT) {
				addSequential(new RedLeftToLeftScale());
			}
			break;
		}
		}
    
    public class RedLeftToLeftSwitch extends CommandGroup {
    	public RedLeftToLeftSwitch() {
    		addSequential(new SetDriveScale(1));
    		addSequential(new DriveStraightDistance(1.0,149.5,Drive.Direction.FORWARD));
    		addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,20.875,Drive.Direction.FORWARD));
			// drop the cube
		}
	}

	public class RedLeftToLeftScale extends CommandGroup {
		public RedLeftToLeftScale() {
			addSequential(new SetDriveScale(0.6));
			addSequential(new DriveStraightDistance(1.0,305.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,20.875,Drive.Direction.FORWARD));
		}
	}

	public class RedLeftToRightSwitch extends CommandGroup {
		public RedLeftToRightSwitch() {
			addSequential(new DriveStraightDistance(1.0,207,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,212.75,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,45,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,20.0,Drive.Direction.FORWARD));
		}
	}

	public class RedMiddleToRightSwitch extends CommandGroup {
		public RedMiddleToRightSwitch() {
			addSequential(new DriveStraightDistance(1.0,108.75,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,149.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
		}
	}

	public class RedMiddleToLeftSwitch extends CommandGroup {
		public RedMiddleToLeftSwitch() {
			addSequential(new DriveStraightDistance(1.0,84.75,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,148.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
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
		new Waypoint(CoordinateType.RELATIVE, 0.0, 120, 0, PredicateType.GREATER_THAN_Y),
		new Waypoint(CoordinateType.RELATIVE, -21, 29, -90, PredicateType.NONE)
		};
	
	public class RedRightToRightSwitch extends CommandGroup {
		public RedRightToRightSwitch() {
//			addSequential(new DriveStraightDistance(1.0,116.5,Drive.Direction.FORWARD));
//			addSequential(new TurnToHeading(-90));
//			addSequential(new DriveStraightDistance(1.0,20.875,Drive.Direction.FORWARD));
			addParallel(new FollowWaypoints(RRRS_PATH));
			addSequential(new LiftToHeight(CubeLifter.SWITCH_LEVEL));
			addSequential(new EjectUntilCubeOut(0.5, 1.0));
		}
	}

	public class RedRightToRightScale extends CommandGroup {
		public RedRightToRightScale() {
			addSequential(new DriveStraightDistance(1.0,305.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,7.68,Drive.Direction.FORWARD));
		}
	}

	public class RedRightToLeftSwitch extends CommandGroup {
		public RedRightToLeftSwitch() {
			addSequential(new DriveStraightDistance(1.0,207,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,212.7,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,45,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
		}
	}

	public class BlueLeftToLeftSwitch extends CommandGroup {
		public BlueLeftToLeftSwitch() {
			addSequential(new DriveStraightDistance(1.0,149.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20.875,Drive.Direction.FORWARD));
			// drop the cube
		}
	}

	public class BlueLeftToLeftScale extends CommandGroup {
		public BlueLeftToLeftScale() {
			addSequential(new DriveStraightDistance(1.0,305.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20.875,Drive.Direction.FORWARD));
		}
	}

	public class BlueLeftToRightSwitch extends CommandGroup {
		public BlueLeftToRightSwitch() {
			addSequential(new DriveStraightDistance(1.0,207,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,212.75,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,45,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
		}
	}

	public class BlueMiddleToRightSwitch extends CommandGroup {
		public BlueMiddleToRightSwitch() {
			addSequential(new DriveStraightDistance(1.0,108.75,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,149.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
		}
	}

	public class BlueMiddleToLeftSwitch extends CommandGroup {
		public BlueMiddleToLeftSwitch() {
			addSequential(new DriveStraightDistance(1.0,84.75,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,148.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
		}
	}

	public class BlueMiddleToRightScale extends CommandGroup {
		public BlueMiddleToRightScale() {

		}
	}

	public class BlueMiddleToLeftScale extends CommandGroup {
		public BlueMiddleToLeftScale() {

		}
	}

	public class BlueRightToRightSwitch extends CommandGroup {
		public BlueRightToRightSwitch() {
			addSequential(new DriveStraightDistance(1.0,149.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,20.875,Drive.Direction.FORWARD));
		}
	}

	public class BlueRightToRightScale extends CommandGroup {
		public BlueRightToRightScale() {
			addSequential(new DriveStraightDistance(1.0,305.5,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(90));
			addSequential(new DriveStraightDistance(1.0,7.68,Drive.Direction.FORWARD));
		}
	}

	public class BlueRightToLeftSwitch extends CommandGroup {
		public BlueRightToLeftSwitch() {
			addSequential(new DriveStraightDistance(1.0,207,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,212.7,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,45,Drive.Direction.FORWARD));
			addSequential(new TurnToHeading(-90));
			addSequential(new DriveStraightDistance(1.0,20,Drive.Direction.FORWARD));
		}
	}
    
		// Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }

