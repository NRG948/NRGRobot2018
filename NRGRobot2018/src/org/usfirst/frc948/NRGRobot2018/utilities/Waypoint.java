package org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.Robot;

public class Waypoint {
	public enum CoordinateType {
		ABSOLUTE, RELATIVE
	}

	public final CoordinateType coordinateType;
	public final double x, y, heading;
	public final WaypointPredicate waypointPredicate;

	public Waypoint(CoordinateType coordinateType, double x, double y, double heading,
			WaypointPredicate waypointPredicate) {
		this.coordinateType = coordinateType;
		this.x = x;
		this.y = y;
		this.heading = heading;
		this.waypointPredicate = waypointPredicate;
	}

	public WaypointPredicate getPredicate() {
		return waypointPredicate;
	}

	public Waypoint toAbsolute(double previousX, double previousY) {
		if (this.coordinateType == CoordinateType.RELATIVE) {
			return new Waypoint(CoordinateType.ABSOLUTE, previousX + this.x, previousY + this.y, this.heading,
					this.waypointPredicate);
		}
		return this;
	}

	public static DefaultPredicate USE_PID = new DefaultPredicate();

	public static class DefaultPredicate implements WaypointPredicate {
		@Override
		public boolean isFinished(Waypoint w) {
			return false;
		}
	}

	public static class GreaterThanY implements WaypointPredicate {
		double y;

		public GreaterThanY(double y) {
			this.y = y;
		}

		@Override
		public boolean isFinished(Waypoint w) {
			return Robot.positionTracker.getY() > y;
		}
	}

	public static class GreaterThanX implements WaypointPredicate {
		double x;

		public GreaterThanX(double x) {
			this.x = x;
		}

		@Override
		public boolean isFinished(Waypoint w) {
			return Robot.positionTracker.getX() > x;
		}
	}

	public static class LessThanX implements WaypointPredicate {
		double x;

		public LessThanX(double x) {
			this.x = x;
		}

		@Override
		public boolean isFinished(Waypoint w) {
			return Robot.positionTracker.getX() < x;
		}
	}

	public static class WithinInches implements WaypointPredicate {
		private double tolerance;

		public WithinInches(double tolerance) {
			this.tolerance = tolerance;
		}

		@Override
		public boolean isFinished(Waypoint w) {
			double dX = Robot.positionTracker.getX() - w.x;
			double dY = Robot.positionTracker.getY() - w.y;

			return dX * dX + dY * dY <= tolerance * tolerance;
		}
	}
}
