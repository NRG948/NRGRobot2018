package org.usfirst.frc948.NRGRobot2018.utilities;

import org.usfirst.frc948.NRGRobot2018.Robot;

public class Waypoint {
    public interface Predicate {
        boolean isAtWaypoint();
    }

    public enum PredicateType {
        NONE, 
        GREATER_THAN_X, 
        GREATER_THAN_Y, 
        LESS_THAN_X,
        LESS_THAN_Y
    }

    public final double x, y, heading;
    public final PredicateType predicateType;

    public Waypoint(double x, double y, double heading, PredicateType predicateType) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.predicateType = predicateType;
    }

    public Predicate getPredicate() {
        switch (predicateType) {
        case NONE:
            return new Predicate() {
                @Override
                public boolean isAtWaypoint() {
                    return false;
                }
            };

        case GREATER_THAN_X:
            return new Predicate() {
                @Override
                public boolean isAtWaypoint() {
                    return Robot.positionTracker.getX() > x;
                }
            };

        case GREATER_THAN_Y:
            return new Predicate() {
                @Override
                public boolean isAtWaypoint() {
                    return Robot.positionTracker.getY() > y;
                }
            };

        case LESS_THAN_X:
            return new Predicate() {
                @Override
                public boolean isAtWaypoint() {
                    return Robot.positionTracker.getX() < x;
                }
            };

        case LESS_THAN_Y:
            return new Predicate() {
                @Override
                public boolean isAtWaypoint() {
                    return Robot.positionTracker.getY() < y;
                }
            };

        default:
            throw new IllegalStateException("Invalid PredicateType");

        }
    }
}
