package org.usfirst.frc948.NRGRobot2018.utilities;

import java.util.ArrayList;

import org.usfirst.frc948.NRGRobot2018.vision.PixyCam.Block;

public class CubeCalculations {
	public static final double CENTER_PIXEL_LOCATION = 159.5;
	// Measured constants for converting camera coordinates (pixels) -> real-world coordinates
	// relative to robot (inches)
	public static final double KNOWN_DISTANCE_INCHES = 36;
	// subtracting 10 pixels to compensate for image noise
	public static final double NOISE_FUDGE_FACTOR = 10;
	public static final double KNOWN_AREA_PIXELS = (90 - NOISE_FUDGE_FACTOR) * (75 - NOISE_FUDGE_FACTOR);
	public static final double KNOWN_WIDTH_PIXELS = 87;
	public static final double KNOWN_FOCAL_LENGTH_PIXELS = 208.52; // (320 / 2) / tan(37.5 deg)

	public static double getDistanceFromArea(Block cube) {
		double currAreaPixels = (cube.width - NOISE_FUDGE_FACTOR) * (cube.height - NOISE_FUDGE_FACTOR);
		double currDistanceInches = KNOWN_DISTANCE_INCHES * Math.sqrt(KNOWN_AREA_PIXELS / currAreaPixels);
		return currDistanceInches;
	}
	
	public static double getDistanceFromWidth(Block cube) {
		double currDistanceInches = KNOWN_DISTANCE_INCHES * (KNOWN_WIDTH_PIXELS / cube.width);
		return currDistanceInches;
	}

	// assumes entire cube is in frame
	public static double getAngleToTurn(Block cube) {
		double angleToTurn = Math.toDegrees(Math.atan((cube.x - CENTER_PIXEL_LOCATION) / KNOWN_FOCAL_LENGTH_PIXELS));
		return angleToTurn;
	}
	
	// returns -1 to 1; uses center of cube
	public static double getDistanceToCenterNormalized(Block cube) {
		double distance = (cube.x - CENTER_PIXEL_LOCATION) / CENTER_PIXEL_LOCATION;
		return distance;
	}
}
