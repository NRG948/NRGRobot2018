package org.usfirst.frc948.NRGRobot2018.utilities;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc948.NRGRobot2018.Robot;
import org.usfirst.frc948.NRGRobot2018.vision.IPixyLink;

public class Arduino {
	private IPixyLink link;

	public Arduino(IPixyLink link) {
		this.link = link;
	}

	public void startArduinoThread() { // starts vision thread
		new Timer().schedule(new ArduinoTask(this), 0, 10);
	}

	void updateLights() {
		byte[] outBuf = new byte[1];
		boolean currentCubeSwitchState = Robot.cubeAcquirer.isCubeIn();
		
		if(currentCubeSwitchState != Robot.cubeAcquirer.lastCubeSwitchState) {
			// cube is fully acquired if switch state is true
			outBuf[0] = (byte) (currentCubeSwitchState ? 
					ArduinoLightConstants.CUBE_SENSOR_PRESSED : ArduinoLightConstants.CUBE_SENSOR_RELEASED);		
			Robot.cubeAcquirer.lastCubeSwitchState = currentCubeSwitchState;
		}
		
		link.send(outBuf);
	}

//	public boolean isVisionEnabled() {
//		return isEnabled;
//	}
//
//	public void enableVision() {
//		isEnabled = true;
//	}
//
//	public void disableVision() {
//		isEnabled = false;
//	}

	private class ArduinoTask extends TimerTask {
		private Arduino arduino;

		public ArduinoTask(Arduino pixyCam) {
			this.arduino = pixyCam;
		}

		@Override
		public void run() {
			arduino.updateLights();
		}
	}
}
