package org.usfirst.frc948.NRGRobot2018.vision;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class SPIwrapper implements IPixyLink {
	SPI spi;
	Port spiPort;

	public SPIwrapper(Port port) {
		spiPort = port;
		spi = new SPI(spiPort);
		
		spi.setMSBFirst();
		spi.setClockActiveHigh();
		spi.setSampleDataOnRising();
		spi.setChipSelectActiveLow();
	}

	public int getWord() {
		int w;
		byte[] c = new byte[2];
		spi.read(true, c, 2);
		w = (int) (((c[0] & 0xff) << 8) + (c[1] & 0xff));
		return w;
	}

	public byte getByte() {
		byte[] c = new byte[1];
		spi.read(true, c, 1);
		return c[0];
	}

	public void send(byte[] data) {
		spi.write(data, data.length);
	}

}
