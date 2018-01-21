package org.usfirst.frc948.NRGRobot2018.vision;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class SPIwrapper implements IPixyLink{
	SPI spi;
	Port spiPort;
	public SPIwrapper(Port port) {
		spiPort = port;
		spi = new SPI(spiPort);
	}
	public short getWord() {
		// TODO Auto-generated method stub
		return 0;
	}

	public byte getByte() {
		// TODO Auto-generated method stub
		return 0;
	}

	public void send(byte[] data) {
		// TODO Auto-generated method stub
		
	}
	
}
