package org.usfirst.frc948.NRGRobot2018.vision;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2Cwrapper implements IPixyLink{
	I2C i2c;
	Port i2cPort;
	int deviceAddress;
	public I2Cwrapper(Port port, int deviceAddress) {
		i2cPort = port;
		this.deviceAddress = deviceAddress;
		i2c = new I2C(i2cPort, this.deviceAddress);
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
