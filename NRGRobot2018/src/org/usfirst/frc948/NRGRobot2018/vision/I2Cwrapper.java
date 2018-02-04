package org.usfirst.frc948.NRGRobot2018.vision;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2Cwrapper implements IPixyLink{
	private I2C i2c;
	private Port i2cPort;
	private int deviceAddress;
	public I2Cwrapper(Port port, int deviceAddress) {
		i2cPort = port;
		this.deviceAddress = deviceAddress;
		i2c = new I2C(i2cPort, this.deviceAddress);
	}
	public int getWord() {
		byte[] c = new byte[2];
		i2c.readOnly(c, 2);
		int w = ((c[1] & 0xff) << 8) + (c[0] & 0xff);
		return w;
	}

	public byte getByte() {
		byte[] c = new byte[1];
		i2c.readOnly(c, 1);
		return c[0];
	}

	public void send(byte[] data) {
		i2c.writeBulk(data);
	}
	
}
