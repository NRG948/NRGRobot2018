package org.usfirst.frc948.NRGRobot2018.vision;

interface IPixyLink {
	short getWord();
	byte getByte();
	void send(byte[] data); //i2c data send method
}