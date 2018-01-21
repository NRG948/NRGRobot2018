package org.usfirst.frc948.NRGRobot2018.vision;


public class PixyCam {
	public static final int PIXY_INITIAL_ARRAYSIZE =  30;
	public static final int PIXY_MAXIMUM_ARRAYSIZE = 130;
	public static final int PIXY_START_WORD  = 0xaa55;
	public static final int PIXY_START_WORD_CC  = 0xaa56;
	public static final int PIXY_START_WORDX  = 0x55aa;
	public static final int PIXY_MAX_SIGNATURE  = 7;
	public static final int PIXY_DEFAULT_ARGVAL =  0xffff;

	// Pixy x-y position values
	public static final long PIXY_MIN_X = 0L;
	public static final long PIXY_MAX_X = 319L;
	public static final long PIXY_MIN_Y = 0L;
	public static final long PIXY_MAX_Y = 199L;

	// RC-servo values
	public static final long PIXY_RCS_MIN_POS = 0L;
	public static final long PIXY_RCS_MAX_POS = 1000L;
	public static final long PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2);
	
	enum BlockType
	{
	  NORMAL_BLOCK,
	  CC_BLOCK
	};
	
	class Block {
		public void print() {
			int i, j;
			char[] buf = new char[128];
			char[] sig = new char[6];
			int d;
			boolean flag;
			if(signature > PIXY_MAX_SIGNATURE) {
				for(i=12, j=0, flag=false; i >= 0; i-=3) {
					d = (signature>>>i)&0x07;
					if(d>0 && !flag) {
						flag = true;
					}
					if(flag) {
						sig[j++] = (char) (d + 0);
					}
				}
				sig[j++] = '\0';
				//not sure how to convert this line:
				//sprintf(buf, "CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d\n", sig, signature, x, y, width, height, angle);
			} else {
				//not sure about these two either:
				//sprintf(buf, "sig: %d x: %d y: %d width: %d height: %d\n", signature, x, y, width, height);   
			    //printf(buf);
			}
		}
		int signature, x, y, width, height, angle;
		
	}
	
}
