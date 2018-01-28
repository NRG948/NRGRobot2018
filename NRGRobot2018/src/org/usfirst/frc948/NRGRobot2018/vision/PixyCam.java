package org.usfirst.frc948.NRGRobot2018.vision;
import java.util.*;

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
	
	private IPixyLink link;
	private boolean  skipStart;
	private BlockType blockType;
	private int blockCount;
	private int blockArraySize;
	ArrayList<Block> blocks = new ArrayList<Block>(blockCount);
	
	public PixyCam(IPixyLink link) {
		this.link = link;
	}
	
	enum BlockType {
	  NORMAL_BLOCK,
	  CC_BLOCK
	};
	
	
	void setBrightness(int brightness) {
		byte[] outBuf = new byte[3];
		
		outBuf[0] = (byte)0x00;
		outBuf[1] = (byte)0xfe;
		outBuf[2] = (byte)brightness;
		
		link.send(outBuf);
	}
	
	boolean getStart() {
		int w, lastw;
		lastw = (short)0xffff;
		while(true) {
			w = link.getWord();
			if(w==0 && lastw==0) {
				Timer delay = new Timer();
				delay.schedule(new TimerTask() {
					  @Override
					  public void run() {
					  }
					}, 50);
				return false;
			} else if(w==PIXY_START_WORD && lastw==PIXY_START_WORD) {
				blockType = BlockType.NORMAL_BLOCK;
				return true;
			} else if(w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD) {
				blockType = BlockType.CC_BLOCK;
				return true;
			} else if (w==PIXY_START_WORDX) {
			    link.getByte(); // resync
			}
			lastw = w;
		}
	}
	
	void resize() {
		blockArraySize += PIXY_INITIAL_ARRAYSIZE;
	}
	
	int getBlocks(int maxBlocks) {
		int i, w, checksum, sum;
		Block block;
		if (!skipStart) {
		   if (getStart()==false)
		     return 0;
		}
		else {
			skipStart = false;
		}
		
		blockCount = 0;
		while(blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE) {
			checksum = link.getWord();
		    if (checksum==PIXY_START_WORD) {
		    	skipStart = true;
		    	blockType = BlockType.NORMAL_BLOCK;
		    	return blockCount;
		    } else if(checksum==PIXY_START_WORD_CC) {
		    	skipStart = true;
		        blockType = BlockType.CC_BLOCK;
		        return blockCount;
		    } else if(checksum == 0) {
		    	return blockCount;
		    }
		    if (blockCount>blockArraySize)
		        resize();
		    block = blocks.get(blockCount); 
		    block.signature = link.getWord();
		    block.x = link.getWord();
		    block.y = link.getWord();
		    block.width = link.getWord();
		    block.height = link.getWord();
		    block.angle = link.getWord();
		    
		    if (checksum==sum) {
		    	blockCount++;
		    }
		    w = link.getWord();
		    if (w==PIXY_START_WORD) {
		    	blockType = BlockType.NORMAL_BLOCK;
		    } else if (w==PIXY_START_WORD_CC) {
		    	blockType = BlockType.CC_BLOCK;
		    } else {
		        return blockCount;
		    }
		}
	}
	
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
