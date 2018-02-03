package org.usfirst.frc948.NRGRobot2018.vision;

import java.util.*;

public class PixyCam {
	public static final int PIXY_INITIAL_ARRAYSIZE = 30;
	public static final int PIXY_MAXIMUM_ARRAYSIZE = 130;
	public static final int PIXY_START_WORD = 0xaa55;
	public static final int PIXY_START_WORD_CC = 0xaa56;
	public static final int PIXY_START_WORDX = 0x55aa;
	public static final int PIXY_MAX_SIGNATURE = 7;
	public static final int PIXY_DEFAULT_ARGVAL = 0xffff;

	// Pixy x-y position values
	public static final long PIXY_MIN_X = 0L;
	public static final long PIXY_MAX_X = 319L;
	public static final long PIXY_MIN_Y = 0L;
	public static final long PIXY_MAX_Y = 199L;

	// RC-servo values
	public static final long PIXY_RCS_MIN_POS = 0L;
	public static final long PIXY_RCS_MAX_POS = 1000L;
	public static final long PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2);

	private IPixyLink link;
	private boolean skipStart = false;
	private BlockType blockType;
	private int blockCount;
	ArrayList<Block> blocks = new ArrayList<Block>(blockCount);

	public PixyCam(IPixyLink link) {
		this.link = link;
	}

	enum BlockType {
		NORMAL_BLOCK, CC_BLOCK
	};

	void setBrightness(int brightness) {
		byte[] outBuf = new byte[3];

		outBuf[0] = (byte) 0x00;
		outBuf[1] = (byte) 0xfe;
		outBuf[2] = (byte) brightness;

		link.send(outBuf);
	}

	public boolean getStart() {
		int w, lastw;
		lastw = (short) 0xffff;
		
		while (true) {
			w = link.getWord();
			
			if (w == 0 && lastw == 0) {
				try {
					Thread.sleep(50L);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				return false;
			} else if (w == PIXY_START_WORD && lastw == PIXY_START_WORD) {
				blockType = BlockType.NORMAL_BLOCK;
				return true;
			} else if (w == PIXY_START_WORD_CC && lastw == PIXY_START_WORD) {
				blockType = BlockType.CC_BLOCK;
				return true;
			} else if (w == PIXY_START_WORDX) {
				link.getByte(); // resync
			}
			lastw = w;
		}
	}

	/*
	 * void resize() { blockArraySize += PIXY_INITIAL_ARRAYSIZE; }
	 */
	// might not need this method, because Arraylist automatically resizes.

	public int getBlocksLoop() {
		ArrayList<Block> blocks = new ArrayList<Block>(PIXY_MAXIMUM_ARRAYSIZE);

		if (!skipStart) {
			if (!getStart()) {
				System.out.println("uh oh");
				setBlocks(blocks);
				return 0;
			}
		} else {
			skipStart = false;
		}

		while (blocks.size() < PIXY_MAXIMUM_ARRAYSIZE) {
			int checksum = link.getWord();
			if (checksum == PIXY_START_WORD) {
				skipStart = true;
				blockType = BlockType.NORMAL_BLOCK;
				break;
			} else if (checksum == PIXY_START_WORD_CC) {
				skipStart = true;
				blockType = BlockType.CC_BLOCK;
				break;
			} else if (checksum == 0) {
				break;
			}
			/*
			 * if (blockCount>blockArraySize) resize();
			 */
			// we might not need this if statement, because we have an arraylist now
			Block block = new Block(link);
			if (block.getChecksum() == checksum) {
				blocks.add(block);
			}
			int w = link.getWord();
			if (w == PIXY_START_WORD) {
				blockType = BlockType.NORMAL_BLOCK;
			} else if (w == PIXY_START_WORD_CC) {
				blockType = BlockType.CC_BLOCK;
			} else {
				break;
			}
		}
		setBlocks(blocks);
		return blocks.size();
	}

	private synchronized void setBlocks(ArrayList<Block> blocks) {
		this.blocks = blocks;
	}

	public synchronized ArrayList<Block> getBlocks() {
		return blocks;
	}

	public class Block {
		public int signature, x, y, width, height, angle;
		private int checksum;

		public Block(IPixyLink link) {
			signature = link.getWord();
			x = link.getWord();
			y = link.getWord();
			width = link.getWord();
			height = link.getWord();
			angle = link.getWord();

			checksum = signature + x + y + width + height + angle;
		}

		public int getChecksum() {
			return checksum;
		}

		public String toString() {
			int i, j;
			char[] buf = new char[128];
			char[] sig = new char[6];
			int d;
			boolean flag;
			String rep;
			/*
			 * if (signature > PIXY_MAX_SIGNATURE) { for (i = 12, j = 0, flag = false; i >=
			 * 0; i -= 3) { d = (signature >>> i) & 0x07; if (d > 0 && !flag) { flag = true;
			 * } if (flag) { sig[j++] = (char) (d + 0); } } sig[j++] = '\0';
			 */
			rep = "Block, " + "sig = " + signature + ", x=" + x + ", y=" + y + ", width=" + width + ", height=" + height
					+ ", angle=" + angle + " degrees";
			return rep;
			// not sure how to convert this line:
			// sprintf(buf, "CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d
			// angle %d\n", sig, signature, x, y, width, height, angle);
			/*
			 * } else { // not sure about these two either: // sprintf(buf,
			 * "sig: %d x: %d y: %d width: %d height: %d\n", signature, x, y, // width,
			 * height); // printf(buf);
			 */
			// rep = "Normal Block!, " + "sig = " + signature + ", x=" + x + ", y=" + y + ",
			// width=" + width + ", height=" + height + ", angle=" + angle + " degrees";
			// return rep;
		}

	}

}
