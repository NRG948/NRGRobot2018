package org.usfirst.frc948.NRGRobot2018.vision;

import static java.util.Objects.requireNonNull;

import java.util.*;

import edu.wpi.first.wpilibj.PIDController;

public class PixyCam {
	public static final int PIXY_INITIAL_ARRAYSIZE = 30;
	public static final int PIXY_MAXIMUM_ARRAYSIZE = 130;
	public static final int PIXY_START_WORD = 0xaa55;
	public static final int PIXY_START_WORD_CC = 0xaa56;
	public static final int PIXY_START_WORDX = 0x55aa;
	public static final int PIXY_MAX_SIGNATURE = 7;
	public static final int PIXY_DEFAULT_ARGVAL = 0xffff;
	// Block distance
	public static final double KNOWN_AREA = (90 - 10) * (75 - 10);// SUBTRACTING
																	// 10 PIXELS
																	// BECAUSE
																	// OF NOISE
	public static final double KNOWN_DISTANCE_INCHES = 36;
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
	volatile ArrayList<Block> blocks = new ArrayList<Block>(10);

	public PixyCam(IPixyLink link) {
		this.link = link;
	}

	public void startVision() { // start vision thread
		new Timer().schedule(new VisionTask(this), 0, 10);
	}

	public boolean getStart() {
		int w, lastw;
		lastw = 0xffff;

		while (true) {
			w = link.getWord();

			if (w == PIXY_START_WORD && lastw == PIXY_START_WORD) {
				System.out.println("getStart(): New frame, Normal Block");
				blockType = BlockType.NORMAL_BLOCK;
				return true;
			} else if (w == PIXY_START_WORD_CC && lastw == PIXY_START_WORD) {
				System.out.println("\ngetStart(): New frame, CC Block");
				blockType = BlockType.CC_BLOCK;
				return true;
			} else if (w == PIXY_START_WORDX) {
				System.out.println("getStart(): resyncing");
				link.getByte(); // resync
			} else if (w == 0 && lastw == 0) {
				try {
					Thread.sleep(50L);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				return false;
			}
			
			lastw = w;
		}
	}

	public int getBlocksLoop() {
		ArrayList<Block> blocks = new ArrayList<Block>(PIXY_MAXIMUM_ARRAYSIZE);

		if (!skipStart) {
			if (!getStart()) {
				setBlocks(blocks);
				return 0;
			}
		} else {
			skipStart = false;
		}

		while (blocks.size() < PIXY_MAXIMUM_ARRAYSIZE) {
			int checksum = link.getWord();

			if (checksum == PIXY_START_WORD) {
				System.out.println("getBlocksLoop(): New frame, normal block");
				skipStart = true;
				blockType = BlockType.NORMAL_BLOCK;
				break;
			} else if (checksum == PIXY_START_WORD_CC) {
				System.out.println("getBlocksLoop(): New frame, color code block");
				skipStart = true;
				blockType = BlockType.CC_BLOCK;
				break;
			} else if (checksum == 0) {
				break;
			}
			
			Block block = new Block(link, blockType);
			if (block.getChecksum() == checksum) {
				System.out.print("getBlocksLoop(): Checksums equal, added " + block);
				double distance = Math.sqrt(KNOWN_AREA / ((block.width - 10) * (block.height - 10)))
						* KNOWN_DISTANCE_INCHES;
				System.out.println(", distance in inches = " + distance);
				blocks.add(block);
			} else {
				System.out.println("getBlocksLoop(): Checksums not equal: " + block.getChecksum() + " != " + checksum);
			}

			int w = link.getWord();

			if (w == PIXY_START_WORD) {
				// System.out.println("getBlocksLoop(): got normal sync word");
				blockType = BlockType.NORMAL_BLOCK;
			} else if (w == PIXY_START_WORD_CC) {
				System.out.println("getBlocksLoop(): got CC sync word");
				blockType = BlockType.CC_BLOCK;
			} else {
				if (w != 0) { // Zero just means there's no more data in receive buffer
					System.out.println("getBlocksLooop(): unexpected data, w = " + Integer.toHexString(w));
				}
				break;
			}
		}
		setBlocks(blocks);
		System.out.println("getBlocksLoop(): blocks added to list, size: " + blocks.size() + "\n");
		return blocks.size();
	}

	public synchronized ArrayList<Block> getBlocks() {
		return blocks;
	}

	private synchronized void setBlocks(ArrayList<Block> blocks) {
		this.blocks = blocks;
	}

	void setBrightness(int brightness) {
		byte[] outBuf = new byte[3];

		outBuf[0] = (byte) 0x00;
		outBuf[1] = (byte) 0xfe;
		outBuf[2] = (byte) brightness;

		link.send(outBuf);
	}

	enum BlockType {
		NORMAL_BLOCK, CC_BLOCK
	}

	public class Block {
		public int signature, x, y, width, height, angle;
		private int checksum;

		public Block(IPixyLink link, BlockType blockType) {
			signature = link.getWord();
			x = link.getWord();
			y = link.getWord();
			width = link.getWord();
			height = link.getWord();
			angle = (blockType == BlockType.CC_BLOCK) ? link.getWord() : 0; // pixycam doesn't send
																			// angle value in normal
																			// blocks, only CC
																			// blocks

			checksum = signature + x + y + width + height + angle;
		}

		public int getChecksum() {
			return checksum;
		}

		public String toString() {
			return String.format("block: sig=%04X x=%d y=%d width=%d height=%d angle=%d", signature, x, y, width,
					height, angle);
		}
	}

	private class VisionTask extends TimerTask {
		private PixyCam pixycam;

		public VisionTask(PixyCam pixycam) {
			this.pixycam = pixycam;
		}

		@Override
		public void run() {
			pixycam.getBlocksLoop();
		}
	}
}
