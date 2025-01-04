package org.firstinspires.ftc.teamcode.nordicStorm.pixy;

public class PixyBlock {
    // 0, 1 0 sync (0xaa55)
    // 2, 3 1 checksum (sum of all 16-bit words 2-6)
    // 4, 5 2 signature number
    // 6, 7 3 x center of object
    // 8, 9 4 y center of object
    // 10, 11 5 width of object
    // 12, 13 6 height of object

    // read byte : 85 read byte : -86
    // read byte : 85 read byte : -86
    // read byte : 22 read byte : 1
    // read by
    // read byte : -128 read byte : 0
    // read byte : 118 read byte : 0
    // read byte : 22 read byte : 0

    public int sync;
    public int checksum;
    public int signature;
    public int centerX;
    public int centerY;
    public int width;
    public int height;
    public int angle;
    public int trackingIndex;
    public int age;
}