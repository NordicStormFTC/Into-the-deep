package org.firstinspires.ftc.teamcode.nordicStorm.pixy;

import androidx.annotation.NonNull;

public final class PixyUtils {

    public static PixyBlock bytesToBlock(@NonNull byte[] bytes) {
        PixyBlock pixyBlock = new PixyBlock();
        pixyBlock.sync = PixyUtils.bytesToInt(bytes[1], bytes[0]);
        pixyBlock.checksum = PixyUtils.bytesToInt(bytes[3], bytes[2]);
        pixyBlock.signature = PixyUtils.orBytes(bytes[5], bytes[4]);
        pixyBlock.centerX = ((((int) bytes[7] & 0xff) << 8) | ((int) bytes[6] & 0xff));
        pixyBlock.centerY = ((((int) bytes[9] & 0xff) << 8) | ((int) bytes[8] & 0xff));
        pixyBlock.width = ((((int) bytes[11] & 0xff) << 8) | ((int) bytes[10] & 0xff));
        pixyBlock.height = ((((int) bytes[13] & 0xff) << 8) | ((int) bytes[12] & 0xff));
        return pixyBlock;
    }

    public static int bytesToInt(int b1, int b2) {
        if (b1 < 0)
            b1 += 256;

        if (b2 < 0)
            b2 += 256;

        int intValue = b1 * 256;
        intValue += b2;
        return intValue;
    }

    public static int orBytes(byte b1, byte b2) {
        return (b1 & 0xff) | (b2 & 0xff);
    }

}
