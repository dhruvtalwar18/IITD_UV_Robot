package org.ros.math;

import sensor_msgs.NavSatStatus;

public class Unsigned {
    private Unsigned() {
    }

    public static long intToLong(int value) {
        return ((long) value) & 4294967295L;
    }

    public static int shortToInt(short value) {
        return 65535 & value;
    }

    public static short byteToShort(byte value) {
        return (short) (value & NavSatStatus.STATUS_NO_FIX);
    }
}
