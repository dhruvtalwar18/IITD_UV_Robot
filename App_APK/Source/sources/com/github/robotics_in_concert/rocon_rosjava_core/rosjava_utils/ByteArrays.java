package com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils;

import com.google.common.base.Ascii;
import java.security.InvalidParameterException;
import sensor_msgs.NavSatStatus;

public class ByteArrays {
    private static final byte[] HEX_CHAR_TABLE = {48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 65, 66, 67, 68, 69, 70};

    public static String getHexString(byte[] raw, int len) {
        byte[] hex = new byte[(len * 3)];
        int index = 0;
        int pos = 0;
        byte[] arr$ = raw;
        int len$ = arr$.length;
        int i$ = 0;
        while (i$ < len$) {
            byte b = arr$[i$];
            if (pos >= len) {
                break;
            }
            pos++;
            int v = b & 255;
            int index2 = index + 1;
            hex[index] = HEX_CHAR_TABLE[v >>> 4];
            int index3 = index2 + 1;
            hex[index2] = HEX_CHAR_TABLE[v & 15];
            hex[index3] = 32;
            i$++;
            index = index3 + 1;
        }
        return new String(hex);
    }

    public static byte[] concat(byte[]... arrays) {
        int length = 0;
        for (byte[] array : arrays) {
            length += array.length;
        }
        byte[] result = new byte[length];
        int pos = 0;
        for (byte[] array2 : arrays) {
            System.arraycopy(array2, 0, result, pos, array2.length);
            pos += array2.length;
        }
        return result;
    }

    public static String toString(byte[] input, int offset, int count) {
        if (offset + count <= input.length) {
            byte[] result = new byte[count];
            for (int i = 0; i < count; i++) {
                result[i] = input[offset + i];
            }
            return new String(result);
        }
        throw new ArrayIndexOutOfBoundsException("Requested chunk exceeds byte array limits");
    }

    public static short toShort(byte[] input, int offset) {
        if (offset + 2 <= input.length) {
            return (short) ((input[offset + 1] & NavSatStatus.STATUS_NO_FIX) | ((input[offset + 0] & NavSatStatus.STATUS_NO_FIX) << 8));
        }
        throw new ArrayIndexOutOfBoundsException("Requested chunk exceeds byte array limits");
    }

    public static int toInteger(byte[] input, int offset) {
        if (offset + 4 <= input.length) {
            return (input[offset + 3] & NavSatStatus.STATUS_NO_FIX) | ((input[offset + 2] & NavSatStatus.STATUS_NO_FIX) << 8) | ((input[offset + 1] & NavSatStatus.STATUS_NO_FIX) << 16) | ((input[offset + 0] & NavSatStatus.STATUS_NO_FIX) << Ascii.CAN);
        }
        throw new ArrayIndexOutOfBoundsException("Requested chunk exceeds byte array limits");
    }

    public static byte[] toFixSizeBytes(String input, int length, byte padding) {
        if (input.length() <= length) {
            byte[] result = new byte[length];
            byte[] source = input.getBytes();
            int i = 0;
            while (i < length) {
                result[i] = i < source.length ? source[i] : padding;
                i++;
            }
            return result;
        }
        throw new InvalidParameterException(length + "exceeds limit in " + (input.length() - length) + " chars");
    }

    public static byte[] toBytes(int input) {
        return new byte[]{(byte) (input >> 24), (byte) (input >> 16), (byte) (input >> 8), (byte) input};
    }

    public static byte[] toBytes(short input) {
        return new byte[]{(byte) (input >> 8), (byte) input};
    }
}
