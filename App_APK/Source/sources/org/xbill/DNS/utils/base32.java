package org.xbill.DNS.utils;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import sensor_msgs.NavSatStatus;

public class base32 {
    private String alphabet;
    private boolean lowercase;
    private boolean padding;

    public static class Alphabet {
        public static final String BASE32 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567=";
        public static final String BASE32HEX = "0123456789ABCDEFGHIJKLMNOPQRSTUV=";

        private Alphabet() {
        }
    }

    public base32(String alphabet2, boolean padding2, boolean lowercase2) {
        this.alphabet = alphabet2;
        this.padding = padding2;
        this.lowercase = lowercase2;
    }

    private static int blockLenToPadding(int blocklen) {
        switch (blocklen) {
            case 1:
                return 6;
            case 2:
                return 4;
            case 3:
                return 3;
            case 4:
                return 1;
            case 5:
                return 0;
            default:
                return -1;
        }
    }

    private static int paddingToBlockLen(int padlen) {
        switch (padlen) {
            case 0:
                return 5;
            case 1:
                return 4;
            case 3:
                return 3;
            case 4:
                return 2;
            case 6:
                return 1;
            default:
                return -1;
        }
    }

    public String toString(byte[] b) {
        byte[] bArr = b;
        ByteArrayOutputStream os = new ByteArrayOutputStream();
        for (int i = 0; i < (bArr.length + 4) / 5; i++) {
            short[] s = new short[5];
            int[] t = new int[8];
            int blocklen = 5;
            for (int j = 0; j < 5; j++) {
                if ((i * 5) + j < bArr.length) {
                    s[j] = (short) (bArr[(i * 5) + j] & NavSatStatus.STATUS_NO_FIX);
                } else {
                    s[j] = 0;
                    blocklen--;
                }
            }
            int padlen = blockLenToPadding(blocklen);
            t[0] = (byte) ((s[0] >> 3) & 31);
            t[1] = (byte) (((s[0] & 7) << 2) | ((s[1] >> 6) & 3));
            t[2] = (byte) ((s[1] >> 1) & 31);
            t[3] = (byte) (((s[1] & 1) << 4) | ((s[2] >> 4) & 15));
            t[4] = (byte) (((s[2] & 15) << 1) | ((s[3] >> 7) & 1));
            t[5] = (byte) ((s[3] >> 2) & 31);
            t[6] = (byte) (((s[4] >> 5) & 7) | ((s[3] & 3) << 3));
            t[7] = (byte) (s[4] & 31);
            for (int j2 = 0; j2 < t.length - padlen; j2++) {
                char c = this.alphabet.charAt(t[j2]);
                if (this.lowercase) {
                    c = Character.toLowerCase(c);
                }
                os.write(c);
            }
            if (this.padding != 0) {
                for (int j3 = t.length - padlen; j3 < t.length; j3++) {
                    os.write(61);
                }
            }
        }
        return new String(os.toByteArray());
    }

    public byte[] fromString(String str) {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        byte[] raw = str.getBytes();
        for (byte b : raw) {
            char c = (char) b;
            if (!Character.isWhitespace(c)) {
                bs.write((byte) Character.toUpperCase(c));
            }
        }
        char c2 = '=';
        int i = 8;
        if (this.padding == 0) {
            while (bs.size() % 8 != 0) {
                bs.write(61);
            }
        } else if (bs.size() % 8 != 0) {
            return null;
        }
        byte[] in = bs.toByteArray();
        bs.reset();
        DataOutputStream ds = new DataOutputStream(bs);
        int i2 = 0;
        while (true) {
            int i3 = i2;
            if (i3 >= in.length / i) {
                return bs.toByteArray();
            }
            short[] s = new short[i];
            int[] t = new int[5];
            int padlen = 8;
            int j = 0;
            while (j < i && ((char) in[(i3 * 8) + j]) != c2) {
                s[j] = (short) this.alphabet.indexOf(in[(i3 * 8) + j]);
                if (s[j] < 0) {
                    return null;
                }
                padlen--;
                j++;
                c2 = '=';
                i = 8;
            }
            int blocklen = paddingToBlockLen(padlen);
            if (blocklen < 0) {
                return null;
            }
            t[0] = (s[0] << 3) | (s[1] >> 2);
            t[1] = ((s[1] & 3) << 6) | (s[2] << 1) | (s[3] >> 4);
            t[2] = ((s[3] & 15) << 4) | ((s[4] >> 1) & 15);
            t[3] = (s[4] << 7) | (s[5] << 2) | (s[6] >> 3);
            t[4] = ((s[6] & 7) << 5) | s[7];
            int j2 = 0;
            while (j2 < blocklen) {
                try {
                    ds.writeByte((byte) (t[j2] & 255));
                    j2++;
                } catch (IOException e) {
                }
            }
            i2 = i3 + 1;
            c2 = '=';
            i = 8;
        }
    }
}