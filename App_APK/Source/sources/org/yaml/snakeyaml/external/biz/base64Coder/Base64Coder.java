package org.yaml.snakeyaml.external.biz.base64Coder;

import org.apache.commons.io.IOUtils;

public class Base64Coder {
    private static char[] map1 = new char[64];
    private static byte[] map2 = new byte[128];
    private static final String systemLineSeparator = System.getProperty("line.separator");

    static {
        int i = 0;
        char c = 'A';
        while (c <= 'Z') {
            map1[i] = c;
            c = (char) (c + 1);
            i++;
        }
        char c2 = 'a';
        while (c2 <= 'z') {
            map1[i] = c2;
            c2 = (char) (c2 + 1);
            i++;
        }
        char c3 = '0';
        while (c3 <= '9') {
            map1[i] = c3;
            c3 = (char) (c3 + 1);
            i++;
        }
        int i2 = i + 1;
        map1[i] = '+';
        int i3 = i2 + 1;
        map1[i2] = IOUtils.DIR_SEPARATOR_UNIX;
        for (int i4 = 0; i4 < map2.length; i4++) {
            map2[i4] = -1;
        }
        for (int i5 = 0; i5 < 64; i5++) {
            map2[map1[i5]] = (byte) i5;
        }
    }

    public static String encodeString(String s) {
        return new String(encode(s.getBytes()));
    }

    public static String encodeLines(byte[] in) {
        return encodeLines(in, 0, in.length, 76, systemLineSeparator);
    }

    public static String encodeLines(byte[] in, int iOff, int iLen, int lineLen, String lineSeparator) {
        int blockLen = (lineLen * 3) / 4;
        if (blockLen > 0) {
            StringBuilder buf = new StringBuilder((((iLen + 2) / 3) * 4) + (lineSeparator.length() * (((iLen + blockLen) - 1) / blockLen)));
            int ip = 0;
            while (ip < iLen) {
                int l = Math.min(iLen - ip, blockLen);
                buf.append(encode(in, iOff + ip, l));
                buf.append(lineSeparator);
                ip += l;
            }
            return buf.toString();
        }
        throw new IllegalArgumentException();
    }

    public static char[] encode(byte[] in) {
        return encode(in, 0, in.length);
    }

    public static char[] encode(byte[] in, int iLen) {
        return encode(in, 0, iLen);
    }

    public static char[] encode(byte[] in, int iOff, int iLen) {
        int ip;
        int ip2;
        int ip3;
        int ip4;
        int oDataLen = ((iLen * 4) + 2) / 3;
        char[] out = new char[(((iLen + 2) / 3) * 4)];
        int iEnd = iOff + iLen;
        int ip5 = iOff;
        int op = 0;
        while (ip5 < iEnd) {
            int ip6 = ip5 + 1;
            int i0 = in[ip5] & 255;
            if (ip6 < iEnd) {
                ip = ip6 + 1;
                ip2 = in[ip6] & 255;
            } else {
                ip = ip6;
                ip2 = 0;
            }
            if (ip < iEnd) {
                ip3 = ip + 1;
                ip4 = in[ip] & 255;
            } else {
                ip3 = ip;
                ip4 = 0;
            }
            int o2 = ((ip2 & 15) << 2) | (ip4 >>> 6);
            int o3 = ip4 & 63;
            int op2 = op + 1;
            out[op] = map1[i0 >>> 2];
            int op3 = op2 + 1;
            out[op2] = map1[((i0 & 3) << 4) | (ip2 >>> 4)];
            char c = '=';
            out[op3] = op3 < oDataLen ? map1[o2] : '=';
            int op4 = op3 + 1;
            if (op4 < oDataLen) {
                c = map1[o3];
            }
            out[op4] = c;
            op = op4 + 1;
            ip5 = ip3;
        }
        return out;
    }

    public static String decodeString(String s) {
        return new String(decode(s));
    }

    public static byte[] decodeLines(String s) {
        char[] buf = new char[s.length()];
        int p = 0;
        for (int ip = 0; ip < s.length(); ip++) {
            char c = s.charAt(ip);
            if (!(c == ' ' || c == 13 || c == 10 || c == 9)) {
                buf[p] = c;
                p++;
            }
        }
        return decode(buf, 0, p);
    }

    public static byte[] decode(String s) {
        return decode(s.toCharArray());
    }

    public static byte[] decode(char[] in) {
        return decode(in, 0, in.length);
    }

    /* JADX WARNING: Incorrect type for immutable var: ssa=char, code=int, for r9v3, types: [char] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static byte[] decode(char[] r22, int r23, int r24) {
        /*
            int r3 = r24 % 4
            if (r3 != 0) goto L_0x00bc
            r2 = r24
        L_0x0006:
            if (r2 <= 0) goto L_0x0015
            int r3 = r23 + r2
            int r3 = r3 + -1
            char r3 = r22[r3]
            r4 = 61
            if (r3 != r4) goto L_0x0015
            int r2 = r2 + -1
            goto L_0x0006
        L_0x0015:
            int r3 = r2 * 3
            int r3 = r3 / 4
            byte[] r4 = new byte[r3]
            r5 = r23
            int r6 = r23 + r2
            r7 = 0
        L_0x0020:
            if (r5 >= r6) goto L_0x00b7
            int r8 = r5 + 1
            char r5 = r22[r5]
            int r9 = r8 + 1
            char r8 = r22[r8]
            r10 = 65
            if (r9 >= r6) goto L_0x0033
            int r11 = r9 + 1
            char r9 = r22[r9]
            goto L_0x0036
        L_0x0033:
            r11 = r9
            r9 = 65
        L_0x0036:
            if (r11 >= r6) goto L_0x0042
            int r10 = r11 + 1
            char r11 = r22[r11]
            r21 = r11
            r11 = r10
            r10 = r21
        L_0x0042:
            r12 = 127(0x7f, float:1.78E-43)
            if (r5 > r12) goto L_0x00a9
            if (r8 > r12) goto L_0x00a9
            if (r9 > r12) goto L_0x00a9
            if (r10 > r12) goto L_0x00a9
            byte[] r12 = map2
            byte r12 = r12[r5]
            byte[] r13 = map2
            byte r13 = r13[r8]
            byte[] r14 = map2
            byte r14 = r14[r9]
            byte[] r15 = map2
            byte r15 = r15[r10]
            if (r12 < 0) goto L_0x009b
            if (r13 < 0) goto L_0x009b
            if (r14 < 0) goto L_0x009b
            if (r15 < 0) goto L_0x009b
            int r16 = r12 << 2
            int r17 = r13 >>> 4
            r0 = r16 | r17
            r16 = r13 & 15
            int r16 = r16 << 4
            int r17 = r14 >>> 2
            r1 = r16 | r17
            r16 = r14 & 3
            int r16 = r16 << 6
            r18 = r2
            r2 = r16 | r15
            r19 = r5
            int r5 = r7 + 1
            r20 = r6
            byte r6 = (byte) r0
            r4[r7] = r6
            if (r5 >= r3) goto L_0x008b
            int r6 = r5 + 1
            byte r7 = (byte) r1
            r4[r5] = r7
            r5 = r6
        L_0x008b:
            if (r5 >= r3) goto L_0x0094
            int r6 = r5 + 1
            byte r7 = (byte) r2
            r4[r5] = r7
            r7 = r6
            goto L_0x0095
        L_0x0094:
            r7 = r5
        L_0x0095:
            r5 = r11
            r2 = r18
            r6 = r20
            goto L_0x0020
        L_0x009b:
            r18 = r2
            r19 = r5
            r20 = r6
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException
            java.lang.String r1 = "Illegal character in Base64 encoded data."
            r0.<init>(r1)
            throw r0
        L_0x00a9:
            r18 = r2
            r19 = r5
            r20 = r6
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException
            java.lang.String r1 = "Illegal character in Base64 encoded data."
            r0.<init>(r1)
            throw r0
        L_0x00b7:
            r18 = r2
            r20 = r6
            return r4
        L_0x00bc:
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException
            java.lang.String r1 = "Length of Base64 encoded input string is not a multiple of 4."
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.yaml.snakeyaml.external.biz.base64Coder.Base64Coder.decode(char[], int, int):byte[]");
    }

    private Base64Coder() {
    }
}
