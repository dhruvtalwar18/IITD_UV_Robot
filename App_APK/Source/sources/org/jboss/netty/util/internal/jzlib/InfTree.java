package org.jboss.netty.util.internal.jzlib;

import android.support.v4.app.FragmentTransaction;
import android.support.v4.view.InputDeviceCompat;
import org.apache.commons.httpclient.HttpStatus;
import org.apache.commons.net.telnet.TelnetCommand;
import uk.co.blogspot.fractiousg.texample.GLText;

final class InfTree {
    static final int BMAX = 15;
    static final int[] cpdext = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
    static final int[] cpdist = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, InputDeviceCompat.SOURCE_GAMEPAD, 1537, 2049, 3073, FragmentTransaction.TRANSIT_FRAGMENT_OPEN, 6145, 8193, 12289, 16385, 24577};
    static final int[] cplens = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258, 0, 0};
    static final int[] cplext = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0, 112, 112};
    static final int fixed_bd = 5;
    static final int fixed_bl = 9;
    static final int[] fixed_td = {80, 5, 1, 87, 5, 257, 83, 5, 17, 91, 5, FragmentTransaction.TRANSIT_FRAGMENT_OPEN, 81, 5, 5, 89, 5, InputDeviceCompat.SOURCE_GAMEPAD, 85, 5, 65, 93, 5, 16385, 80, 5, 3, 88, 5, 513, 84, 5, 33, 92, 5, 8193, 82, 5, 9, 90, 5, 2049, 86, 5, 129, 192, 5, 24577, 80, 5, 2, 87, 5, 385, 83, 5, 25, 91, 5, 6145, 81, 5, 7, 89, 5, 1537, 85, 5, 97, 93, 5, 24577, 80, 5, 4, 88, 5, 769, 84, 5, 49, 92, 5, 12289, 82, 5, 13, 90, 5, 3073, 86, 5, 193, 192, 5, 24577};
    static final int[] fixed_tl = {96, 7, 256, 0, 8, 80, 0, 8, 16, 84, 8, 115, 82, 7, 31, 0, 8, 112, 0, 8, 48, 0, 9, 192, 80, 7, 10, 0, 8, 96, 0, 8, 32, 0, 9, 160, 0, 8, 0, 0, 8, 128, 0, 8, 64, 0, 9, 224, 80, 7, 6, 0, 8, 88, 0, 8, 24, 0, 9, 144, 83, 7, 59, 0, 8, 120, 0, 8, 56, 0, 9, 208, 81, 7, 17, 0, 8, 104, 0, 8, 40, 0, 9, 176, 0, 8, 8, 0, 8, 136, 0, 8, 72, 0, 9, 240, 80, 7, 4, 0, 8, 84, 0, 8, 20, 85, 8, 227, 83, 7, 43, 0, 8, 116, 0, 8, 52, 0, 9, 200, 81, 7, 13, 0, 8, 100, 0, 8, 36, 0, 9, 168, 0, 8, 4, 0, 8, 132, 0, 8, 68, 0, 9, 232, 80, 7, 8, 0, 8, 92, 0, 8, 28, 0, 9, 152, 84, 7, 83, 0, 8, 124, 0, 8, 60, 0, 9, 216, 82, 7, 23, 0, 8, 108, 0, 8, 44, 0, 9, 184, 0, 8, 12, 0, 8, 140, 0, 8, 76, 0, 9, TelnetCommand.EL, 80, 7, 3, 0, 8, 82, 0, 8, 18, 85, 8, 163, 83, 7, 35, 0, 8, 114, 0, 8, 50, 0, 9, 196, 81, 7, 11, 0, 8, 98, 0, 8, 34, 0, 9, 164, 0, 8, 2, 0, 8, 130, 0, 8, 66, 0, 9, 228, 80, 7, 7, 0, 8, 90, 0, 8, 26, 0, 9, 148, 84, 7, 67, 0, 8, 122, 0, 8, 58, 0, 9, 212, 82, 7, 19, 0, 8, 106, 0, 8, 42, 0, 9, GLText.FONT_SIZE_MAX, 0, 8, 10, 0, 8, 138, 0, 8, 74, 0, 9, TelnetCommand.IP, 80, 7, 5, 0, 8, 86, 0, 8, 22, 192, 8, 0, 83, 7, 51, 0, 8, 118, 0, 8, 54, 0, 9, HttpStatus.SC_NO_CONTENT, 81, 7, 15, 0, 8, 102, 0, 8, 38, 0, 9, 172, 0, 8, 6, 0, 8, 134, 0, 8, 70, 0, 9, TelnetCommand.EOF, 80, 7, 9, 0, 8, 94, 0, 8, 30, 0, 9, 156, 84, 7, 99, 0, 8, 126, 0, 8, 62, 0, 9, 220, 82, 7, 27, 0, 8, 110, 0, 8, 46, 0, 9, 188, 0, 8, 14, 0, 8, 142, 0, 8, 78, 0, 9, 252, 96, 7, 256, 0, 8, 81, 0, 8, 17, 85, 8, 131, 82, 7, 31, 0, 8, 113, 0, 8, 49, 0, 9, 194, 80, 7, 10, 0, 8, 97, 0, 8, 33, 0, 9, 162, 0, 8, 1, 0, 8, 129, 0, 8, 65, 0, 9, 226, 80, 7, 6, 0, 8, 89, 0, 8, 25, 0, 9, 146, 83, 7, 59, 0, 8, 121, 0, 8, 57, 0, 9, 210, 81, 7, 17, 0, 8, 105, 0, 8, 41, 0, 9, 178, 0, 8, 9, 0, 8, 137, 0, 8, 73, 0, 9, 242, 80, 7, 4, 0, 8, 85, 0, 8, 21, 80, 8, 258, 83, 7, 43, 0, 8, 117, 0, 8, 53, 0, 9, 202, 81, 7, 13, 0, 8, 101, 0, 8, 37, 0, 9, 170, 0, 8, 5, 0, 8, 133, 0, 8, 69, 0, 9, 234, 80, 7, 8, 0, 8, 93, 0, 8, 29, 0, 9, 154, 84, 7, 83, 0, 8, 125, 0, 8, 61, 0, 9, 218, 82, 7, 23, 0, 8, 109, 0, 8, 45, 0, 9, 186, 0, 8, 13, 0, 8, 141, 0, 8, 77, 0, 9, 250, 80, 7, 3, 0, 8, 83, 0, 8, 19, 85, 8, 195, 83, 7, 35, 0, 8, 115, 0, 8, 51, 0, 9, 198, 81, 7, 11, 0, 8, 99, 0, 8, 35, 0, 9, 166, 0, 8, 3, 0, 8, 131, 0, 8, 67, 0, 9, 230, 80, 7, 7, 0, 8, 91, 0, 8, 27, 0, 9, 150, 84, 7, 67, 0, 8, 123, 0, 8, 59, 0, 9, 214, 82, 7, 19, 0, 8, 107, 0, 8, 43, 0, 9, 182, 0, 8, 11, 0, 8, 139, 0, 8, 75, 0, 9, TelnetCommand.AYT, 80, 7, 5, 0, 8, 87, 0, 8, 23, 192, 8, 0, 83, 7, 51, 0, 8, 119, 0, 8, 55, 0, 9, HttpStatus.SC_PARTIAL_CONTENT, 81, 7, 15, 0, 8, 103, 0, 8, 39, 0, 9, 174, 0, 8, 7, 0, 8, 135, 0, 8, 71, 0, 9, TelnetCommand.ABORT, 80, 7, 9, 0, 8, 95, 0, 8, 31, 0, 9, 158, 84, 7, 99, 0, 8, 127, 0, 8, 63, 0, 9, 222, 82, 7, 27, 0, 8, 111, 0, 8, 47, 0, 9, 190, 0, 8, 15, 0, 8, 143, 0, 8, 79, 0, 9, 254, 96, 7, 256, 0, 8, 80, 0, 8, 16, 84, 8, 115, 82, 7, 31, 0, 8, 112, 0, 8, 48, 0, 9, 193, 80, 7, 10, 0, 8, 96, 0, 8, 32, 0, 9, 161, 0, 8, 0, 0, 8, 128, 0, 8, 64, 0, 9, 225, 80, 7, 6, 0, 8, 88, 0, 8, 24, 0, 9, 145, 83, 7, 59, 0, 8, 120, 0, 8, 56, 0, 9, 209, 81, 7, 17, 0, 8, 104, 0, 8, 40, 0, 9, 177, 0, 8, 8, 0, 8, 136, 0, 8, 72, 0, 9, TelnetCommand.NOP, 80, 7, 4, 0, 8, 84, 0, 8, 20, 85, 8, 227, 83, 7, 43, 0, 8, 116, 0, 8, 52, 0, 9, 201, 81, 7, 13, 0, 8, 100, 0, 8, 36, 0, 9, 169, 0, 8, 4, 0, 8, 132, 0, 8, 68, 0, 9, 233, 80, 7, 8, 0, 8, 92, 0, 8, 28, 0, 9, 153, 84, 7, 83, 0, 8, 124, 0, 8, 60, 0, 9, 217, 82, 7, 23, 0, 8, 108, 0, 8, 44, 0, 9, 185, 0, 8, 12, 0, 8, 140, 0, 8, 76, 0, 9, 249, 80, 7, 3, 0, 8, 82, 0, 8, 18, 85, 8, 163, 83, 7, 35, 0, 8, 114, 0, 8, 50, 0, 9, 197, 81, 7, 11, 0, 8, 98, 0, 8, 34, 0, 9, 165, 0, 8, 2, 0, 8, 130, 0, 8, 66, 0, 9, 229, 80, 7, 7, 0, 8, 90, 0, 8, 26, 0, 9, 149, 84, 7, 67, 0, 8, 122, 0, 8, 58, 0, 9, 213, 82, 7, 19, 0, 8, 106, 0, 8, 42, 0, 9, 181, 0, 8, 10, 0, 8, 138, 0, 8, 74, 0, 9, 245, 80, 7, 5, 0, 8, 86, 0, 8, 22, 192, 8, 0, 83, 7, 51, 0, 8, 118, 0, 8, 54, 0, 9, 205, 81, 7, 15, 0, 8, 102, 0, 8, 38, 0, 9, 173, 0, 8, 6, 0, 8, 134, 0, 8, 70, 0, 9, TelnetCommand.SUSP, 80, 7, 9, 0, 8, 94, 0, 8, 30, 0, 9, 157, 84, 7, 99, 0, 8, 126, 0, 8, 62, 0, 9, 221, 82, 7, 27, 0, 8, 110, 0, 8, 46, 0, 9, 189, 0, 8, 14, 0, 8, 142, 0, 8, 78, 0, 9, 253, 96, 7, 256, 0, 8, 81, 0, 8, 17, 85, 8, 131, 82, 7, 31, 0, 8, 113, 0, 8, 49, 0, 9, 195, 80, 7, 10, 0, 8, 97, 0, 8, 33, 0, 9, 163, 0, 8, 1, 0, 8, 129, 0, 8, 65, 0, 9, 227, 80, 7, 6, 0, 8, 89, 0, 8, 25, 0, 9, 147, 83, 7, 59, 0, 8, 121, 0, 8, 57, 0, 9, 211, 81, 7, 17, 0, 8, 105, 0, 8, 41, 0, 9, 179, 0, 8, 9, 0, 8, 137, 0, 8, 73, 0, 9, 243, 80, 7, 4, 0, 8, 85, 0, 8, 21, 80, 8, 258, 83, 7, 43, 0, 8, 117, 0, 8, 53, 0, 9, HttpStatus.SC_NON_AUTHORITATIVE_INFORMATION, 81, 7, 13, 0, 8, 101, 0, 8, 37, 0, 9, 171, 0, 8, 5, 0, 8, 133, 0, 8, 69, 0, 9, 235, 80, 7, 8, 0, 8, 93, 0, 8, 29, 0, 9, 155, 84, 7, 83, 0, 8, 125, 0, 8, 61, 0, 9, 219, 82, 7, 23, 0, 8, 109, 0, 8, 45, 0, 9, 187, 0, 8, 13, 0, 8, 141, 0, 8, 77, 0, 9, 251, 80, 7, 3, 0, 8, 83, 0, 8, 19, 85, 8, 195, 83, 7, 35, 0, 8, 115, 0, 8, 51, 0, 9, 199, 81, 7, 11, 0, 8, 99, 0, 8, 35, 0, 9, 167, 0, 8, 3, 0, 8, 131, 0, 8, 67, 0, 9, 231, 80, 7, 7, 0, 8, 91, 0, 8, 27, 0, 9, 151, 84, 7, 67, 0, 8, 123, 0, 8, 59, 0, 9, 215, 82, 7, 19, 0, 8, 107, 0, 8, 43, 0, 9, 183, 0, 8, 11, 0, 8, 139, 0, 8, 75, 0, 9, TelnetCommand.EC, 80, 7, 5, 0, 8, 87, 0, 8, 23, 192, 8, 0, 83, 7, 51, 0, 8, 119, 0, 8, 55, 0, 9, HttpStatus.SC_MULTI_STATUS, 81, 7, 15, 0, 8, 103, 0, 8, 39, 0, 9, 175, 0, 8, 7, 0, 8, 135, 0, 8, 71, 0, 9, TelnetCommand.EOR, 80, 7, 9, 0, 8, 95, 0, 8, 31, 0, 9, 159, 84, 7, 99, 0, 8, 127, 0, 8, 63, 0, 9, 223, 82, 7, 27, 0, 8, 111, 0, 8, 47, 0, 9, 191, 0, 8, 15, 0, 8, 143, 0, 8, 79, 0, 9, 255};
    private int[] c;
    private int[] hn;
    private int[] r;
    private int[] u;
    private int[] v;
    private int[] x;

    InfTree() {
    }

    /* JADX WARNING: Code restructure failed: missing block: B:101:0x0252, code lost:
        r31 = r3;
        r28 = r12;
        r5 = r5 + 1;
        r1 = r23;
        r4 = r36;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:77:0x01b0, code lost:
        r28 = r12;
        r0.r[1] = (byte) (r5 - r22);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:78:0x01ba, code lost:
        if (r9 < r3) goto L_0x01c4;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:79:0x01bc, code lost:
        r0.r[0] = 192;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:81:0x01c6, code lost:
        if (r43[r9] >= r4) goto L_0x01e1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:82:0x01c8, code lost:
        r2 = r0.r;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:83:0x01ce, code lost:
        if (r43[r9] >= 256) goto L_0x01d2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:84:0x01d0, code lost:
        r12 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:85:0x01d2, code lost:
        r12 = 96;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:86:0x01d4, code lost:
        r2[0] = (byte) r12;
        r7 = r9 + 1;
        r0.r[2] = r43[r9];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:87:0x01e1, code lost:
        r0.r[0] = (byte) ((r38[r43[r9] - r4] + 16) + 64);
        r7 = r9 + 1;
        r0.r[2] = r37[r43[r9] - r4];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:88:0x01fb, code lost:
        r9 = r7;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:89:0x01fc, code lost:
        r2 = 1 << (r5 - r22);
        r7 = r8 >>> r22;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:90:0x0203, code lost:
        if (r7 >= r1) goto L_0x021c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:91:0x0205, code lost:
        java.lang.System.arraycopy(r0.r, 0, r6, (r16 + r7) * 3, 3);
        r7 = r7 + r2;
        r1 = r1;
        r3 = r3;
        r4 = r36;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:92:0x021c, code lost:
        r30 = r1;
        r31 = r3;
        r13 = 1 << (r5 - 1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:94:0x0229, code lost:
        if ((r8 & r13) == 0) goto L_0x022f;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:95:0x022b, code lost:
        r8 = r8 ^ r13;
        r13 = r13 >>> 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:96:0x022f, code lost:
        r8 = r8 ^ r13;
        r3 = (1 << r22) - 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:98:0x023b, code lost:
        if ((r8 & r3) == r0.x[r10]) goto L_0x0246;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:99:0x023d, code lost:
        r10 = r10 - 1;
        r22 = r22 - r11;
        r3 = (1 << r22) - 1;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private int huft_build(int[] r33, int r34, int r35, int r36, int[] r37, int[] r38, int[] r39, int[] r40, int[] r41, int[] r42, int[] r43) {
        /*
            r32 = this;
            r0 = r32
            r3 = r35
            r4 = r36
            r6 = r41
            r8 = 0
            r9 = r8
            r8 = r3
        L_0x000b:
            int[] r10 = r0.c
            int r11 = r34 + r9
            r11 = r33[r11]
            r12 = r10[r11]
            r13 = 1
            int r12 = r12 + r13
            r10[r11] = r12
            int r9 = r9 + r13
            r10 = -1
            int r8 = r8 + r10
            if (r8 != 0) goto L_0x0278
            int[] r11 = r0.c
            r12 = 0
            r11 = r11[r12]
            if (r11 != r3) goto L_0x0028
            r39[r12] = r10
            r40[r12] = r12
            return r12
        L_0x0028:
            r11 = r40[r12]
            r14 = 1
        L_0x002b:
            r15 = r14
            r10 = 15
            r13 = r15
            if (r13 > r10) goto L_0x003d
            int[] r10 = r0.c
            r10 = r10[r13]
            if (r10 == 0) goto L_0x0038
            goto L_0x003d
        L_0x0038:
            int r14 = r13 + 1
            r10 = -1
            r13 = 1
            goto L_0x002b
        L_0x003d:
            r10 = r13
            if (r11 >= r13) goto L_0x0041
            r11 = r13
        L_0x0041:
            r8 = 15
        L_0x0043:
            if (r8 == 0) goto L_0x0050
            int[] r12 = r0.c
            r12 = r12[r8]
            if (r12 == 0) goto L_0x004c
            goto L_0x0050
        L_0x004c:
            int r8 = r8 + -1
            r12 = 0
            goto L_0x0043
        L_0x0050:
            r12 = r8
            if (r11 <= r8) goto L_0x0054
            r11 = r8
        L_0x0054:
            r14 = 0
            r40[r14] = r11
            r14 = 1
            int r15 = r14 << r13
        L_0x005a:
            r14 = r15
            r15 = -3
            if (r13 >= r8) goto L_0x006d
            int[] r5 = r0.c
            r5 = r5[r13]
            int r5 = r14 - r5
            r14 = r5
            if (r5 >= 0) goto L_0x0068
            return r15
        L_0x0068:
            int r13 = r13 + 1
            int r15 = r14 << 1
            goto L_0x005a
        L_0x006d:
            int[] r5 = r0.c
            r5 = r5[r8]
            int r5 = r14 - r5
            r14 = r5
            if (r5 >= 0) goto L_0x0077
            return r15
        L_0x0077:
            int[] r5 = r0.c
            r19 = r5[r8]
            int r19 = r19 + r14
            r5[r8] = r19
            int[] r5 = r0.x
            r18 = 0
            r13 = r18
            r17 = 1
            r5[r17] = r18
            r5 = 1
            r19 = 2
            r9 = r5
            r5 = 2
        L_0x008e:
            r16 = -1
            int r8 = r8 + -1
            if (r8 == 0) goto L_0x00ab
            r20 = r8
            int[] r8 = r0.x
            r21 = r10
            int[] r10 = r0.c
            r10 = r10[r9]
            int r10 = r10 + r13
            r13 = r10
            r8[r5] = r10
            int r5 = r5 + 1
            int r9 = r9 + 1
            r8 = r20
            r10 = r21
            goto L_0x008e
        L_0x00ab:
            r20 = r8
            r21 = r10
            r8 = 0
            r9 = 0
        L_0x00b1:
            int r10 = r34 + r9
            r10 = r33[r10]
            r13 = r10
            if (r10 == 0) goto L_0x00c2
            int[] r10 = r0.x
            r16 = r10[r13]
            int r20 = r16 + 1
            r10[r13] = r20
            r43[r16] = r8
        L_0x00c2:
            int r9 = r9 + 1
            int r8 = r8 + 1
            if (r8 < r3) goto L_0x0274
            int[] r10 = r0.x
            r3 = r10[r12]
            int[] r10 = r0.x
            r16 = 0
            r8 = r16
            r10[r16] = r16
            r9 = 0
            r10 = -1
            int r1 = -r11
            r22 = r1
            int[] r1 = r0.u
            r1[r16] = r16
            r1 = 0
            r16 = r1
            r20 = r5
            r5 = r21
            r1 = 0
        L_0x00e5:
            if (r5 > r12) goto L_0x0260
            r23 = r1
            int[] r1 = r0.c
            r1 = r1[r5]
        L_0x00ed:
            int r21 = r1 + -1
            if (r1 == 0) goto L_0x0252
            r1 = r23
        L_0x00f3:
            int r2 = r22 + r11
            r24 = r13
            if (r5 <= r2) goto L_0x01b0
            int r10 = r10 + 1
            int r22 = r22 + r11
            int r1 = r12 - r22
            if (r1 <= r11) goto L_0x0103
            r2 = r11
            goto L_0x0104
        L_0x0103:
            r2 = r1
        L_0x0104:
            r1 = r2
            int r2 = r5 - r22
            r25 = r2
            r17 = 1
            int r2 = r17 << r2
            r23 = r2
            int r13 = r21 + 1
            if (r2 <= r13) goto L_0x014f
            int r2 = r21 + 1
            int r23 = r23 - r2
            r2 = r5
            r13 = r25
            if (r13 >= r1) goto L_0x014a
        L_0x011c:
            r17 = 1
            int r13 = r13 + 1
            if (r13 >= r1) goto L_0x0143
            r26 = r1
            int r1 = r23 << 1
            r23 = r1
            r27 = r13
            int[] r13 = r0.c
            int r2 = r2 + 1
            r13 = r13[r2]
            if (r1 > r13) goto L_0x0138
            r20 = r2
            r13 = r27
            goto L_0x0153
        L_0x0138:
            int[] r1 = r0.c
            r1 = r1[r2]
            int r23 = r23 - r1
            r1 = r26
            r13 = r27
            goto L_0x011c
        L_0x0143:
            r26 = r1
            r27 = r13
            r20 = r2
            goto L_0x0153
        L_0x014a:
            r26 = r1
            r20 = r2
            goto L_0x0153
        L_0x014f:
            r26 = r1
            r13 = r25
        L_0x0153:
            r1 = 1
            int r2 = r1 << r13
            r1 = 0
            r18 = r42[r1]
            int r1 = r18 + r2
            r28 = r12
            r12 = 1440(0x5a0, float:2.018E-42)
            if (r1 <= r12) goto L_0x0162
            return r15
        L_0x0162:
            int[] r1 = r0.u
            r12 = 0
            r18 = r42[r12]
            r16 = r18
            r1[r10] = r18
            r1 = r42[r12]
            int r1 = r1 + r2
            r42[r12] = r1
            if (r10 == 0) goto L_0x01aa
            int[] r1 = r0.x
            r1[r10] = r8
            int[] r1 = r0.r
            r29 = r2
            byte r2 = (byte) r13
            r1[r12] = r2
            int[] r1 = r0.r
            byte r2 = (byte) r11
            r12 = 1
            r1[r12] = r2
            int r1 = r22 - r11
            int r13 = r8 >>> r1
            int[] r1 = r0.r
            int[] r2 = r0.u
            int r12 = r10 + -1
            r2 = r2[r12]
            int r2 = r16 - r2
            int r2 = r2 - r13
            r1[r19] = r2
            int[] r1 = r0.r
            int[] r2 = r0.u
            int r12 = r10 + -1
            r2 = r2[r12]
            int r2 = r2 + r13
            r12 = 3
            int r2 = r2 * 3
            r7 = 0
            java.lang.System.arraycopy(r1, r7, r6, r2, r12)
        L_0x01a4:
            r12 = r28
            r1 = r29
            goto L_0x00f3
        L_0x01aa:
            r29 = r2
            r7 = 0
            r39[r7] = r16
            goto L_0x01a4
        L_0x01b0:
            r28 = r12
            int[] r2 = r0.r
            int r7 = r5 - r22
            byte r7 = (byte) r7
            r12 = 1
            r2[r12] = r7
            if (r9 < r3) goto L_0x01c4
            int[] r2 = r0.r
            r7 = 192(0xc0, float:2.69E-43)
            r12 = 0
            r2[r12] = r7
            goto L_0x01fc
        L_0x01c4:
            r2 = r43[r9]
            if (r2 >= r4) goto L_0x01e1
            int[] r2 = r0.r
            r7 = r43[r9]
            r12 = 256(0x100, float:3.59E-43)
            if (r7 >= r12) goto L_0x01d2
            r12 = 0
            goto L_0x01d4
        L_0x01d2:
            r12 = 96
        L_0x01d4:
            byte r7 = (byte) r12
            r12 = 0
            r2[r12] = r7
            int[] r2 = r0.r
            int r7 = r9 + 1
            r9 = r43[r9]
            r2[r19] = r9
            goto L_0x01fb
        L_0x01e1:
            int[] r2 = r0.r
            r7 = r43[r9]
            int r7 = r7 - r4
            r7 = r38[r7]
            int r7 = r7 + 16
            int r7 = r7 + 64
            byte r7 = (byte) r7
            r13 = 0
            r2[r13] = r7
            int[] r2 = r0.r
            int r7 = r9 + 1
            r9 = r43[r9]
            int r9 = r9 - r4
            r9 = r37[r9]
            r2[r19] = r9
        L_0x01fb:
            r9 = r7
        L_0x01fc:
            int r2 = r5 - r22
            r7 = 1
            int r2 = r7 << r2
            int r7 = r8 >>> r22
        L_0x0203:
            if (r7 >= r1) goto L_0x021c
            int[] r13 = r0.r
            int r23 = r16 + r7
            r30 = r1
            r31 = r3
            r1 = 3
            int r3 = r23 * 3
            r4 = 0
            java.lang.System.arraycopy(r13, r4, r6, r3, r1)
            int r7 = r7 + r2
            r1 = r30
            r3 = r31
            r4 = r36
            goto L_0x0203
        L_0x021c:
            r30 = r1
            r31 = r3
            r4 = 0
            int r1 = r5 + -1
            r3 = 1
            int r1 = r3 << r1
            r13 = r1
        L_0x0227:
            r1 = r8 & r13
            if (r1 == 0) goto L_0x022f
            r8 = r8 ^ r13
            int r13 = r13 >>> 1
            goto L_0x0227
        L_0x022f:
            r8 = r8 ^ r13
            r1 = 1
            int r3 = r1 << r22
            int r3 = r3 - r1
        L_0x0234:
            r1 = r3
            r3 = r8 & r1
            int[] r7 = r0.x
            r7 = r7[r10]
            if (r3 == r7) goto L_0x0246
            int r10 = r10 + -1
            int r22 = r22 - r11
            r7 = 1
            int r3 = r7 << r22
            int r3 = r3 - r7
            goto L_0x0234
        L_0x0246:
            r1 = r21
            r12 = r28
            r23 = r30
            r3 = r31
            r4 = r36
            goto L_0x00ed
        L_0x0252:
            r31 = r3
            r28 = r12
            r4 = 0
            r7 = 1
            int r5 = r5 + 1
            r1 = r23
            r4 = r36
            goto L_0x00e5
        L_0x0260:
            r23 = r1
            r31 = r3
            r28 = r12
            r4 = 0
            r7 = 1
            if (r14 == 0) goto L_0x0271
            r1 = r28
            if (r1 == r7) goto L_0x0273
            r2 = -5
            r4 = -5
            goto L_0x0273
        L_0x0271:
            r1 = r28
        L_0x0273:
            return r4
        L_0x0274:
            r4 = r36
            goto L_0x00b1
        L_0x0278:
            r4 = r36
            goto L_0x000b
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.InfTree.huft_build(int[], int, int, int, int[], int[], int[], int[], int[], int[], int[]):int");
    }

    /* access modifiers changed from: package-private */
    public int inflate_trees_bits(int[] c2, int[] bb, int[] tb, int[] hp, ZStream z) {
        ZStream zStream = z;
        initWorkArea(19);
        this.hn[0] = 0;
        int result = huft_build(c2, 0, 19, 19, (int[]) null, (int[]) null, tb, bb, hp, this.hn, this.v);
        if (result == -3) {
            zStream.msg = "oversubscribed dynamic bit lengths tree";
            return result;
        } else if (result != -5 && bb[0] != 0) {
            return result;
        } else {
            zStream.msg = "incomplete dynamic bit lengths tree";
            return -3;
        }
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:10:0x0065, code lost:
        if (r18 > 257) goto L_0x006d;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int inflate_trees_dynamic(int r18, int r19, int[] r20, int[] r21, int[] r22, int[] r23, int[] r24, int[] r25, org.jboss.netty.util.internal.jzlib.ZStream r26) {
        /*
            r17 = this;
            r12 = r17
            r13 = r26
            r14 = 288(0x120, float:4.04E-43)
            r12.initWorkArea(r14)
            int[] r0 = r12.hn
            r15 = 0
            r0[r15] = r15
            int[] r5 = cplens
            int[] r6 = cplext
            int[] r10 = r12.hn
            int[] r11 = r12.v
            r2 = 0
            r4 = 257(0x101, float:3.6E-43)
            r0 = r17
            r1 = r20
            r3 = r18
            r7 = r23
            r8 = r21
            r9 = r25
            int r11 = r0.huft_build(r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11)
            r10 = -4
            r9 = -3
            if (r11 != 0) goto L_0x0087
            r0 = r21[r15]
            if (r0 != 0) goto L_0x0037
            r3 = r18
            r14 = r11
            r2 = -3
            r4 = -4
            goto L_0x008c
        L_0x0037:
            r12.initWorkArea(r14)
            r4 = 0
            int[] r5 = cpdist
            int[] r6 = cpdext
            int[] r14 = r12.hn
            int[] r8 = r12.v
            r0 = r17
            r1 = r20
            r2 = r18
            r3 = r19
            r7 = r24
            r16 = r8
            r8 = r22
            r9 = r25
            r10 = r14
            r14 = r11
            r11 = r16
            int r0 = r0.huft_build(r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11)
            if (r0 != 0) goto L_0x006b
            r2 = r22[r15]
            if (r2 != 0) goto L_0x0068
            r2 = 257(0x101, float:3.6E-43)
            r3 = r18
            if (r3 <= r2) goto L_0x006a
            goto L_0x006d
        L_0x0068:
            r3 = r18
        L_0x006a:
            return r15
        L_0x006b:
            r3 = r18
        L_0x006d:
            r2 = -3
            if (r0 != r2) goto L_0x0075
            java.lang.String r2 = "oversubscribed distance tree"
            r13.msg = r2
            goto L_0x0086
        L_0x0075:
            r2 = -5
            if (r0 != r2) goto L_0x007e
            java.lang.String r2 = "incomplete distance tree"
            r13.msg = r2
            r0 = -3
            goto L_0x0086
        L_0x007e:
            r4 = -4
            if (r0 == r4) goto L_0x0086
            java.lang.String r2 = "empty distance tree with lengths"
            r13.msg = r2
            r0 = -3
        L_0x0086:
            return r0
        L_0x0087:
            r3 = r18
            r14 = r11
            r2 = -3
            r4 = -4
        L_0x008c:
            if (r14 != r2) goto L_0x0093
            java.lang.String r0 = "oversubscribed literal/length tree"
            r13.msg = r0
            goto L_0x009b
        L_0x0093:
            if (r14 == r4) goto L_0x009b
            java.lang.String r0 = "incomplete literal/length tree"
            r13.msg = r0
            r11 = -3
            goto L_0x009c
        L_0x009b:
            r11 = r14
        L_0x009c:
            return r11
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.InfTree.inflate_trees_dynamic(int, int, int[], int[], int[], int[], int[], int[], org.jboss.netty.util.internal.jzlib.ZStream):int");
    }

    static int inflate_trees_fixed(int[] bl, int[] bd, int[][] tl, int[][] td) {
        bl[0] = 9;
        bd[0] = 5;
        tl[0] = fixed_tl;
        td[0] = fixed_td;
        return 0;
    }

    private void initWorkArea(int vsize) {
        if (this.hn == null) {
            this.hn = new int[1];
            this.v = new int[vsize];
            this.c = new int[16];
            this.r = new int[3];
            this.u = new int[15];
            this.x = new int[16];
            return;
        }
        if (this.v.length < vsize) {
            this.v = new int[vsize];
        } else {
            for (int i = 0; i < vsize; i++) {
                this.v[i] = 0;
            }
        }
        for (int i2 = 0; i2 < 16; i2++) {
            this.c[i2] = 0;
        }
        for (int i3 = 0; i3 < 3; i3++) {
            this.r[i3] = 0;
        }
        System.arraycopy(this.c, 0, this.u, 0, 15);
        System.arraycopy(this.c, 0, this.x, 0, 16);
    }
}
