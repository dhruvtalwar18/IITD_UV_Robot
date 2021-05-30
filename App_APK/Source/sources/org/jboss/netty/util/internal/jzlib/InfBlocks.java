package org.jboss.netty.util.internal.jzlib;

import javax.jmdns.impl.constants.DNSRecordClass;
import org.apache.commons.net.bsd.RCommandClient;

final class InfBlocks {
    private static final int BAD = 9;
    private static final int BTREE = 4;
    private static final int CODES = 6;
    private static final int DONE = 8;
    private static final int DRY = 7;
    private static final int DTREE = 5;
    private static final int LENS = 1;
    private static final int STORED = 2;
    private static final int TABLE = 3;
    private static final int TYPE = 0;
    private static final int[] border = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    private static final int[] inflate_mask = {0, 1, 3, 7, 15, 31, 63, 127, 255, 511, RCommandClient.MAX_CLIENT_PORT, 2047, 4095, 8191, 16383, DNSRecordClass.CLASS_MASK, 65535};
    private final int[] bb = new int[1];
    int bitb;
    int bitk;
    private int[] blens;
    private long check;
    private final Object checkfn;
    private final InfCodes codes = new InfCodes();
    final int end;
    private int[] hufts = new int[4320];
    private int index;
    private final InfTree inftree = new InfTree();
    private int last;
    private int left;
    private int mode;
    int read;
    private int table;
    private final int[] tb = new int[1];
    byte[] window;
    int write;

    InfBlocks(ZStream z, Object checkfn2, int w) {
        this.window = new byte[w];
        this.end = w;
        this.checkfn = checkfn2;
        this.mode = 0;
        reset(z, (long[]) null);
    }

    /* access modifiers changed from: package-private */
    public void reset(ZStream z, long[] c) {
        if (c != null) {
            c[0] = this.check;
        }
        this.mode = 0;
        this.bitk = 0;
        this.bitb = 0;
        this.write = 0;
        this.read = 0;
        if (this.checkfn != null) {
            long adler32 = Adler32.adler32(0, (byte[]) null, 0, 0);
            this.check = adler32;
            z.adler = adler32;
        }
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Can't fix incorrect switch cases order */
    /* JADX WARNING: Code restructure failed: missing block: B:101:0x037b, code lost:
        if (r9 >= (r1 + r5)) goto L_0x03af;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:102:0x037d, code lost:
        if (r15 == 0) goto L_0x0392;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:103:0x037f, code lost:
        r2 = 0;
        r15 = r15 - 1;
        r13 = r13 | ((r11.next_in[r12] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << r9);
        r9 = r9 + 8;
        r12 = r12 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:104:0x0392, code lost:
        r0.bitb = r13;
        r0.bitk = r9;
        r11.avail_in = r15;
        r34 = r15;
        r35 = r3;
        r11.total_in += (long) (r12 - r11.next_in_index);
        r11.next_in_index = r12;
        r0.write = r8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:105:0x03ae, code lost:
        return inflate_flush(r11, r2);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:106:0x03af, code lost:
        r35 = r3;
        r34 = r15;
        r3 = r13 >>> r1;
        r7 = r7 + (inflate_mask[r5] & r3);
        r3 = r3 >>> r5;
        r9 = (r9 - r1) - r5;
        r5 = r0.index;
        r1 = r0.table;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:107:0x03cd, code lost:
        if ((r5 + r7) > (((r1 & 31) + 258) + ((r1 >> 5) & 31))) goto L_0x03fe;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:108:0x03cf, code lost:
        r10 = r35;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:109:0x03d3, code lost:
        if (r10 != 16) goto L_0x03d9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:111:0x03d6, code lost:
        if (r5 >= 1) goto L_0x03d9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:112:0x03d9, code lost:
        if (r10 != 16) goto L_0x03e2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:113:0x03db, code lost:
        r13 = r0.blens[r5 - 1];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:114:0x03e2, code lost:
        r13 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:115:0x03e3, code lost:
        r15 = r5 + 1;
        r0.blens[r5] = r13;
        r7 = r7 - 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:116:0x03ea, code lost:
        if (r7 != 0) goto L_0x03fc;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:117:0x03ec, code lost:
        r0.index = r15;
        r7 = r1;
        r13 = r3;
        r10 = r9;
        r9 = r12;
        r15 = r34;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:118:0x03f6, code lost:
        r14 = r8;
        r8 = r15;
        r15 = r25;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:119:0x03fc, code lost:
        r5 = r15;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:120:0x03fe, code lost:
        r10 = r35;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:123:0x0427, code lost:
        r8 = r14;
        r25 = r15;
        r6 = 9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:124:0x042c, code lost:
        r0.mode = r6;
        r11.msg = "too many length or distance symbols";
        r0.bitb = r3;
        r0.bitk = r4;
        r11.avail_in = r2;
        r11.total_in += (long) (r5 - r11.next_in_index);
        r11.next_in_index = r5;
        r0.write = r8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:125:0x044b, code lost:
        return inflate_flush(r11, -3);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:14:0x006f, code lost:
        r10 = r1;
        r6 = r3;
        r3 = r2;
        r37 = r5;
        r5 = r4;
        r4 = r37;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:206:0x061f, code lost:
        r14 = r8;
        r6 = r25;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:21:0x00aa, code lost:
        r6 = r3 & 16383;
        r7 = r6;
        r0.table = r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:22:0x00b3, code lost:
        if ((r7 & 31) > 29) goto L_0x0427;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:24:0x00bb, code lost:
        if (((r7 >> 5) & 31) <= 29) goto L_0x00c4;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:25:0x00bd, code lost:
        r8 = r14;
        r25 = r15;
        r6 = 9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:26:0x00c4, code lost:
        r7 = ((r7 & 31) + 258) + ((r7 >> 5) & 31);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:27:0x00d0, code lost:
        if (r0.blens == null) goto L_0x00e2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x00d5, code lost:
        if (r0.blens.length >= r7) goto L_0x00d8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:30:0x00d8, code lost:
        r6 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:31:0x00d9, code lost:
        if (r6 >= r7) goto L_0x00e6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:32:0x00db, code lost:
        r0.blens[r6] = r13;
        r6 = r6 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:33:0x00e2, code lost:
        r0.blens = new int[r7];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:34:0x00e6, code lost:
        r3 = r3 >>> 14;
        r4 = r4 - 14;
        r0.index = r13;
        r0.mode = 4;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:36:0x00f9, code lost:
        if (r0.index >= ((r0.table >>> 10) + 4)) goto L_0x013f;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:37:0x00fb, code lost:
        if (r5 >= r8) goto L_0x0129;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:38:0x00fd, code lost:
        if (r3 == 0) goto L_0x0110;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:39:0x00ff, code lost:
        r10 = 0;
        r3 = r3 - 1;
        r6 = r6 | ((r11.next_in[r4] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << r5);
        r5 = r5 + 8;
        r4 = r4 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:40:0x0110, code lost:
        r0.bitb = r6;
        r0.bitk = r5;
        r11.avail_in = r3;
        r11.total_in += (long) (r4 - r11.next_in_index);
        r11.next_in_index = r4;
        r0.write = r14;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:41:0x0128, code lost:
        return inflate_flush(r11, r10);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:42:0x0129, code lost:
        r1 = r0.blens;
        r2 = border;
        r8 = r0.index;
        r0.index = r8 + 1;
        r1[r2[r8]] = r6 & 7;
        r6 = r6 >>> 3;
        r5 = r5 - 3;
        r8 = 3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:44:0x0143, code lost:
        if (r0.index >= 19) goto L_0x0154;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:45:0x0145, code lost:
        r1 = r0.blens;
        r2 = border;
        r8 = r0.index;
        r0.index = r8 + 1;
        r1[r2[r8]] = r13;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:46:0x0154, code lost:
        r0.bb[r13] = 7;
        r13 = r3;
        r8 = r4;
        r9 = r5;
        r12 = r6;
        r7 = r0.inftree.inflate_trees_bits(r0.blens, r0.bb, r0.tb, r0.hufts, r39);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:47:0x016f, code lost:
        if (r7 == 0) goto L_0x0195;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:48:0x0171, code lost:
        r1 = r7;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:49:0x0173, code lost:
        if (r1 != -3) goto L_0x017c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:50:0x0175, code lost:
        r0.blens = null;
        r0.mode = 9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:51:0x017c, code lost:
        r0.bitb = r12;
        r0.bitk = r9;
        r11.avail_in = r13;
        r11.total_in += (long) (r8 - r11.next_in_index);
        r11.next_in_index = r8;
        r0.write = r14;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:52:0x0194, code lost:
        return inflate_flush(r11, r1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:53:0x0195, code lost:
        r0.index = 0;
        r0.mode = 5;
        r37 = r9;
        r9 = r8;
        r8 = r13;
        r13 = r12;
        r12 = r10;
        r10 = r37;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:54:0x01a4, code lost:
        r1 = r0.table;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:55:0x01b2, code lost:
        if (r0.index < (((r1 & 31) + 258) + ((r1 >> 5) & 31))) goto L_0x02e1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:56:0x01b4, code lost:
        r0.tb[0] = -1;
        r22 = r12;
        r12 = new int[1];
        r7 = new int[]{9};
        r3 = r0.table;
        r19 = r3;
        r18 = new int[1];
        r20 = new int[]{6};
        r23 = r7;
        r25 = r15;
        r15 = r8;
        r8 = r12;
        r26 = r12;
        r27 = r14;
        r12 = r9;
        r28 = r10;
        r1 = r0.inftree.inflate_trees_dynamic((r3 & 31) + 257, ((r3 >> 5) & 31) + 1, r0.blens, r7, r20, r18, r8, r0.hufts, r39);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:57:0x0207, code lost:
        if (r1 == 0) goto L_0x0230;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:58:0x0209, code lost:
        if (r1 != -3) goto L_0x0212;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:59:0x020b, code lost:
        r0.blens = null;
        r0.mode = 9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:60:0x0212, code lost:
        r0.bitb = r13;
        r0.bitk = r28;
        r11.avail_in = r15;
        r11.total_in += (long) (r12 - r11.next_in_index);
        r11.next_in_index = r12;
        r0.write = r27;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:61:0x022f, code lost:
        return inflate_flush(r11, r1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:62:0x0230, code lost:
        r8 = r27;
        r0.codes.init(r23[0], r20[0], r0.hufts, r18[0], r0.hufts, r26[0]);
        r0.mode = 6;
        r7 = r1;
        r4 = r28;
        r5 = r12;
        r1 = r22;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:63:0x0254, code lost:
        r0.bitb = r13;
        r0.bitk = r4;
        r11.avail_in = r15;
        r11.total_in += (long) (r5 - r11.next_in_index);
        r11.next_in_index = r5;
        r0.write = r8;
        r2 = r0.codes.proc(r0, r11, r1);
        r1 = r2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:64:0x0270, code lost:
        if (r2 == 1) goto L_0x0277;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:66:0x0276, code lost:
        return inflate_flush(r11, r1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:67:0x0277, code lost:
        r1 = 0;
        r5 = r11.next_in_index;
        r2 = r11.avail_in;
        r3 = r0.bitb;
        r4 = r0.bitk;
        r14 = r0.write;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:68:0x0284, code lost:
        if (r14 >= r0.read) goto L_0x028c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:69:0x0286, code lost:
        r6 = (r0.read - r14) - 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:70:0x028c, code lost:
        r6 = r0.end - r14;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:72:0x0291, code lost:
        if (r0.last != 0) goto L_0x0298;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:73:0x0293, code lost:
        r0.mode = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:74:0x0298, code lost:
        r0.mode = 7;
        r15 = r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:75:0x029c, code lost:
        r0.write = r14;
        r1 = inflate_flush(r11, r1);
        r14 = r0.write;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:76:0x02a8, code lost:
        if (r0.read == r0.write) goto L_0x02c3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:77:0x02aa, code lost:
        r0.bitb = r3;
        r0.bitk = r4;
        r11.avail_in = r2;
        r11.total_in += (long) (r5 - r11.next_in_index);
        r11.next_in_index = r5;
        r0.write = r14;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:78:0x02c2, code lost:
        return inflate_flush(r11, r1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:79:0x02c3, code lost:
        r0.mode = 8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:80:0x02c7, code lost:
        r0.bitb = r3;
        r0.bitk = r4;
        r11.avail_in = r2;
        r11.total_in += (long) (r5 - r11.next_in_index);
        r11.next_in_index = r5;
        r0.write = r14;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:81:0x02e0, code lost:
        return inflate_flush(r11, 1);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:82:0x02e1, code lost:
        r22 = r12;
        r25 = r15;
        r15 = r8;
        r12 = r9;
        r9 = r10;
        r8 = r14;
        r1 = r0.bb[0];
        r2 = r22;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:83:0x02f4, code lost:
        if (r9 >= r1) goto L_0x0322;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:84:0x02f6, code lost:
        if (r15 == 0) goto L_0x0309;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:85:0x02f8, code lost:
        r2 = 0;
        r15 = r15 - 1;
        r13 = r13 | ((r11.next_in[r12] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << r9);
        r9 = r9 + 8;
        r12 = r12 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:86:0x0309, code lost:
        r0.bitb = r13;
        r0.bitk = r9;
        r11.avail_in = r15;
        r11.total_in += (long) (r12 - r11.next_in_index);
        r11.next_in_index = r12;
        r0.write = r8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:87:0x0321, code lost:
        return inflate_flush(r11, r2);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:88:0x0322, code lost:
        r3 = r0.tb[0];
        r1 = r0.hufts[((r0.tb[0] + (r13 & inflate_mask[r1])) * 3) + 1];
        r3 = r0.hufts[((r0.tb[0] + (inflate_mask[r1] & r13)) * 3) + 2];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:89:0x0353, code lost:
        if (r3 >= 16) goto L_0x0369;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:90:0x0355, code lost:
        r5 = r0.blens;
        r7 = r0.index;
        r0.index = r7 + 1;
        r5[r7] = r3;
        r7 = r1;
        r13 = r13 >>> r1;
        r10 = r9 - r1;
        r9 = r12;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:91:0x0366, code lost:
        r12 = r2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:93:0x036b, code lost:
        if (r3 != 18) goto L_0x036f;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:94:0x036d, code lost:
        r5 = 7;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:95:0x036f, code lost:
        r5 = r3 - 14;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:97:0x0373, code lost:
        if (r3 != 18) goto L_0x0378;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:98:0x0375, code lost:
        r7 = 11;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:99:0x0378, code lost:
        r7 = 3;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int proc(org.jboss.netty.util.internal.jzlib.ZStream r39, int r40) {
        /*
            r38 = this;
            r0 = r38
            r11 = r39
            int r1 = r11.next_in_index
            int r2 = r11.avail_in
            int r3 = r0.bitb
            int r4 = r0.bitk
            int r5 = r0.write
            int r6 = r0.read
            r12 = 1
            if (r5 >= r6) goto L_0x0018
            int r6 = r0.read
            int r6 = r6 - r5
            int r6 = r6 - r12
            goto L_0x001b
        L_0x0018:
            int r6 = r0.end
            int r6 = r6 - r5
        L_0x001b:
            r13 = 0
            r14 = r5
            r7 = 0
            r5 = r1
            r1 = r40
        L_0x0021:
            r15 = r6
            int r6 = r0.mode
            r12 = 7
            r8 = 3
            switch(r6) {
                case 0: goto L_0x0579;
                case 1: goto L_0x04ff;
                case 2: goto L_0x044c;
                case 3: goto L_0x0079;
                case 4: goto L_0x006f;
                case 5: goto L_0x0067;
                case 6: goto L_0x0060;
                case 7: goto L_0x029c;
                case 8: goto L_0x02c7;
                case 9: goto L_0x0046;
                default: goto L_0x0029;
            }
        L_0x0029:
            r8 = r14
            r25 = r15
            r1 = -2
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r9 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r9 = r9 + r12
            r11.total_in = r9
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x0046:
            r1 = -3
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r8 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r8 = r8 + r12
            r11.total_in = r8
            r11.next_in_index = r5
            r0.write = r14
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x0060:
            r13 = r3
            r8 = r14
            r25 = r15
            r15 = r2
            goto L_0x0254
        L_0x0067:
            r12 = r1
            r8 = r2
            r13 = r3
            r10 = r4
            r9 = r5
        L_0x006c:
            r6 = -3
            goto L_0x01a4
        L_0x006f:
            r10 = r1
            r6 = r3
            r3 = r2
            r37 = r5
            r5 = r4
            r4 = r37
            goto L_0x00f1
        L_0x0079:
            r6 = 14
            if (r4 >= r6) goto L_0x00aa
            if (r2 == 0) goto L_0x0091
            r1 = 0
            int r2 = r2 + -1
            byte[] r6 = r11.next_in
            int r18 = r5 + 1
            byte r5 = r6[r5]
            r5 = r5 & 255(0xff, float:3.57E-43)
            int r5 = r5 << r4
            r3 = r3 | r5
            int r4 = r4 + 8
            r5 = r18
            goto L_0x0079
        L_0x0091:
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r8 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r8 = r8 + r12
            r11.total_in = r8
            r11.next_in_index = r5
            r0.write = r14
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x00aa:
            r6 = r3 & 16383(0x3fff, float:2.2957E-41)
            r7 = r6
            r0.table = r6
            r6 = r7 & 31
            r10 = 29
            if (r6 > r10) goto L_0x0427
            int r6 = r7 >> 5
            r6 = r6 & 31
            r10 = 29
            if (r6 <= r10) goto L_0x00c4
            r8 = r14
            r25 = r15
            r6 = 9
            goto L_0x042c
        L_0x00c4:
            r6 = r7 & 31
            int r6 = r6 + 258
            int r10 = r7 >> 5
            r10 = r10 & 31
            int r7 = r6 + r10
            int[] r6 = r0.blens
            if (r6 == 0) goto L_0x00e2
            int[] r6 = r0.blens
            int r6 = r6.length
            if (r6 >= r7) goto L_0x00d8
            goto L_0x00e2
        L_0x00d8:
            r6 = 0
        L_0x00d9:
            if (r6 >= r7) goto L_0x00e6
            int[] r10 = r0.blens
            r10[r6] = r13
            int r6 = r6 + 1
            goto L_0x00d9
        L_0x00e2:
            int[] r6 = new int[r7]
            r0.blens = r6
        L_0x00e6:
            int r3 = r3 >>> 14
            int r4 = r4 + -14
            r0.index = r13
            r6 = 4
            r0.mode = r6
            goto L_0x006f
        L_0x00f1:
            int r1 = r0.index
            int r2 = r0.table
            int r2 = r2 >>> 10
            int r2 = r2 + 4
            if (r1 >= r2) goto L_0x013f
        L_0x00fb:
            if (r5 >= r8) goto L_0x0129
            if (r3 == 0) goto L_0x0110
            r10 = 0
            int r3 = r3 + -1
            byte[] r1 = r11.next_in
            int r2 = r4 + 1
            byte r1 = r1[r4]
            r1 = r1 & 255(0xff, float:3.57E-43)
            int r1 = r1 << r5
            r6 = r6 | r1
            int r5 = r5 + 8
            r4 = r2
            goto L_0x00fb
        L_0x0110:
            r0.bitb = r6
            r0.bitk = r5
            r11.avail_in = r3
            long r1 = r11.total_in
            int r8 = r11.next_in_index
            int r8 = r4 - r8
            long r8 = (long) r8
            long r1 = r1 + r8
            r11.total_in = r1
            r11.next_in_index = r4
            r0.write = r14
            int r1 = r0.inflate_flush(r11, r10)
            return r1
        L_0x0129:
            int[] r1 = r0.blens
            int[] r2 = border
            int r8 = r0.index
            int r9 = r8 + 1
            r0.index = r9
            r2 = r2[r8]
            r8 = r6 & 7
            r1[r2] = r8
            int r6 = r6 >>> 3
            int r5 = r5 + -3
            r8 = 3
            goto L_0x00f1
        L_0x013f:
            int r1 = r0.index
            r2 = 19
            if (r1 >= r2) goto L_0x0154
            int[] r1 = r0.blens
            int[] r2 = border
            int r8 = r0.index
            int r9 = r8 + 1
            r0.index = r9
            r2 = r2[r8]
            r1[r2] = r13
            goto L_0x013f
        L_0x0154:
            int[] r1 = r0.bb
            r1[r13] = r12
            org.jboss.netty.util.internal.jzlib.InfTree r1 = r0.inftree
            int[] r2 = r0.blens
            int[] r8 = r0.bb
            int[] r9 = r0.tb
            int[] r12 = r0.hufts
            r13 = r3
            r3 = r8
            r8 = r4
            r4 = r9
            r9 = r5
            r5 = r12
            r12 = r6
            r6 = r39
            int r7 = r1.inflate_trees_bits(r2, r3, r4, r5, r6)
            if (r7 == 0) goto L_0x0195
            r1 = r7
            r6 = -3
            if (r1 != r6) goto L_0x017c
            r2 = 0
            r0.blens = r2
            r2 = 9
            r0.mode = r2
        L_0x017c:
            r0.bitb = r12
            r0.bitk = r9
            r11.avail_in = r13
            long r2 = r11.total_in
            int r4 = r11.next_in_index
            int r4 = r8 - r4
            long r4 = (long) r4
            long r2 = r2 + r4
            r11.total_in = r2
            r11.next_in_index = r8
            r0.write = r14
            int r2 = r0.inflate_flush(r11, r1)
            return r2
        L_0x0195:
            r6 = -3
            r1 = 0
            r0.index = r1
            r1 = 5
            r0.mode = r1
            r37 = r9
            r9 = r8
            r8 = r13
            r13 = r12
            r12 = r10
            r10 = r37
        L_0x01a4:
            int r1 = r0.table
            int r2 = r0.index
            r3 = r1 & 31
            int r3 = r3 + 258
            int r4 = r1 >> 5
            r4 = r4 & 31
            int r3 = r3 + r4
            r4 = -1
            if (r2 < r3) goto L_0x02e1
            int[] r2 = r0.tb
            r3 = 0
            r2[r3] = r4
            r2 = 1
            int[] r7 = new int[r2]
            int[] r5 = new int[r2]
            int[] r4 = new int[r2]
            r22 = r12
            int[] r12 = new int[r2]
            r17 = 9
            r7[r3] = r17
            r2 = 6
            r5[r3] = r2
            int r3 = r0.table
            org.jboss.netty.util.internal.jzlib.InfTree r1 = r0.inftree
            r2 = r3 & 31
            int r2 = r2 + 257
            int r18 = r3 >> 5
            r18 = r18 & 31
            r16 = 1
            int r18 = r18 + 1
            int[] r6 = r0.blens
            r24 = r9
            int[] r9 = r0.hufts
            r19 = r3
            r3 = r18
            r18 = r4
            r4 = r6
            r20 = r5
            r5 = r7
            r21 = -3
            r6 = r20
            r23 = r7
            r7 = r18
            r25 = r15
            r15 = r8
            r8 = r12
            r26 = r12
            r27 = r14
            r12 = r24
            r14 = -3
            r28 = r10
            r10 = r39
            int r1 = r1.inflate_trees_dynamic(r2, r3, r4, r5, r6, r7, r8, r9, r10)
            if (r1 == 0) goto L_0x0230
            if (r1 != r14) goto L_0x0212
            r2 = 0
            r0.blens = r2
            r6 = 9
            r0.mode = r6
        L_0x0212:
            r2 = r1
            r0.bitb = r13
            r9 = r28
            r0.bitk = r9
            r11.avail_in = r15
            long r3 = r11.total_in
            int r5 = r11.next_in_index
            int r5 = r12 - r5
            long r5 = (long) r5
            long r3 = r3 + r5
            r11.total_in = r3
            r11.next_in_index = r12
            r8 = r27
            r0.write = r8
            int r3 = r0.inflate_flush(r11, r2)
            return r3
        L_0x0230:
            r8 = r27
            r9 = r28
            org.jboss.netty.util.internal.jzlib.InfCodes r2 = r0.codes
            r3 = 0
            r28 = r23[r3]
            r29 = r20[r3]
            int[] r4 = r0.hufts
            r31 = r18[r3]
            int[] r5 = r0.hufts
            r33 = r26[r3]
            r27 = r2
            r30 = r4
            r32 = r5
            r27.init(r28, r29, r30, r31, r32, r33)
            r2 = 6
            r0.mode = r2
            r7 = r1
            r4 = r9
            r5 = r12
            r1 = r22
        L_0x0254:
            r0.bitb = r13
            r0.bitk = r4
            r11.avail_in = r15
            long r2 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r9 = (long) r6
            long r2 = r2 + r9
            r11.total_in = r2
            r11.next_in_index = r5
            r0.write = r8
            org.jboss.netty.util.internal.jzlib.InfCodes r2 = r0.codes
            int r2 = r2.proc(r0, r11, r1)
            r1 = r2
            r3 = 1
            if (r2 == r3) goto L_0x0277
            int r2 = r0.inflate_flush(r11, r1)
            return r2
        L_0x0277:
            r1 = 0
            int r5 = r11.next_in_index
            int r2 = r11.avail_in
            int r3 = r0.bitb
            int r4 = r0.bitk
            int r14 = r0.write
            int r6 = r0.read
            if (r14 >= r6) goto L_0x028c
            int r6 = r0.read
            int r6 = r6 - r14
            r8 = 1
            int r6 = r6 - r8
            goto L_0x028f
        L_0x028c:
            int r6 = r0.end
            int r6 = r6 - r14
        L_0x028f:
            int r8 = r0.last
            if (r8 != 0) goto L_0x0298
            r8 = 0
            r0.mode = r8
            goto L_0x0622
        L_0x0298:
            r10 = 7
            r0.mode = r10
            r15 = r6
        L_0x029c:
            r0.write = r14
            int r1 = r0.inflate_flush(r11, r1)
            int r14 = r0.write
            int r6 = r0.read
            int r8 = r0.write
            if (r6 == r8) goto L_0x02c3
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r8 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r8 = r8 + r12
            r11.total_in = r8
            r11.next_in_index = r5
            r0.write = r14
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x02c3:
            r6 = 8
            r0.mode = r6
        L_0x02c7:
            r1 = 1
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r8 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r8 = r8 + r12
            r11.total_in = r8
            r11.next_in_index = r5
            r0.write = r14
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x02e1:
            r22 = r12
            r25 = r15
            r6 = 9
            r15 = r8
            r12 = r9
            r9 = r10
            r8 = r14
            r10 = 7
            r14 = -3
            int[] r2 = r0.bb
            r3 = 0
            r1 = r2[r3]
            r2 = r22
        L_0x02f4:
            if (r9 >= r1) goto L_0x0322
            if (r15 == 0) goto L_0x0309
            r2 = 0
            int r15 = r15 + -1
            byte[] r3 = r11.next_in
            int r5 = r12 + 1
            byte r3 = r3[r12]
            r3 = r3 & 255(0xff, float:3.57E-43)
            int r3 = r3 << r9
            r13 = r13 | r3
            int r9 = r9 + 8
            r12 = r5
            goto L_0x02f4
        L_0x0309:
            r0.bitb = r13
            r0.bitk = r9
            r11.avail_in = r15
            long r3 = r11.total_in
            int r5 = r11.next_in_index
            int r5 = r12 - r5
            long r5 = (long) r5
            long r3 = r3 + r5
            r11.total_in = r3
            r11.next_in_index = r12
            r0.write = r8
            int r3 = r0.inflate_flush(r11, r2)
            return r3
        L_0x0322:
            int[] r3 = r0.tb
            r5 = 0
            r3 = r3[r5]
            int[] r3 = r0.hufts
            int[] r7 = r0.tb
            r7 = r7[r5]
            int[] r18 = inflate_mask
            r18 = r18[r1]
            r18 = r13 & r18
            int r7 = r7 + r18
            r18 = 3
            int r7 = r7 * 3
            r16 = 1
            int r7 = r7 + 1
            r1 = r3[r7]
            int[] r3 = r0.hufts
            int[] r7 = r0.tb
            r7 = r7[r5]
            int[] r5 = inflate_mask
            r5 = r5[r1]
            r5 = r5 & r13
            int r7 = r7 + r5
            int r7 = r7 * 3
            int r7 = r7 + 2
            r3 = r3[r7]
            r5 = 16
            if (r3 >= r5) goto L_0x0369
            int r4 = r13 >>> r1
            int r9 = r9 - r1
            int[] r5 = r0.blens
            int r7 = r0.index
            int r13 = r7 + 1
            r0.index = r13
            r5[r7] = r3
            r7 = r1
            r13 = r4
            r10 = r9
            r9 = r12
        L_0x0366:
            r12 = r2
            goto L_0x03f6
        L_0x0369:
            r5 = 18
            if (r3 != r5) goto L_0x036f
            r5 = 7
            goto L_0x0371
        L_0x036f:
            int r5 = r3 + -14
        L_0x0371:
            r7 = 18
            if (r3 != r7) goto L_0x0378
            r7 = 11
            goto L_0x0379
        L_0x0378:
            r7 = 3
        L_0x0379:
            int r10 = r1 + r5
            if (r9 >= r10) goto L_0x03af
            if (r15 == 0) goto L_0x0392
            r2 = 0
            int r15 = r15 + -1
            byte[] r10 = r11.next_in
            int r18 = r12 + 1
            byte r10 = r10[r12]
            r10 = r10 & 255(0xff, float:3.57E-43)
            int r10 = r10 << r9
            r13 = r13 | r10
            int r9 = r9 + 8
            r12 = r18
            r10 = 7
            goto L_0x0379
        L_0x0392:
            r0.bitb = r13
            r0.bitk = r9
            r11.avail_in = r15
            r34 = r15
            long r14 = r11.total_in
            int r4 = r11.next_in_index
            int r4 = r12 - r4
            r35 = r3
            long r3 = (long) r4
            long r14 = r14 + r3
            r11.total_in = r14
            r11.next_in_index = r12
            r0.write = r8
            int r3 = r0.inflate_flush(r11, r2)
            return r3
        L_0x03af:
            r35 = r3
            r34 = r15
            int r3 = r13 >>> r1
            int r9 = r9 - r1
            int[] r10 = inflate_mask
            r10 = r10[r5]
            r10 = r10 & r3
            int r7 = r7 + r10
            int r3 = r3 >>> r5
            int r9 = r9 - r5
            int r5 = r0.index
            int r1 = r0.table
            int r10 = r5 + r7
            r13 = r1 & 31
            int r13 = r13 + 258
            int r15 = r1 >> 5
            r15 = r15 & 31
            int r13 = r13 + r15
            if (r10 > r13) goto L_0x03fe
            r10 = r35
            r13 = 16
            if (r10 != r13) goto L_0x03d9
            r15 = 1
            if (r5 >= r15) goto L_0x03d9
            goto L_0x0400
        L_0x03d9:
            if (r10 != r13) goto L_0x03e2
            int[] r13 = r0.blens
            int r15 = r5 + -1
            r13 = r13[r15]
            goto L_0x03e3
        L_0x03e2:
            r13 = 0
        L_0x03e3:
            int[] r10 = r0.blens
            int r15 = r5 + 1
            r10[r5] = r13
            int r7 = r7 + r4
            if (r7 != 0) goto L_0x03fc
            r0.index = r15
            r7 = r1
            r13 = r3
            r10 = r9
            r9 = r12
            r15 = r34
            goto L_0x0366
        L_0x03f6:
            r14 = r8
            r8 = r15
            r15 = r25
            goto L_0x006c
        L_0x03fc:
            r5 = r15
            goto L_0x03e3
        L_0x03fe:
            r10 = r35
        L_0x0400:
            r4 = 0
            r0.blens = r4
            r0.mode = r6
            java.lang.String r4 = "invalid bit length repeat"
            r11.msg = r4
            r2 = -3
            r0.bitb = r3
            r0.bitk = r9
            r15 = r34
            r11.avail_in = r15
            long r13 = r11.total_in
            int r4 = r11.next_in_index
            int r4 = r12 - r4
            r36 = r3
            long r3 = (long) r4
            long r13 = r13 + r3
            r11.total_in = r13
            r11.next_in_index = r12
            r0.write = r8
            int r3 = r0.inflate_flush(r11, r2)
            return r3
        L_0x0427:
            r8 = r14
            r25 = r15
            r6 = 9
        L_0x042c:
            r0.mode = r6
            java.lang.String r6 = "too many length or distance symbols"
            r11.msg = r6
            r1 = -3
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r9 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r9 = r9 + r12
            r11.total_in = r9
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x044c:
            r8 = r14
            r25 = r15
            if (r2 != 0) goto L_0x046a
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r9 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r9 = r9 + r12
            r11.total_in = r9
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x046a:
            if (r25 != 0) goto L_0x04d2
            int r6 = r0.end
            if (r8 != r6) goto L_0x0484
            int r6 = r0.read
            if (r6 == 0) goto L_0x0484
            r14 = 0
            int r6 = r0.read
            if (r14 >= r6) goto L_0x047f
            int r6 = r0.read
            int r6 = r6 - r14
            r8 = 1
            int r6 = r6 - r8
            goto L_0x0482
        L_0x047f:
            int r6 = r0.end
            int r6 = r6 - r14
        L_0x0482:
            r15 = r6
            goto L_0x0487
        L_0x0484:
            r14 = r8
            r15 = r25
        L_0x0487:
            if (r15 != 0) goto L_0x04d5
            r0.write = r14
            int r1 = r0.inflate_flush(r11, r1)
            int r6 = r0.write
            int r8 = r0.read
            if (r6 >= r8) goto L_0x049b
            int r8 = r0.read
            int r8 = r8 - r6
            r9 = 1
            int r8 = r8 - r9
            goto L_0x049e
        L_0x049b:
            int r8 = r0.end
            int r8 = r8 - r6
        L_0x049e:
            int r9 = r0.end
            if (r6 != r9) goto L_0x04b5
            int r9 = r0.read
            if (r9 == 0) goto L_0x04b5
            r6 = 0
            int r9 = r0.read
            if (r6 >= r9) goto L_0x04b1
            int r9 = r0.read
            int r9 = r9 - r6
            r10 = 1
            int r9 = r9 - r10
            goto L_0x04b4
        L_0x04b1:
            int r9 = r0.end
            int r9 = r9 - r6
        L_0x04b4:
            r8 = r9
        L_0x04b5:
            r14 = r6
            r15 = r8
            if (r15 != 0) goto L_0x04d5
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r8 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r8 = r8 + r12
            r11.total_in = r8
            r11.next_in_index = r5
            r0.write = r14
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x04d2:
            r14 = r8
            r15 = r25
        L_0x04d5:
            r1 = 0
            int r6 = r0.left
            if (r6 <= r2) goto L_0x04db
            r6 = r2
        L_0x04db:
            if (r6 <= r15) goto L_0x04de
            r6 = r15
        L_0x04de:
            r7 = r6
            byte[] r6 = r11.next_in
            byte[] r8 = r0.window
            java.lang.System.arraycopy(r6, r5, r8, r14, r7)
            int r5 = r5 + r7
            int r2 = r2 - r7
            int r14 = r14 + r7
            int r6 = r15 - r7
            int r8 = r0.left
            int r8 = r8 - r7
            r0.left = r8
            if (r8 == 0) goto L_0x04f4
            goto L_0x0622
        L_0x04f4:
            int r8 = r0.last
            if (r8 == 0) goto L_0x04fa
            r8 = 7
            goto L_0x04fb
        L_0x04fa:
            r8 = 0
        L_0x04fb:
            r0.mode = r8
            goto L_0x0622
        L_0x04ff:
            r8 = r14
            r25 = r15
            r6 = 9
        L_0x0504:
            r9 = 32
            if (r4 >= r9) goto L_0x0534
            if (r2 == 0) goto L_0x051b
            r1 = 0
            int r2 = r2 + -1
            byte[] r9 = r11.next_in
            int r10 = r5 + 1
            byte r5 = r9[r5]
            r5 = r5 & 255(0xff, float:3.57E-43)
            int r5 = r5 << r4
            r3 = r3 | r5
            int r4 = r4 + 8
            r5 = r10
            goto L_0x0504
        L_0x051b:
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r9 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r9 = r9 + r12
            r11.total_in = r9
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x0534:
            r9 = r3 ^ -1
            r10 = 16
            int r9 = r9 >>> r10
            r10 = 65535(0xffff, float:9.1834E-41)
            r9 = r9 & r10
            r12 = r3 & r10
            if (r9 == r12) goto L_0x0561
            r0.mode = r6
            java.lang.String r6 = "invalid stored block lengths"
            r11.msg = r6
            r1 = -3
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r9 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r9 = r9 + r12
            r11.total_in = r9
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x0561:
            r6 = r3 & r10
            r0.left = r6
            r6 = 0
            r4 = r6
            r3 = r6
            int r6 = r0.left
            if (r6 == 0) goto L_0x056e
            r13 = 2
            goto L_0x0575
        L_0x056e:
            int r6 = r0.last
            if (r6 == 0) goto L_0x0574
            r13 = 7
            goto L_0x0575
        L_0x0574:
            r13 = 0
        L_0x0575:
            r0.mode = r13
            goto L_0x061f
        L_0x0579:
            r8 = r14
            r25 = r15
            r6 = 9
            r14 = -3
        L_0x057f:
            r7 = 3
            if (r4 >= r7) goto L_0x05ae
            if (r2 == 0) goto L_0x0595
            r1 = 0
            int r2 = r2 + -1
            byte[] r7 = r11.next_in
            int r9 = r5 + 1
            byte r5 = r7[r5]
            r5 = r5 & 255(0xff, float:3.57E-43)
            int r5 = r5 << r4
            r3 = r3 | r5
            int r4 = r4 + 8
            r5 = r9
            goto L_0x057f
        L_0x0595:
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r6 = r11.total_in
            int r9 = r11.next_in_index
            int r9 = r5 - r9
            long r9 = (long) r9
            long r6 = r6 + r9
            r11.total_in = r6
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x05ae:
            r7 = r3 & 7
            r9 = r7 & 1
            r0.last = r9
            int r9 = r7 >>> 1
            switch(r9) {
                case 0: goto L_0x0611;
                case 1: goto L_0x05e7;
                case 2: goto L_0x05df;
                case 3: goto L_0x05bc;
                default: goto L_0x05b9;
            }
        L_0x05b9:
            r9 = 1
            r14 = 0
            goto L_0x061f
        L_0x05bc:
            r9 = 3
            int r3 = r3 >>> r9
            int r4 = r4 + r14
            r0.mode = r6
            java.lang.String r6 = "invalid block type"
            r11.msg = r6
            r1 = -3
            r0.bitb = r3
            r0.bitk = r4
            r11.avail_in = r2
            long r9 = r11.total_in
            int r6 = r11.next_in_index
            int r6 = r5 - r6
            long r12 = (long) r6
            long r9 = r9 + r12
            r11.total_in = r9
            r11.next_in_index = r5
            r0.write = r8
            int r6 = r0.inflate_flush(r11, r1)
            return r6
        L_0x05df:
            int r3 = r3 >>> 3
            int r4 = r4 + -3
            r6 = 3
            r0.mode = r6
            goto L_0x05b9
        L_0x05e7:
            r6 = 1
            int[] r9 = new int[r6]
            int[] r10 = new int[r6]
            int[][] r12 = new int[r6][]
            int[][] r13 = new int[r6][]
            org.jboss.netty.util.internal.jzlib.InfTree.inflate_trees_fixed(r9, r10, r12, r13)
            org.jboss.netty.util.internal.jzlib.InfCodes r6 = r0.codes
            r14 = 0
            r27 = r9[r14]
            r28 = r10[r14]
            r29 = r12[r14]
            r30 = 0
            r31 = r13[r14]
            r32 = 0
            r26 = r6
            r26.init(r27, r28, r29, r30, r31, r32)
            int r3 = r3 >>> 3
            int r4 = r4 + -3
            r6 = 6
            r0.mode = r6
            r9 = 1
            goto L_0x061f
        L_0x0611:
            r14 = 0
            int r3 = r3 >>> 3
            int r4 = r4 + -3
            r6 = r4 & 7
            int r3 = r3 >>> r6
            int r4 = r4 - r6
            r9 = 1
            r0.mode = r9
            r7 = r6
        L_0x061f:
            r14 = r8
            r6 = r25
        L_0x0622:
            r12 = 1
            r13 = 0
            goto L_0x0021
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.InfBlocks.proc(org.jboss.netty.util.internal.jzlib.ZStream, int):int");
    }

    /* access modifiers changed from: package-private */
    public void free(ZStream z) {
        reset(z, (long[]) null);
        this.window = null;
        this.hufts = null;
    }

    /* access modifiers changed from: package-private */
    public void set_dictionary(byte[] d, int start, int n) {
        System.arraycopy(d, start, this.window, 0, n);
        this.write = n;
        this.read = n;
    }

    /* access modifiers changed from: package-private */
    public int sync_point() {
        return this.mode == 1 ? 1 : 0;
    }

    /* access modifiers changed from: package-private */
    public int inflate_flush(ZStream z, int r) {
        int n;
        int p = z.next_out_index;
        int q = this.read;
        int n2 = (q <= this.write ? this.write : this.end) - q;
        if (n2 > z.avail_out) {
            n2 = z.avail_out;
        }
        if (n2 != 0 && r == -5) {
            r = 0;
        }
        z.avail_out -= n2;
        z.total_out += (long) n2;
        if (this.checkfn != null) {
            long adler32 = Adler32.adler32(this.check, this.window, q, n2);
            this.check = adler32;
            z.adler = adler32;
        }
        System.arraycopy(this.window, q, z.next_out, p, n2);
        int p2 = p + n2;
        int q2 = q + n2;
        if (q2 == this.end) {
            if (this.write == this.end) {
                this.write = 0;
            }
            int n3 = this.write - 0;
            if (n3 > z.avail_out) {
                n = z.avail_out;
            } else {
                n = n3;
            }
            if (n != 0 && r == -5) {
                r = 0;
            }
            z.avail_out -= n;
            z.total_out += (long) n;
            if (this.checkfn != null) {
                long adler322 = Adler32.adler32(this.check, this.window, 0, n);
                this.check = adler322;
                z.adler = adler322;
            }
            System.arraycopy(this.window, 0, z.next_out, p2, n);
            p2 += n;
            q2 = 0 + n;
        }
        z.next_out_index = p2;
        this.read = q2;
        return r;
    }
}
