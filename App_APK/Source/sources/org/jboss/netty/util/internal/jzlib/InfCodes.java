package org.jboss.netty.util.internal.jzlib;

import javax.jmdns.impl.constants.DNSRecordClass;
import org.apache.commons.net.bsd.RCommandClient;
import sensor_msgs.NavSatStatus;

final class InfCodes {
    private static final int BADCODE = 9;
    private static final int COPY = 5;
    private static final int DIST = 3;
    private static final int DISTEXT = 4;
    private static final int END = 8;
    private static final int LEN = 1;
    private static final int LENEXT = 2;
    private static final int LIT = 6;
    private static final int START = 0;
    private static final int WASH = 7;
    private static final int[] inflate_mask = {0, 1, 3, 7, 15, 31, 63, 127, 255, 511, RCommandClient.MAX_CLIENT_PORT, 2047, 4095, 8191, 16383, DNSRecordClass.CLASS_MASK, 65535};
    private byte dbits;
    private int dist;
    private int[] dtree;
    private int dtree_index;
    private int get;
    private byte lbits;
    private int len;
    private int lit;
    private int[] ltree;
    private int ltree_index;
    private int mode;
    private int need;
    private int[] tree;
    private int tree_index;

    InfCodes() {
    }

    /* access modifiers changed from: package-private */
    public void init(int bl, int bd, int[] tl, int tl_index, int[] td, int td_index) {
        this.mode = 0;
        this.lbits = (byte) bl;
        this.dbits = (byte) bd;
        this.ltree = tl;
        this.ltree_index = tl_index;
        this.dtree = td;
        this.dtree_index = td_index;
        this.tree = null;
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Can't fix incorrect switch cases order */
    /* JADX WARNING: Code restructure failed: missing block: B:103:0x025e, code lost:
        r1 = r0.need;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:104:0x0260, code lost:
        if (r15 >= r1) goto L_0x028f;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:105:0x0262, code lost:
        if (r8 == 0) goto L_0x0276;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:106:0x0264, code lost:
        r13 = 0;
        r8 = r8 - 1;
        r14 = r14 | ((r10.next_in[r7] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << r15);
        r15 = r15 + 8;
        r7 = r7 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:107:0x0276, code lost:
        r9.bitb = r14;
        r9.bitk = r15;
        r10.avail_in = r8;
        r10.total_in += (long) (r7 - r10.next_in_index);
        r10.next_in_index = r7;
        r9.write = r5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:108:0x028e, code lost:
        return r9.inflate_flush(r10, r13);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:109:0x028f, code lost:
        r17 = (r0.tree_index + (r14 & inflate_mask[r1])) * 3;
        r14 = r14 >> r0.tree[r17 + 1];
        r15 = r15 - r0.tree[r17 + 1];
        r2 = r0.tree[r17];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:110:0x02af, code lost:
        if ((r2 & 16) == 0) goto L_0x02c1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:111:0x02b1, code lost:
        r0.get = r2 & 15;
        r0.dist = r0.tree[r17 + 2];
        r0.mode = 4;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:113:0x02c3, code lost:
        if ((r2 & 64) != 0) goto L_0x02d9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:114:0x02c5, code lost:
        r0.need = r2;
        r0.tree_index = (r17 / 3) + r0.tree[r17 + 2];
     */
    /* JADX WARNING: Code restructure failed: missing block: B:115:0x02d3, code lost:
        r16 = r1;
        r18 = r2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:116:0x02d9, code lost:
        r0.mode = 9;
        r10.msg = "invalid distance code";
        r9.bitb = r14;
        r9.bitk = r15;
        r10.avail_in = r8;
        r20 = r1;
        r21 = r2;
        r10.total_in += (long) (r7 - r10.next_in_index);
        r10.next_in_index = r7;
        r9.write = r5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:117:0x02fc, code lost:
        return r9.inflate_flush(r10, -3);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:154:0x0422, code lost:
        r11 = 1;
        r12 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:56:0x017a, code lost:
        r1 = r5 - r0.dist;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:57:0x017e, code lost:
        if (r1 >= 0) goto L_0x0184;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:58:0x0180, code lost:
        r1 = r1 + r9.end;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:59:0x0184, code lost:
        r19 = r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:61:0x0188, code lost:
        if (r0.len == 0) goto L_0x020b;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:62:0x018a, code lost:
        if (r6 != 0) goto L_0x01eb;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:64:0x018e, code lost:
        if (r5 != r9.end) goto L_0x01a2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:66:0x0192, code lost:
        if (r9.read == 0) goto L_0x01a2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:67:0x0194, code lost:
        r5 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:68:0x0197, code lost:
        if (0 >= r9.read) goto L_0x019e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:69:0x0199, code lost:
        r1 = (r9.read - 0) - r11;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:70:0x019e, code lost:
        r1 = r9.end - 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:71:0x01a1, code lost:
        r6 = r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:72:0x01a2, code lost:
        if (r6 != 0) goto L_0x01eb;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:73:0x01a4, code lost:
        r9.write = r5;
        r13 = r9.inflate_flush(r10, r13);
        r1 = r9.write;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:74:0x01ae, code lost:
        if (r1 >= r9.read) goto L_0x01b5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:75:0x01b0, code lost:
        r2 = (r9.read - r1) - r11;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:76:0x01b5, code lost:
        r2 = r9.end - r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:78:0x01ba, code lost:
        if (r1 != r9.end) goto L_0x01ce;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:80:0x01be, code lost:
        if (r9.read == 0) goto L_0x01ce;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:81:0x01c0, code lost:
        r1 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:82:0x01c3, code lost:
        if (0 >= r9.read) goto L_0x01ca;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:83:0x01c5, code lost:
        r3 = (r9.read - 0) - r11;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:84:0x01ca, code lost:
        r3 = r9.end - 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:85:0x01cd, code lost:
        r2 = r3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:86:0x01ce, code lost:
        r5 = r1;
        r6 = r2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:87:0x01d0, code lost:
        if (r6 != 0) goto L_0x01eb;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:88:0x01d2, code lost:
        r9.bitb = r14;
        r9.bitk = r15;
        r10.avail_in = r8;
        r10.total_in += (long) (r7 - r10.next_in_index);
        r10.next_in_index = r7;
        r9.write = r5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:89:0x01ea, code lost:
        return r9.inflate_flush(r10, r13);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:90:0x01eb, code lost:
        r2 = r5 + 1;
        r4 = r19 + 1;
        r9.window[r5] = r9.window[r19];
        r6 = r6 - 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:91:0x01fb, code lost:
        if (r4 != r9.end) goto L_0x0201;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:92:0x01fd, code lost:
        r19 = 0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:93:0x0201, code lost:
        r19 = r4;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:94:0x0203, code lost:
        r0.len -= r11;
        r5 = r2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:95:0x020b, code lost:
        r0.mode = r12;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int proc(org.jboss.netty.util.internal.jzlib.InfBlocks r29, org.jboss.netty.util.internal.jzlib.ZStream r30, int r31) {
        /*
            r28 = this;
            r0 = r28
            r9 = r29
            r10 = r30
            r1 = 0
            r2 = 0
            r3 = 0
            int r3 = r10.next_in_index
            int r4 = r10.avail_in
            int r1 = r9.bitb
            int r2 = r9.bitk
            int r5 = r9.write
            int r6 = r9.read
            r11 = 1
            if (r5 >= r6) goto L_0x001d
            int r6 = r9.read
            int r6 = r6 - r5
            int r6 = r6 - r11
            goto L_0x0020
        L_0x001d:
            int r6 = r9.end
            int r6 = r6 - r5
        L_0x0020:
            r12 = 0
            r13 = r31
            r14 = r1
            r15 = r2
            r7 = r3
            r8 = r4
            r16 = 0
            r17 = 0
            r18 = 0
            r19 = 0
        L_0x002f:
            int r1 = r0.mode
            r4 = 9
            r3 = 7
            r2 = 3
            switch(r1) {
                case 0: goto L_0x0304;
                case 1: goto L_0x02fd;
                case 2: goto L_0x020f;
                case 3: goto L_0x025e;
                case 4: goto L_0x0137;
                case 5: goto L_0x017a;
                case 6: goto L_0x00c4;
                case 7: goto L_0x0077;
                case 8: goto L_0x00aa;
                case 9: goto L_0x005d;
                default: goto L_0x0038;
            }
        L_0x0038:
            r12 = r5
            r11 = r7
            r24 = r8
            r22 = r13
            r23 = r15
            r13 = r6
            r1 = -2
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r4 = r10.next_in_index
            int r7 = r11 - r4
            long r4 = (long) r7
            long r2 = r2 + r4
            r10.total_in = r2
            r10.next_in_index = r11
            r9.write = r12
            int r2 = r9.inflate_flush(r10, r1)
            return r2
        L_0x005d:
            r1 = -3
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r4 = r10.next_in_index
            int r4 = r7 - r4
            long r11 = (long) r4
            long r2 = r2 + r11
            r10.total_in = r2
            r10.next_in_index = r7
            r9.write = r5
            int r2 = r9.inflate_flush(r10, r1)
            return r2
        L_0x0077:
            if (r15 <= r3) goto L_0x007f
            int r15 = r15 + -8
            int r8 = r8 + 1
            int r7 = r7 + -1
        L_0x007f:
            r9.write = r5
            int r13 = r9.inflate_flush(r10, r13)
            int r5 = r9.write
            int r1 = r9.read
            int r2 = r9.write
            if (r1 == r2) goto L_0x00a6
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r1 = r10.total_in
            int r3 = r10.next_in_index
            int r3 = r7 - r3
            long r3 = (long) r3
            long r1 = r1 + r3
            r10.total_in = r1
            r10.next_in_index = r7
            r9.write = r5
            int r1 = r9.inflate_flush(r10, r13)
            return r1
        L_0x00a6:
            r1 = 8
            r0.mode = r1
        L_0x00aa:
            r1 = 1
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r4 = r10.next_in_index
            int r4 = r7 - r4
            long r11 = (long) r4
            long r2 = r2 + r11
            r10.total_in = r2
            r10.next_in_index = r7
            r9.write = r5
            int r2 = r9.inflate_flush(r10, r1)
            return r2
        L_0x00c4:
            if (r6 != 0) goto L_0x0125
            int r1 = r9.end
            if (r5 != r1) goto L_0x00dc
            int r1 = r9.read
            if (r1 == 0) goto L_0x00dc
            r5 = 0
            int r1 = r9.read
            if (r5 >= r1) goto L_0x00d8
            int r1 = r9.read
            int r1 = r1 - r5
            int r1 = r1 - r11
            goto L_0x00db
        L_0x00d8:
            int r1 = r9.end
            int r1 = r1 - r5
        L_0x00db:
            r6 = r1
        L_0x00dc:
            if (r6 != 0) goto L_0x0125
            r9.write = r5
            int r13 = r9.inflate_flush(r10, r13)
            int r1 = r9.write
            int r2 = r9.read
            if (r1 >= r2) goto L_0x00ef
            int r2 = r9.read
            int r2 = r2 - r1
            int r2 = r2 - r11
            goto L_0x00f2
        L_0x00ef:
            int r2 = r9.end
            int r2 = r2 - r1
        L_0x00f2:
            int r3 = r9.end
            if (r1 != r3) goto L_0x0108
            int r3 = r9.read
            if (r3 == 0) goto L_0x0108
            r1 = 0
            int r3 = r9.read
            if (r1 >= r3) goto L_0x0104
            int r3 = r9.read
            int r3 = r3 - r1
            int r3 = r3 - r11
            goto L_0x0107
        L_0x0104:
            int r3 = r9.end
            int r3 = r3 - r1
        L_0x0107:
            r2 = r3
        L_0x0108:
            r5 = r1
            r6 = r2
            if (r6 != 0) goto L_0x0125
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r1 = r10.total_in
            int r3 = r10.next_in_index
            int r3 = r7 - r3
            long r3 = (long) r3
            long r1 = r1 + r3
            r10.total_in = r1
            r10.next_in_index = r7
            r9.write = r5
            int r1 = r9.inflate_flush(r10, r13)
            return r1
        L_0x0125:
            r13 = 0
            byte[] r1 = r9.window
            int r2 = r5 + 1
            int r3 = r0.lit
            byte r3 = (byte) r3
            r1[r5] = r3
            int r6 = r6 + -1
            r0.mode = r12
            r5 = r2
            goto L_0x002f
        L_0x0137:
            int r1 = r0.get
        L_0x0139:
            if (r15 >= r1) goto L_0x0167
            if (r8 == 0) goto L_0x014e
            r13 = 0
            int r8 = r8 + -1
            byte[] r2 = r10.next_in
            int r3 = r7 + 1
            byte r2 = r2[r7]
            r2 = r2 & 255(0xff, float:3.57E-43)
            int r2 = r2 << r15
            r14 = r14 | r2
            int r15 = r15 + 8
            r7 = r3
            goto L_0x0139
        L_0x014e:
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r4 = r10.next_in_index
            int r4 = r7 - r4
            long r11 = (long) r4
            long r2 = r2 + r11
            r10.total_in = r2
            r10.next_in_index = r7
            r9.write = r5
            int r2 = r9.inflate_flush(r10, r13)
            return r2
        L_0x0167:
            int r2 = r0.dist
            int[] r3 = inflate_mask
            r3 = r3[r1]
            r3 = r3 & r14
            int r2 = r2 + r3
            r0.dist = r2
            int r2 = r14 >> r1
            int r15 = r15 - r1
            r3 = 5
            r0.mode = r3
            r16 = r1
            r14 = r2
        L_0x017a:
            int r1 = r0.dist
            int r1 = r5 - r1
        L_0x017e:
            if (r1 >= 0) goto L_0x0184
            int r2 = r9.end
            int r1 = r1 + r2
            goto L_0x017e
        L_0x0184:
            r19 = r1
        L_0x0186:
            int r1 = r0.len
            if (r1 == 0) goto L_0x020b
            if (r6 != 0) goto L_0x01eb
            int r1 = r9.end
            if (r5 != r1) goto L_0x01a2
            int r1 = r9.read
            if (r1 == 0) goto L_0x01a2
            r5 = 0
            int r1 = r9.read
            if (r5 >= r1) goto L_0x019e
            int r1 = r9.read
            int r1 = r1 - r5
            int r1 = r1 - r11
            goto L_0x01a1
        L_0x019e:
            int r1 = r9.end
            int r1 = r1 - r5
        L_0x01a1:
            r6 = r1
        L_0x01a2:
            if (r6 != 0) goto L_0x01eb
            r9.write = r5
            int r13 = r9.inflate_flush(r10, r13)
            int r1 = r9.write
            int r2 = r9.read
            if (r1 >= r2) goto L_0x01b5
            int r2 = r9.read
            int r2 = r2 - r1
            int r2 = r2 - r11
            goto L_0x01b8
        L_0x01b5:
            int r2 = r9.end
            int r2 = r2 - r1
        L_0x01b8:
            int r3 = r9.end
            if (r1 != r3) goto L_0x01ce
            int r3 = r9.read
            if (r3 == 0) goto L_0x01ce
            r1 = 0
            int r3 = r9.read
            if (r1 >= r3) goto L_0x01ca
            int r3 = r9.read
            int r3 = r3 - r1
            int r3 = r3 - r11
            goto L_0x01cd
        L_0x01ca:
            int r3 = r9.end
            int r3 = r3 - r1
        L_0x01cd:
            r2 = r3
        L_0x01ce:
            r5 = r1
            r6 = r2
            if (r6 != 0) goto L_0x01eb
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r1 = r10.total_in
            int r3 = r10.next_in_index
            int r3 = r7 - r3
            long r3 = (long) r3
            long r1 = r1 + r3
            r10.total_in = r1
            r10.next_in_index = r7
            r9.write = r5
            int r1 = r9.inflate_flush(r10, r13)
            return r1
        L_0x01eb:
            byte[] r1 = r9.window
            int r2 = r5 + 1
            byte[] r3 = r9.window
            int r4 = r19 + 1
            byte r3 = r3[r19]
            r1[r5] = r3
            int r6 = r6 + -1
            int r1 = r9.end
            if (r4 != r1) goto L_0x0201
            r1 = 0
            r19 = r1
            goto L_0x0203
        L_0x0201:
            r19 = r4
        L_0x0203:
            int r1 = r0.len
            int r1 = r1 - r11
            r0.len = r1
            r5 = r2
            goto L_0x0186
        L_0x020b:
            r0.mode = r12
            goto L_0x002f
        L_0x020f:
            int r1 = r0.get
        L_0x0211:
            if (r15 >= r1) goto L_0x0240
            if (r8 == 0) goto L_0x0227
            r13 = 0
            int r8 = r8 + -1
            byte[] r3 = r10.next_in
            int r16 = r7 + 1
            byte r3 = r3[r7]
            r3 = r3 & 255(0xff, float:3.57E-43)
            int r3 = r3 << r15
            r14 = r14 | r3
            int r15 = r15 + 8
            r7 = r16
            goto L_0x0211
        L_0x0227:
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r4 = r10.next_in_index
            int r4 = r7 - r4
            long r11 = (long) r4
            long r2 = r2 + r11
            r10.total_in = r2
            r10.next_in_index = r7
            r9.write = r5
            int r2 = r9.inflate_flush(r10, r13)
            return r2
        L_0x0240:
            int r3 = r0.len
            int[] r16 = inflate_mask
            r16 = r16[r1]
            r16 = r14 & r16
            int r3 = r3 + r16
            r0.len = r3
            int r14 = r14 >> r1
            int r15 = r15 - r1
            byte r3 = r0.dbits
            r0.need = r3
            int[] r3 = r0.dtree
            r0.tree = r3
            int r3 = r0.dtree_index
            r0.tree_index = r3
            r0.mode = r2
            r16 = r1
        L_0x025e:
            int r1 = r0.need
        L_0x0260:
            if (r15 >= r1) goto L_0x028f
            if (r8 == 0) goto L_0x0276
            r13 = 0
            int r8 = r8 + -1
            byte[] r3 = r10.next_in
            int r16 = r7 + 1
            byte r3 = r3[r7]
            r3 = r3 & 255(0xff, float:3.57E-43)
            int r3 = r3 << r15
            r14 = r14 | r3
            int r15 = r15 + 8
            r7 = r16
            goto L_0x0260
        L_0x0276:
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r4 = r10.next_in_index
            int r4 = r7 - r4
            long r11 = (long) r4
            long r2 = r2 + r11
            r10.total_in = r2
            r10.next_in_index = r7
            r9.write = r5
            int r2 = r9.inflate_flush(r10, r13)
            return r2
        L_0x028f:
            int r3 = r0.tree_index
            int[] r16 = inflate_mask
            r16 = r16[r1]
            r16 = r14 & r16
            int r3 = r3 + r16
            int r17 = r3 * 3
            int[] r2 = r0.tree
            int r3 = r17 + 1
            r2 = r2[r3]
            int r14 = r14 >> r2
            int[] r2 = r0.tree
            int r3 = r17 + 1
            r2 = r2[r3]
            int r15 = r15 - r2
            int[] r2 = r0.tree
            r2 = r2[r17]
            r3 = r2 & 16
            if (r3 == 0) goto L_0x02c1
            r3 = r2 & 15
            r0.get = r3
            int[] r3 = r0.tree
            int r4 = r17 + 2
            r3 = r3[r4]
            r0.dist = r3
            r3 = 4
            r0.mode = r3
            goto L_0x02d3
        L_0x02c1:
            r3 = r2 & 64
            if (r3 != 0) goto L_0x02d9
            r0.need = r2
            int r3 = r17 / 3
            int[] r4 = r0.tree
            int r16 = r17 + 2
            r4 = r4[r16]
            int r3 = r3 + r4
            r0.tree_index = r3
        L_0x02d3:
            r16 = r1
            r18 = r2
            goto L_0x002f
        L_0x02d9:
            r0.mode = r4
            java.lang.String r3 = "invalid distance code"
            r10.msg = r3
            r3 = -3
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r11 = r10.total_in
            int r4 = r10.next_in_index
            int r4 = r7 - r4
            r20 = r1
            r21 = r2
            long r1 = (long) r4
            long r11 = r11 + r1
            r10.total_in = r11
            r10.next_in_index = r7
            r9.write = r5
            int r1 = r9.inflate_flush(r10, r3)
            return r1
        L_0x02fd:
            r2 = r15
            r1 = 1
            r15 = 7
            r20 = 3
            goto L_0x0391
        L_0x0304:
            r1 = 258(0x102, float:3.62E-43)
            if (r6 < r1) goto L_0x036d
            r1 = 10
            if (r8 < r1) goto L_0x036d
            r9.bitb = r14
            r9.bitk = r15
            r10.avail_in = r8
            long r2 = r10.total_in
            int r1 = r10.next_in_index
            int r1 = r7 - r1
            r22 = r13
            long r12 = (long) r1
            long r2 = r2 + r12
            r10.total_in = r2
            r10.next_in_index = r7
            r9.write = r5
            byte r1 = r0.lbits
            byte r2 = r0.dbits
            int[] r3 = r0.ltree
            int r12 = r0.ltree_index
            int[] r13 = r0.dtree
            int r11 = r0.dtree_index
            r20 = 3
            r23 = r15
            r15 = 7
            r4 = r12
            r12 = r5
            r5 = r13
            r13 = r6
            r6 = r11
            r11 = r7
            r7 = r29
            r24 = r8
            r8 = r30
            int r1 = inflate_fast(r1, r2, r3, r4, r5, r6, r7, r8)
            int r7 = r10.next_in_index
            int r8 = r10.avail_in
            int r14 = r9.bitb
            int r2 = r9.bitk
            int r5 = r9.write
            int r3 = r9.read
            if (r5 >= r3) goto L_0x0357
            int r3 = r9.read
            int r3 = r3 - r5
            r4 = 1
            int r3 = r3 - r4
            goto L_0x035b
        L_0x0357:
            r4 = 1
            int r3 = r9.end
            int r3 = r3 - r5
        L_0x035b:
            r6 = r3
            if (r1 == 0) goto L_0x036a
            if (r1 != r4) goto L_0x0361
            goto L_0x0363
        L_0x0361:
            r15 = 9
        L_0x0363:
            r0.mode = r15
            r13 = r1
            r15 = r2
            goto L_0x0422
        L_0x036a:
            r13 = r1
            r12 = r5
            goto L_0x0381
        L_0x036d:
            r12 = r5
            r11 = r7
            r24 = r8
            r22 = r13
            r23 = r15
            r15 = 7
            r20 = 3
            r13 = r6
            r7 = r11
            r6 = r13
            r13 = r22
            r2 = r23
            r8 = r24
        L_0x0381:
            byte r1 = r0.lbits
            r0.need = r1
            int[] r1 = r0.ltree
            r0.tree = r1
            int r1 = r0.ltree_index
            r0.tree_index = r1
            r1 = 1
            r0.mode = r1
            r5 = r12
        L_0x0391:
            int r3 = r0.need
        L_0x0393:
            if (r2 >= r3) goto L_0x03c3
            if (r8 == 0) goto L_0x03a8
            r13 = 0
            int r8 = r8 + -1
            byte[] r4 = r10.next_in
            int r11 = r7 + 1
            byte r4 = r4[r7]
            r4 = r4 & 255(0xff, float:3.57E-43)
            int r4 = r4 << r2
            r14 = r14 | r4
            int r2 = r2 + 8
            r7 = r11
            goto L_0x0393
        L_0x03a8:
            r9.bitb = r14
            r9.bitk = r2
            r10.avail_in = r8
            long r11 = r10.total_in
            int r1 = r10.next_in_index
            int r1 = r7 - r1
            r25 = r2
            long r1 = (long) r1
            long r11 = r11 + r1
            r10.total_in = r11
            r10.next_in_index = r7
            r9.write = r5
            int r1 = r9.inflate_flush(r10, r13)
            return r1
        L_0x03c3:
            r25 = r2
            int r2 = r0.tree_index
            int[] r4 = inflate_mask
            r4 = r4[r3]
            r4 = r4 & r14
            int r2 = r2 + r4
            int r17 = r2 * 3
            int[] r2 = r0.tree
            int r4 = r17 + 1
            r2 = r2[r4]
            int r14 = r14 >>> r2
            int[] r2 = r0.tree
            int r4 = r17 + 1
            r2 = r2[r4]
            int r2 = r25 - r2
            int[] r4 = r0.tree
            r4 = r4[r17]
            if (r4 != 0) goto L_0x03f0
            int[] r11 = r0.tree
            int r12 = r17 + 2
            r11 = r11[r12]
            r0.lit = r11
            r11 = 6
            r0.mode = r11
            goto L_0x041d
        L_0x03f0:
            r11 = r4 & 16
            if (r11 == 0) goto L_0x0404
            r11 = r4 & 15
            r0.get = r11
            int[] r11 = r0.tree
            int r12 = r17 + 2
            r11 = r11[r12]
            r0.len = r11
            r11 = 2
            r0.mode = r11
            goto L_0x041d
        L_0x0404:
            r11 = r4 & 64
            if (r11 != 0) goto L_0x0416
            r0.need = r4
            int r11 = r17 / 3
            int[] r12 = r0.tree
            int r15 = r17 + 2
            r12 = r12[r15]
            int r11 = r11 + r12
            r0.tree_index = r11
            goto L_0x041d
        L_0x0416:
            r11 = r4 & 32
            if (r11 == 0) goto L_0x0426
            r0.mode = r15
        L_0x041d:
            r15 = r2
            r16 = r3
            r18 = r4
        L_0x0422:
            r11 = 1
            r12 = 0
            goto L_0x002f
        L_0x0426:
            r1 = 9
            r0.mode = r1
            java.lang.String r1 = "invalid literal/length code"
            r10.msg = r1
            r1 = -3
            r9.bitb = r14
            r9.bitk = r2
            r10.avail_in = r8
            long r11 = r10.total_in
            int r13 = r10.next_in_index
            int r13 = r7 - r13
            r27 = r2
            r26 = r3
            long r2 = (long) r13
            long r11 = r11 + r2
            r10.total_in = r11
            r10.next_in_index = r7
            r9.write = r5
            int r2 = r9.inflate_flush(r10, r1)
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.InfCodes.proc(org.jboss.netty.util.internal.jzlib.InfBlocks, org.jboss.netty.util.internal.jzlib.ZStream, int):int");
    }

    static int inflate_fast(int bl, int bd, int[] tl, int tl_index, int[] td, int td_index, InfBlocks s, ZStream z) {
        int ml;
        int e;
        int r;
        int q;
        int m;
        int q2;
        int q3;
        InfBlocks infBlocks = s;
        ZStream zStream = z;
        int p = zStream.next_in_index;
        int n = zStream.avail_in;
        int b = infBlocks.bitb;
        int k = infBlocks.bitk;
        int q4 = infBlocks.write;
        int m2 = q4 < infBlocks.read ? (infBlocks.read - q4) - 1 : infBlocks.end - q4;
        int ml2 = inflate_mask[bl];
        int md = inflate_mask[bd];
        while (true) {
            if (k < 20) {
                n--;
                b |= (zStream.next_in[p] & 255) << k;
                k += 8;
                p++;
            } else {
                int t = b & ml2;
                int[] tp = tl;
                int tp_index = tl_index;
                int tp_index_t_3 = (tp_index + t) * 3;
                int i = tp[tp_index_t_3];
                int e2 = i;
                if (i == 0) {
                    int b2 = b >> tp[tp_index_t_3 + 1];
                    k -= tp[tp_index_t_3 + 1];
                    infBlocks.window[q4] = (byte) tp[tp_index_t_3 + 2];
                    m = m2 - 1;
                    ml = ml2;
                    q = q4 + 1;
                    e = e2;
                    b = b2;
                } else {
                    while (true) {
                        b >>= tp[tp_index_t_3 + 1];
                        k -= tp[tp_index_t_3 + 1];
                        if ((e2 & 16) != 0) {
                            int e3 = e2 & 15;
                            ml = ml2;
                            int c = tp[tp_index_t_3 + 2] + (b & inflate_mask[e3]);
                            int b3 = b >> e3;
                            int k2 = k - e3;
                            while (true) {
                                int e4 = e3;
                                if (k2 >= 15) {
                                    break;
                                }
                                n--;
                                b3 |= (zStream.next_in[p] & 255) << k2;
                                k2 += 8;
                                p++;
                                e3 = e4;
                            }
                            int t2 = b3 & md;
                            int[] tp2 = td;
                            int tp_index2 = td_index;
                            int tp_index_t_32 = (tp_index2 + t2) * 3;
                            int tp_index_t_33 = tp2[tp_index_t_32];
                            int i2 = tp_index_t_32;
                            int t3 = t2;
                            int e5 = tp_index_t_33;
                            int tp_index_t_34 = i2;
                            while (true) {
                                b3 >>= tp2[tp_index_t_34 + 1];
                                k2 -= tp2[tp_index_t_34 + 1];
                                if ((e5 & 16) != 0) {
                                    e = e5 & 15;
                                    int p2 = p;
                                    int n2 = n;
                                    while (k2 < e) {
                                        n2--;
                                        b3 |= (zStream.next_in[p2] & NavSatStatus.STATUS_NO_FIX) << k2;
                                        k2 += 8;
                                        p2++;
                                    }
                                    int d = tp2[tp_index_t_34 + 2] + (inflate_mask[e] & b3);
                                    int i3 = b3 >> e;
                                    int i4 = k2 - e;
                                    int i5 = m2 - c;
                                    if (q4 >= d) {
                                        int r2 = q4 - d;
                                        if (q4 - r2 <= 0 || 2 <= q4 - r2) {
                                            System.arraycopy(infBlocks.window, r2, infBlocks.window, q4, 2);
                                            q4 += 2;
                                            r = r2 + 2;
                                            c -= 2;
                                        } else {
                                            int q5 = q4 + 1;
                                            int r3 = r2 + 1;
                                            infBlocks.window[q4] = infBlocks.window[r2];
                                            q4 = q5 + 1;
                                            r = r3 + 1;
                                            infBlocks.window[q5] = infBlocks.window[r3];
                                            c -= 2;
                                        }
                                    } else {
                                        int r4 = q4 - d;
                                        do {
                                            r4 += infBlocks.end;
                                        } while (r4 < 0);
                                        int e6 = infBlocks.end - r4;
                                        if (c > e6) {
                                            c -= e6;
                                            if (q4 - r4 <= 0 || e6 <= q4 - r4) {
                                                System.arraycopy(infBlocks.window, r4, infBlocks.window, q4, e6);
                                                q4 += e6;
                                                int i6 = r4 + e6;
                                                e6 = 0;
                                            } else {
                                                while (true) {
                                                    q3 = q4 + 1;
                                                    int r5 = r4 + 1;
                                                    infBlocks.window[q4] = infBlocks.window[r4];
                                                    e6--;
                                                    if (e6 == 0) {
                                                        break;
                                                    }
                                                    q4 = q3;
                                                    r4 = r5;
                                                }
                                                q4 = q3;
                                            }
                                            r = 0;
                                        } else {
                                            r = r4;
                                        }
                                        e = e6;
                                    }
                                    if (q4 - r <= 0 || c <= q4 - r) {
                                        System.arraycopy(infBlocks.window, r, infBlocks.window, q4, c);
                                        q = q4 + c;
                                        int r6 = r + c;
                                    } else {
                                        while (true) {
                                            q2 = q4 + 1;
                                            int r7 = r + 1;
                                            infBlocks.window[q4] = infBlocks.window[r];
                                            c--;
                                            if (c == 0) {
                                                break;
                                            }
                                            q4 = q2;
                                            r = r7;
                                        }
                                        q = q2;
                                    }
                                    n = n2;
                                    p = p2;
                                    b = i3;
                                    k = i4;
                                    m = i5;
                                } else if (e5 == false || !true) {
                                    t3 = t3 + tp2[tp_index_t_34 + 2] + (b3 & inflate_mask[e5]);
                                    tp_index_t_34 = (tp_index2 + t3) * 3;
                                    e5 = tp2[tp_index_t_34];
                                } else {
                                    int i7 = e5;
                                    zStream.msg = "invalid distance code";
                                    int c2 = zStream.avail_in - n;
                                    int c3 = (k2 >> 3) < c2 ? k2 >> 3 : c2;
                                    int n3 = n + c3;
                                    int p3 = p - c3;
                                    infBlocks.bitb = b3;
                                    infBlocks.bitk = k2 - (c3 << 3);
                                    zStream.avail_in = n3;
                                    int i8 = n3;
                                    int i9 = b3;
                                    int i10 = c3;
                                    zStream.total_in += (long) (p3 - zStream.next_in_index);
                                    zStream.next_in_index = p3;
                                    infBlocks.write = q4;
                                    return -3;
                                }
                            }
                        } else {
                            ml = ml2;
                            if ((e2 & 64) == 0) {
                                t = t + tp[tp_index_t_3 + 2] + (inflate_mask[e2] & b);
                                tp_index_t_3 = (tp_index + t) * 3;
                                int i11 = tp[tp_index_t_3];
                                e2 = i11;
                                if (i11 == 0) {
                                    int b4 = b >> tp[tp_index_t_3 + 1];
                                    k -= tp[tp_index_t_3 + 1];
                                    infBlocks.window[q4] = (byte) tp[tp_index_t_3 + 2];
                                    m = m2 - 1;
                                    q = q4 + 1;
                                    e = e2;
                                    b = b4;
                                    break;
                                }
                                ml2 = ml;
                            } else if ((e2 & 32) != 0) {
                                int c4 = zStream.avail_in - n;
                                int c5 = (k >> 3) < c4 ? k >> 3 : c4;
                                int p4 = p - c5;
                                infBlocks.bitb = b;
                                infBlocks.bitk = k - (c5 << 3);
                                zStream.avail_in = n + c5;
                                int i12 = m2;
                                int i13 = c5;
                                zStream.total_in += (long) (p4 - zStream.next_in_index);
                                zStream.next_in_index = p4;
                                infBlocks.write = q4;
                                return 1;
                            } else {
                                zStream.msg = "invalid literal/length code";
                                int c6 = zStream.avail_in - n;
                                int c7 = (k >> 3) < c6 ? k >> 3 : c6;
                                int n4 = n + c7;
                                int p5 = p - c7;
                                infBlocks.bitb = b;
                                infBlocks.bitk = k - (c7 << 3);
                                zStream.avail_in = n4;
                                int i14 = n4;
                                int i15 = b;
                                zStream.total_in += (long) (p5 - zStream.next_in_index);
                                zStream.next_in_index = p5;
                                infBlocks.write = q4;
                                return -3;
                            }
                        }
                    }
                }
                if (m2 < 258 || n < 10) {
                    int c8 = zStream.avail_in - n;
                    int i16 = m2;
                } else {
                    ml2 = ml;
                }
            }
        }
        int c82 = zStream.avail_in - n;
        int i162 = m2;
        int c9 = (k >> 3) < c82 ? k >> 3 : c82;
        int n5 = n + c9;
        int p6 = p - c9;
        infBlocks.bitb = b;
        infBlocks.bitk = k - (c9 << 3);
        zStream.avail_in = n5;
        int i17 = n5;
        int i18 = b;
        int i19 = c9;
        int i20 = e;
        zStream.total_in += (long) (p6 - zStream.next_in_index);
        zStream.next_in_index = p6;
        infBlocks.write = q4;
        return 0;
    }
}
