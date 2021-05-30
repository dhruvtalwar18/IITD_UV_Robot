package org.jboss.netty.util.internal.jzlib;

import org.jboss.netty.util.internal.jzlib.JZlib;

final class Inflate {
    private static final int BAD = 13;
    private static final int BLOCKS = 7;
    private static final int CHECK1 = 11;
    private static final int CHECK2 = 10;
    private static final int CHECK3 = 9;
    private static final int CHECK4 = 8;
    private static final int DICT0 = 6;
    private static final int DICT1 = 5;
    private static final int DICT2 = 4;
    private static final int DICT3 = 3;
    private static final int DICT4 = 2;
    private static final int DONE = 12;
    private static final int FLAG = 1;
    private static final int GZIP_CM = 16;
    private static final int GZIP_CRC32 = 24;
    private static final int GZIP_FCOMMENT = 22;
    private static final int GZIP_FEXTRA = 20;
    private static final int GZIP_FHCRC = 23;
    private static final int GZIP_FLG = 17;
    private static final int GZIP_FNAME = 21;
    private static final int GZIP_ID1 = 14;
    private static final int GZIP_ID2 = 15;
    private static final int GZIP_ISIZE = 25;
    private static final int GZIP_MTIME_XFL_OS = 18;
    private static final int GZIP_XLEN = 19;
    private static final int METHOD = 0;
    private static final byte[] mark = {0, 0, -1, -1};
    private InfBlocks blocks;
    private int gzipBytesToRead;
    private int gzipCRC32;
    private int gzipFlag;
    private int gzipISize;
    private int gzipUncompressedBytes;
    private int gzipXLen;
    private int marker;
    private int method;
    private int mode;
    private long need;
    private final long[] was = new long[1];
    private int wbits;
    private JZlib.WrapperType wrapperType;

    Inflate() {
    }

    private int inflateReset(ZStream z) {
        if (z == null || z.istate == null) {
            return -2;
        }
        z.total_out = 0;
        z.total_in = 0;
        z.msg = null;
        switch (this.wrapperType) {
            case NONE:
                z.istate.mode = 7;
                break;
            case ZLIB:
            case ZLIB_OR_NONE:
                z.istate.mode = 0;
                break;
            case GZIP:
                z.istate.mode = 14;
                break;
        }
        z.istate.blocks.reset(z, (long[]) null);
        this.gzipUncompressedBytes = 0;
        return 0;
    }

    /* access modifiers changed from: package-private */
    public int inflateEnd(ZStream z) {
        if (this.blocks != null) {
            this.blocks.free(z);
        }
        this.blocks = null;
        return 0;
    }

    /* access modifiers changed from: package-private */
    public int inflateInit(ZStream z, int w, JZlib.WrapperType wrapperType2) {
        Inflate inflate = null;
        z.msg = null;
        this.blocks = null;
        this.wrapperType = wrapperType2;
        if (w < 0) {
            throw new IllegalArgumentException("w: " + w);
        } else if (w < 8 || w > 15) {
            inflateEnd(z);
            return -2;
        } else {
            this.wbits = w;
            Inflate inflate2 = z.istate;
            if (z.istate.wrapperType != JZlib.WrapperType.NONE) {
                inflate = this;
            }
            inflate2.blocks = new InfBlocks(z, inflate, 1 << w);
            inflateReset(z);
            return 0;
        }
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Can't fix incorrect switch cases order */
    /* JADX WARNING: Code restructure failed: missing block: B:100:0x0299, code lost:
        if (r0[r3] != 0) goto L_0x027d;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:101:0x029b, code lost:
        r0 = 2;
        r1.gzipBytesToRead = 2;
        r2.istate.mode = 23;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:103:0x02a7, code lost:
        if ((r0 & r1.gzipFlag) == 0) goto L_0x02ca;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:105:0x02ab, code lost:
        if (r1.gzipBytesToRead <= 0) goto L_0x02ca;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:107:0x02af, code lost:
        if (r2.avail_in != 0) goto L_0x02b2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:108:0x02b1, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:109:0x02b2, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r2.next_in_index++;
        r1.gzipBytesToRead--;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:110:0x02ca, code lost:
        r2.istate.mode = 7;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:111:0x02d1, code lost:
        r2.istate.mode = 21;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:136:0x035f, code lost:
        if (r2.avail_in != 0) goto L_0x0362;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:137:0x0361, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:138:0x0362, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r3 = r2.istate;
        r4 = r2.next_in;
        r9 = r2.next_in_index;
        r2.next_in_index = r9 + 1;
        r3.need = ((long) ((r4[r9] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << com.google.common.base.Ascii.CAN)) & 4278190080L;
        r2.istate.mode = 9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:140:0x0392, code lost:
        if (r2.avail_in != 0) goto L_0x0395;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:141:0x0394, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:142:0x0395, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r3 = r2.istate;
        r9 = r3.need;
        r4 = r2.next_in;
        r12 = r2.next_in_index;
        r2.next_in_index = r12 + 1;
        r3.need = r9 + (((long) ((r4[r12] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << 16)) & 16711680);
        r2.istate.mode = 10;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:144:0x03c4, code lost:
        if (r2.avail_in != 0) goto L_0x03c7;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:145:0x03c6, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:146:0x03c7, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r3 = r2.istate;
        r9 = r3.need;
        r4 = r2.next_in;
        r12 = r2.next_in_index;
        r2.next_in_index = r12 + 1;
        r3.need = r9 + (((long) ((r4[r12] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << 8)) & 65280);
        r2.istate.mode = 11;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:148:0x03f5, code lost:
        if (r2.avail_in != 0) goto L_0x03f8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:149:0x03f7, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:150:0x03f8, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r3 = r2.istate;
        r9 = r3.need;
        r4 = r2.next_in;
        r12 = r2.next_in_index;
        r2.next_in_index = r12 + 1;
        r3.need = r9 + (((long) r4[r12]) & 255);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:151:0x0427, code lost:
        if (((int) r2.istate.was[0]) == ((int) r2.istate.need)) goto L_0x0437;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:152:0x0429, code lost:
        r2.istate.mode = 13;
        r2.msg = "incorrect data check";
        r2.istate.marker = 5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:153:0x0437, code lost:
        r2.istate.mode = 12;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:15:0x0034, code lost:
        if (r1.gzipBytesToRead <= 0) goto L_0x0068;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:17:0x0038, code lost:
        if (r2.avail_in != 0) goto L_0x003b;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:18:0x003a, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:19:0x003b, code lost:
        r6 = r5;
        r2.avail_in -= r15;
        r2.total_in++;
        r1.gzipBytesToRead -= r15;
        r9 = r2.istate;
        r10 = r9.gzipCRC32;
        r14 = r2.next_in;
        r4 = r2.next_in_index;
        r2.next_in_index = r4 + 1;
        r9.gzipCRC32 = ((r14[r4] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << ((3 - r1.gzipBytesToRead) * 8)) | r10;
        r15 = 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:21:0x006e, code lost:
        if (r2.crc32 == r2.istate.gzipCRC32) goto L_0x007e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:22:0x0070, code lost:
        r2.istate.mode = 13;
        r2.msg = "incorrect CRC32 checksum";
        r2.istate.marker = 5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:23:0x007e, code lost:
        r1.gzipBytesToRead = r3;
        r2.istate.mode = 25;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:25:0x0088, code lost:
        if (r1.gzipBytesToRead <= 0) goto L_0x00bc;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:268:?, code lost:
        return 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:27:0x008c, code lost:
        if (r2.avail_in != 0) goto L_0x008f;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:28:0x008e, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x008f, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r1.gzipBytesToRead--;
        r4 = r2.istate;
        r9 = r4.gzipISize;
        r10 = r2.next_in;
        r14 = r2.next_in_index;
        r2.next_in_index = r14 + 1;
        r4.gzipISize = r9 | ((r10[r14] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << ((3 - r1.gzipBytesToRead) * 8));
     */
    /* JADX WARNING: Code restructure failed: missing block: B:31:0x00c2, code lost:
        if (r1.gzipUncompressedBytes == r2.istate.gzipISize) goto L_0x00d2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:32:0x00c4, code lost:
        r2.istate.mode = 13;
        r2.msg = "incorrect ISIZE checksum";
        r2.istate.marker = 5;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:33:0x00d2, code lost:
        r2.istate.mode = 12;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:64:0x01b8, code lost:
        if (r1.gzipBytesToRead <= 0) goto L_0x01d7;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:66:0x01bc, code lost:
        if (r2.avail_in != 0) goto L_0x01bf;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:67:0x01be, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:68:0x01bf, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r2.next_in_index++;
        r1.gzipBytesToRead--;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:69:0x01d7, code lost:
        r2.istate.mode = 19;
        r1.gzipXLen = 0;
        r1.gzipBytesToRead = 2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:71:0x01e7, code lost:
        if ((r1.gzipFlag & 4) == 0) goto L_0x02d1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:73:0x01eb, code lost:
        if (r1.gzipBytesToRead <= 0) goto L_0x021e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:75:0x01ef, code lost:
        if (r2.avail_in != 0) goto L_0x01f2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:76:0x01f1, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:77:0x01f2, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r0 = r1.gzipXLen;
        r3 = r2.next_in;
        r4 = r2.next_in_index;
        r2.next_in_index = r4 + 1;
        r1.gzipXLen = r0 | ((r3[r4] & sensor_msgs.NavSatStatus.STATUS_NO_FIX) << ((1 - r1.gzipBytesToRead) * 8));
        r1.gzipBytesToRead--;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:78:0x021e, code lost:
        r1.gzipBytesToRead = r1.gzipXLen;
        r2.istate.mode = 20;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:80:0x022a, code lost:
        if (r1.gzipBytesToRead <= 0) goto L_0x0249;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:82:0x022e, code lost:
        if (r2.avail_in != 0) goto L_0x0231;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:83:0x0230, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:84:0x0231, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r2.next_in_index++;
        r1.gzipBytesToRead--;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:85:0x0249, code lost:
        r2.istate.mode = 21;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:87:0x0252, code lost:
        if ((r1.gzipFlag & 8) == 0) goto L_0x0272;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:89:0x0256, code lost:
        if (r2.avail_in != 0) goto L_0x0259;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:90:0x0258, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:91:0x0259, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r0 = r2.next_in;
        r3 = r2.next_in_index;
        r2.next_in_index = r3 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:92:0x0270, code lost:
        if (r0[r3] != 0) goto L_0x0254;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:93:0x0272, code lost:
        r2.istate.mode = 22;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:95:0x027b, code lost:
        if ((r1.gzipFlag & 16) == 0) goto L_0x029b;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:97:0x027f, code lost:
        if (r2.avail_in != 0) goto L_0x0282;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:98:0x0281, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:99:0x0282, code lost:
        r6 = r5;
        r2.avail_in--;
        r2.total_in++;
        r0 = r2.next_in;
        r3 = r2.next_in_index;
        r2.next_in_index = r3 + 1;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int inflate(org.jboss.netty.util.internal.jzlib.ZStream r21, int r22) {
        /*
            r20 = this;
            r1 = r20
            r2 = r21
            if (r2 == 0) goto L_0x061c
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            if (r3 == 0) goto L_0x061c
            byte[] r3 = r2.next_in
            if (r3 != 0) goto L_0x0010
            goto L_0x061c
        L_0x0010:
            r3 = 4
            r5 = r22
            if (r5 != r3) goto L_0x0017
            r6 = -5
            goto L_0x0018
        L_0x0017:
            r6 = 0
        L_0x0018:
            r5 = r6
            r6 = -5
            r7 = 0
            r8 = 0
        L_0x001c:
            org.jboss.netty.util.internal.jzlib.Inflate r9 = r2.istate
            int r9 = r9.mode
            r10 = 15
            r12 = 3
            r14 = 16
            r0 = 5
            r13 = 8
            r11 = 13
            r18 = 1
            r15 = 1
            switch(r9) {
                case 0: goto L_0x048c;
                case 1: goto L_0x0489;
                case 2: goto L_0x055a;
                case 3: goto L_0x058a;
                case 4: goto L_0x05ba;
                case 5: goto L_0x05e9;
                case 6: goto L_0x047a;
                case 7: goto L_0x02db;
                case 8: goto L_0x035d;
                case 9: goto L_0x0390;
                case 10: goto L_0x03c2;
                case 11: goto L_0x03f3;
                case 12: goto L_0x043d;
                case 13: goto L_0x02d9;
                case 14: goto L_0x00dd;
                case 15: goto L_0x0111;
                case 16: goto L_0x0145;
                case 17: goto L_0x0179;
                case 18: goto L_0x01b6;
                case 19: goto L_0x01e3;
                case 20: goto L_0x0228;
                case 21: goto L_0x024f;
                case 22: goto L_0x0278;
                case 23: goto L_0x00da;
                case 24: goto L_0x0032;
                case 25: goto L_0x0086;
                default: goto L_0x0030;
            }
        L_0x0030:
            r0 = -2
            return r0
        L_0x0032:
            int r9 = r1.gzipBytesToRead
            if (r9 <= 0) goto L_0x0068
            int r9 = r2.avail_in
            if (r9 != 0) goto L_0x003b
            return r6
        L_0x003b:
            r6 = r5
            int r9 = r2.avail_in
            int r9 = r9 - r15
            r2.avail_in = r9
            long r9 = r2.total_in
            long r9 = r9 + r18
            r2.total_in = r9
            int r9 = r1.gzipBytesToRead
            int r9 = r9 - r15
            r1.gzipBytesToRead = r9
            org.jboss.netty.util.internal.jzlib.Inflate r9 = r2.istate
            int r10 = r9.gzipCRC32
            byte[] r14 = r2.next_in
            int r4 = r2.next_in_index
            int r15 = r4 + 1
            r2.next_in_index = r15
            byte r4 = r14[r4]
            r4 = r4 & 255(0xff, float:3.57E-43)
            int r14 = r1.gzipBytesToRead
            int r14 = 3 - r14
            int r14 = r14 * 8
            int r4 = r4 << r14
            r4 = r4 | r10
            r9.gzipCRC32 = r4
            r15 = 1
            goto L_0x0032
        L_0x0068:
            int r4 = r2.crc32
            org.jboss.netty.util.internal.jzlib.Inflate r9 = r2.istate
            int r9 = r9.gzipCRC32
            if (r4 == r9) goto L_0x007e
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.mode = r11
            java.lang.String r4 = "incorrect CRC32 checksum"
            r2.msg = r4
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.marker = r0
            goto L_0x0551
        L_0x007e:
            r1.gzipBytesToRead = r3
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r9 = 25
            r4.mode = r9
        L_0x0086:
            int r4 = r1.gzipBytesToRead
            if (r4 <= 0) goto L_0x00bc
            int r4 = r2.avail_in
            if (r4 != 0) goto L_0x008f
            return r6
        L_0x008f:
            r6 = r5
            int r4 = r2.avail_in
            r9 = 1
            int r4 = r4 - r9
            r2.avail_in = r4
            long r14 = r2.total_in
            long r14 = r14 + r18
            r2.total_in = r14
            int r4 = r1.gzipBytesToRead
            int r4 = r4 - r9
            r1.gzipBytesToRead = r4
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            int r9 = r4.gzipISize
            byte[] r10 = r2.next_in
            int r14 = r2.next_in_index
            int r15 = r14 + 1
            r2.next_in_index = r15
            byte r10 = r10[r14]
            r10 = r10 & 255(0xff, float:3.57E-43)
            int r14 = r1.gzipBytesToRead
            int r14 = 3 - r14
            int r14 = r14 * 8
            int r10 = r10 << r14
            r9 = r9 | r10
            r4.gzipISize = r9
            goto L_0x0086
        L_0x00bc:
            int r4 = r1.gzipUncompressedBytes
            org.jboss.netty.util.internal.jzlib.Inflate r9 = r2.istate
            int r9 = r9.gzipISize
            if (r4 == r9) goto L_0x00d2
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.mode = r11
            java.lang.String r4 = "incorrect ISIZE checksum"
            r2.msg = r4
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.marker = r0
            goto L_0x0551
        L_0x00d2:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r4 = 12
            r0.mode = r4
            goto L_0x0551
        L_0x00da:
            r0 = 2
            goto L_0x02a4
        L_0x00dd:
            int r4 = r2.avail_in
            if (r4 != 0) goto L_0x00e2
            return r6
        L_0x00e2:
            r6 = r5
            int r4 = r2.avail_in
            r9 = 1
            int r4 = r4 - r9
            r2.avail_in = r4
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r3 = r2.next_in
            int r4 = r2.next_in_index
            int r9 = r4 + 1
            r2.next_in_index = r9
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            r4 = 31
            if (r3 == r4) goto L_0x010d
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "not a gzip stream"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x010d:
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r10
        L_0x0111:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x0116
            return r6
        L_0x0116:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r3 = r2.next_in
            int r4 = r2.next_in_index
            int r9 = r4 + 1
            r2.next_in_index = r9
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            r4 = 139(0x8b, float:1.95E-43)
            if (r3 == r4) goto L_0x0141
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "not a gzip stream"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x0141:
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r14
        L_0x0145:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x014a
            return r6
        L_0x014a:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r3 = r2.next_in
            int r4 = r2.next_in_index
            int r9 = r4 + 1
            r2.next_in_index = r9
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            if (r3 == r13) goto L_0x0173
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "unknown compression method"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x0173:
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 17
            r3.mode = r4
        L_0x0179:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x017e
            return r6
        L_0x017e:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r3 = r2.next_in
            int r4 = r2.next_in_index
            int r9 = r4 + 1
            r2.next_in_index = r9
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            r1.gzipFlag = r3
            int r3 = r1.gzipFlag
            r3 = r3 & 226(0xe2, float:3.17E-43)
            if (r3 == 0) goto L_0x01ad
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "unsupported flag"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x01ad:
            r0 = 6
            r1.gzipBytesToRead = r0
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 18
            r0.mode = r3
        L_0x01b6:
            int r0 = r1.gzipBytesToRead
            if (r0 <= 0) goto L_0x01d7
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x01bf
            return r6
        L_0x01bf:
            r6 = r5
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r9 = r2.total_in
            long r9 = r9 + r18
            r2.total_in = r9
            int r0 = r2.next_in_index
            int r0 = r0 + r3
            r2.next_in_index = r0
            int r0 = r1.gzipBytesToRead
            int r0 = r0 - r3
            r1.gzipBytesToRead = r0
            goto L_0x01b6
        L_0x01d7:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 19
            r0.mode = r3
            r0 = 0
            r1.gzipXLen = r0
            r0 = 2
            r1.gzipBytesToRead = r0
        L_0x01e3:
            int r0 = r1.gzipFlag
            r3 = 4
            r0 = r0 & r3
            if (r0 == 0) goto L_0x02d1
        L_0x01e9:
            int r0 = r1.gzipBytesToRead
            if (r0 <= 0) goto L_0x021e
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x01f2
            return r6
        L_0x01f2:
            r6 = r5
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            int r0 = r1.gzipXLen
            byte[] r3 = r2.next_in
            int r4 = r2.next_in_index
            int r9 = r4 + 1
            r2.next_in_index = r9
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            int r4 = r1.gzipBytesToRead
            r9 = 1
            int r15 = 1 - r4
            int r15 = r15 * 8
            int r3 = r3 << r15
            r0 = r0 | r3
            r1.gzipXLen = r0
            int r0 = r1.gzipBytesToRead
            int r0 = r0 - r9
            r1.gzipBytesToRead = r0
            goto L_0x01e9
        L_0x021e:
            int r0 = r1.gzipXLen
            r1.gzipBytesToRead = r0
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 20
            r0.mode = r3
        L_0x0228:
            int r0 = r1.gzipBytesToRead
            if (r0 <= 0) goto L_0x0249
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x0231
            return r6
        L_0x0231:
            r6 = r5
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r9 = r2.total_in
            long r9 = r9 + r18
            r2.total_in = r9
            int r0 = r2.next_in_index
            int r0 = r0 + r3
            r2.next_in_index = r0
            int r0 = r1.gzipBytesToRead
            int r0 = r0 - r3
            r1.gzipBytesToRead = r0
            goto L_0x0228
        L_0x0249:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 21
            r0.mode = r3
        L_0x024f:
            int r0 = r1.gzipFlag
            r0 = r0 & r13
            if (r0 == 0) goto L_0x0272
        L_0x0254:
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x0259
            return r6
        L_0x0259:
            r6 = r5
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r0 = r2.next_in
            int r3 = r2.next_in_index
            int r4 = r3 + 1
            r2.next_in_index = r4
            byte r0 = r0[r3]
            if (r0 != 0) goto L_0x0254
        L_0x0272:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 22
            r0.mode = r3
        L_0x0278:
            int r0 = r1.gzipFlag
            r0 = r0 & r14
            if (r0 == 0) goto L_0x029b
        L_0x027d:
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x0282
            return r6
        L_0x0282:
            r6 = r5
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r0 = r2.next_in
            int r3 = r2.next_in_index
            int r4 = r3 + 1
            r2.next_in_index = r4
            byte r0 = r0[r3]
            if (r0 != 0) goto L_0x027d
        L_0x029b:
            r0 = 2
            r1.gzipBytesToRead = r0
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 23
            r3.mode = r4
        L_0x02a4:
            int r3 = r1.gzipFlag
            r0 = r0 & r3
            if (r0 == 0) goto L_0x02ca
        L_0x02a9:
            int r0 = r1.gzipBytesToRead
            if (r0 <= 0) goto L_0x02ca
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x02b2
            return r6
        L_0x02b2:
            r6 = r5
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r9 = r2.total_in
            long r9 = r9 + r18
            r2.total_in = r9
            int r0 = r2.next_in_index
            int r0 = r0 + r3
            r2.next_in_index = r0
            int r0 = r1.gzipBytesToRead
            int r0 = r0 - r3
            r1.gzipBytesToRead = r0
            goto L_0x02a9
        L_0x02ca:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 7
            r0.mode = r3
            goto L_0x0551
        L_0x02d1:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 21
            r0.mode = r3
            goto L_0x0551
        L_0x02d9:
            r0 = -3
            return r0
        L_0x02db:
            int r3 = r2.next_out_index
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate     // Catch:{ all -> 0x0466 }
            org.jboss.netty.util.internal.jzlib.InfBlocks r4 = r4.blocks     // Catch:{ all -> 0x0466 }
            int r4 = r4.proc(r2, r6)     // Catch:{ all -> 0x0466 }
            r6 = r4
            r4 = -3
            if (r6 != r4) goto L_0x0308
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate     // Catch:{ all -> 0x0466 }
            r0.mode = r11     // Catch:{ all -> 0x0466 }
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate     // Catch:{ all -> 0x0466 }
            r4 = 0
            r0.marker = r4     // Catch:{ all -> 0x0466 }
            int r0 = r2.next_out_index
            int r0 = r0 - r3
            int r4 = r1.gzipUncompressedBytes
            int r4 = r4 + r0
            r1.gzipUncompressedBytes = r4
            int r4 = r2.crc32
            byte[] r8 = r2.next_out
            int r4 = org.jboss.netty.util.internal.jzlib.CRC32.crc32(r4, r8, r3, r0)
            r2.crc32 = r4
        L_0x0305:
            r8 = r3
            goto L_0x0551
        L_0x0308:
            if (r6 != 0) goto L_0x030b
            r6 = r5
        L_0x030b:
            r4 = 1
            if (r6 == r4) goto L_0x0322
            int r0 = r2.next_out_index
            int r0 = r0 - r3
            int r4 = r1.gzipUncompressedBytes
            int r4 = r4 + r0
            r1.gzipUncompressedBytes = r4
            int r4 = r2.crc32
            byte[] r8 = r2.next_out
            int r4 = org.jboss.netty.util.internal.jzlib.CRC32.crc32(r4, r8, r3, r0)
            r2.crc32 = r4
            return r6
        L_0x0322:
            r6 = r5
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate     // Catch:{ all -> 0x0466 }
            org.jboss.netty.util.internal.jzlib.InfBlocks r4 = r4.blocks     // Catch:{ all -> 0x0466 }
            org.jboss.netty.util.internal.jzlib.Inflate r8 = r2.istate     // Catch:{ all -> 0x0466 }
            long[] r8 = r8.was     // Catch:{ all -> 0x0466 }
            r4.reset(r2, r8)     // Catch:{ all -> 0x0466 }
            int r4 = r2.next_out_index
            int r4 = r4 - r3
            int r8 = r1.gzipUncompressedBytes
            int r8 = r8 + r4
            r1.gzipUncompressedBytes = r8
            int r8 = r2.crc32
            byte[] r9 = r2.next_out
            int r8 = org.jboss.netty.util.internal.jzlib.CRC32.crc32(r8, r9, r3, r4)
            r2.crc32 = r8
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r4 = r4.wrapperType
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r8 = org.jboss.netty.util.internal.jzlib.JZlib.WrapperType.NONE
            if (r4 != r8) goto L_0x0350
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r4 = 12
            r0.mode = r4
            goto L_0x0305
        L_0x0350:
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r4 = r4.wrapperType
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r8 = org.jboss.netty.util.internal.jzlib.JZlib.WrapperType.ZLIB
            if (r4 != r8) goto L_0x043f
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.mode = r13
            r8 = r3
        L_0x035d:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x0362
            return r6
        L_0x0362:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            byte[] r4 = r2.next_in
            int r9 = r2.next_in_index
            int r10 = r9 + 1
            r2.next_in_index = r10
            byte r4 = r4[r9]
            r4 = r4 & 255(0xff, float:3.57E-43)
            r9 = 24
            int r4 = r4 << r9
            long r9 = (long) r4
            r16 = 4278190080(0xff000000, double:2.113706745E-314)
            long r9 = r9 & r16
            r3.need = r9
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 9
            r3.mode = r4
        L_0x0390:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x0395
            return r6
        L_0x0395:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            long r9 = r3.need
            byte[] r4 = r2.next_in
            int r12 = r2.next_in_index
            int r15 = r12 + 1
            r2.next_in_index = r15
            byte r4 = r4[r12]
            r4 = r4 & 255(0xff, float:3.57E-43)
            int r4 = r4 << r14
            long r14 = (long) r4
            r16 = 16711680(0xff0000, double:8.256667E-317)
            long r14 = r14 & r16
            long r9 = r9 + r14
            r3.need = r9
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 10
            r3.mode = r4
        L_0x03c2:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x03c7
            return r6
        L_0x03c7:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            long r9 = r3.need
            byte[] r4 = r2.next_in
            int r12 = r2.next_in_index
            int r14 = r12 + 1
            r2.next_in_index = r14
            byte r4 = r4[r12]
            r4 = r4 & 255(0xff, float:3.57E-43)
            int r4 = r4 << r13
            long r12 = (long) r4
            r14 = 65280(0xff00, double:3.22526E-319)
            long r12 = r12 & r14
            long r9 = r9 + r12
            r3.need = r9
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 11
            r3.mode = r4
        L_0x03f3:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x03f8
            return r6
        L_0x03f8:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            long r9 = r3.need
            byte[] r4 = r2.next_in
            int r12 = r2.next_in_index
            int r13 = r12 + 1
            r2.next_in_index = r13
            byte r4 = r4[r12]
            long r12 = (long) r4
            r14 = 255(0xff, double:1.26E-321)
            long r12 = r12 & r14
            long r9 = r9 + r12
            r3.need = r9
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            long[] r3 = r3.was
            r4 = 0
            r9 = r3[r4]
            int r3 = (int) r9
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            long r9 = r4.need
            int r4 = (int) r9
            if (r3 == r4) goto L_0x0437
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "incorrect data check"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x0437:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 12
            r0.mode = r3
        L_0x043d:
            r0 = 1
            return r0
        L_0x043f:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r0 = r0.wrapperType
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r4 = org.jboss.netty.util.internal.jzlib.JZlib.WrapperType.GZIP
            if (r0 != r4) goto L_0x0457
            r0 = 0
            r1.gzipCRC32 = r0
            r1.gzipISize = r0
            r0 = 4
            r1.gzipBytesToRead = r0
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r4 = 24
            r0.mode = r4
            goto L_0x0305
        L_0x0457:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r0.mode = r11
            java.lang.String r0 = "unexpected state"
            r2.msg = r0
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r4 = 0
            r0.marker = r4
            goto L_0x0305
        L_0x0466:
            r0 = move-exception
            int r4 = r2.next_out_index
            int r4 = r4 - r3
            int r8 = r1.gzipUncompressedBytes
            int r8 = r8 + r4
            r1.gzipUncompressedBytes = r8
            int r8 = r2.crc32
            byte[] r9 = r2.next_out
            int r8 = org.jboss.netty.util.internal.jzlib.CRC32.crc32(r8, r9, r3, r4)
            r2.crc32 = r8
            throw r0
        L_0x047a:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r0.mode = r11
            java.lang.String r0 = "need dictionary"
            r2.msg = r0
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 0
            r0.marker = r3
            r0 = -2
            return r0
        L_0x0489:
            r4 = 1
            goto L_0x0512
        L_0x048c:
            r3 = 0
            int r4 = r2.avail_in
            if (r4 != 0) goto L_0x0492
            return r6
        L_0x0492:
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r4 = r4.wrapperType
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r9 = org.jboss.netty.util.internal.jzlib.JZlib.WrapperType.ZLIB_OR_NONE
            if (r4 != r9) goto L_0x04c7
            byte[] r4 = r2.next_in
            int r9 = r2.next_in_index
            byte r4 = r4[r9]
            r4 = r4 & r10
            if (r4 != r13) goto L_0x04ba
            byte[] r4 = r2.next_in
            int r9 = r2.next_in_index
            byte r4 = r4[r9]
            r9 = 4
            int r4 = r4 >> r9
            int r4 = r4 + r13
            org.jboss.netty.util.internal.jzlib.Inflate r9 = r2.istate
            int r9 = r9.wbits
            if (r4 <= r9) goto L_0x04b3
            goto L_0x04ba
        L_0x04b3:
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r9 = org.jboss.netty.util.internal.jzlib.JZlib.WrapperType.ZLIB
            r4.wrapperType = r9
            goto L_0x04c7
        L_0x04ba:
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            org.jboss.netty.util.internal.jzlib.JZlib$WrapperType r4 = org.jboss.netty.util.internal.jzlib.JZlib.WrapperType.NONE
            r0.wrapperType = r4
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r4 = 7
            r0.mode = r4
            goto L_0x0551
        L_0x04c7:
            r6 = r5
            int r4 = r2.avail_in
            r9 = 1
            int r4 = r4 - r9
            r2.avail_in = r4
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            byte[] r4 = r2.next_in
            int r9 = r2.next_in_index
            int r15 = r9 + 1
            r2.next_in_index = r15
            byte r4 = r4[r9]
            r3.method = r4
            r3 = r4 & 15
            if (r3 == r13) goto L_0x04f3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "unknown compression method"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x04f3:
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            int r3 = r3.method
            r4 = 4
            int r3 = r3 >> r4
            int r3 = r3 + r13
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            int r4 = r4.wbits
            if (r3 <= r4) goto L_0x050d
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r11
            java.lang.String r3 = "invalid window size"
            r2.msg = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.marker = r0
            goto L_0x0551
        L_0x050d:
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 1
            r3.mode = r4
        L_0x0512:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x0517
            return r6
        L_0x0517:
            r6 = r5
            int r3 = r2.avail_in
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            byte[] r3 = r2.next_in
            int r4 = r2.next_in_index
            int r7 = r4 + 1
            r2.next_in_index = r7
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            int r4 = r4.method
            int r4 = r4 << r13
            int r4 = r4 + r3
            int r4 = r4 % 31
            if (r4 == 0) goto L_0x0546
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.mode = r11
            java.lang.String r4 = "incorrect header check"
            r2.msg = r4
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r4.marker = r0
            goto L_0x0550
        L_0x0546:
            r4 = r3 & 32
            if (r4 != 0) goto L_0x0554
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r4 = 7
            r0.mode = r4
        L_0x0550:
            r7 = r3
        L_0x0551:
            r3 = 4
            goto L_0x001c
        L_0x0554:
            org.jboss.netty.util.internal.jzlib.Inflate r4 = r2.istate
            r7 = 2
            r4.mode = r7
            r7 = r3
        L_0x055a:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x055f
            return r6
        L_0x055f:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            byte[] r4 = r2.next_in
            int r8 = r2.next_in_index
            int r9 = r8 + 1
            r2.next_in_index = r9
            byte r4 = r4[r8]
            r4 = r4 & 255(0xff, float:3.57E-43)
            r8 = 24
            int r4 = r4 << r8
            long r8 = (long) r4
            r10 = 4278190080(0xff000000, double:2.113706745E-314)
            long r8 = r8 & r10
            r3.need = r8
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r12
        L_0x058a:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x058f
            return r6
        L_0x058f:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            long r8 = r3.need
            byte[] r4 = r2.next_in
            int r10 = r2.next_in_index
            int r11 = r10 + 1
            r2.next_in_index = r11
            byte r4 = r4[r10]
            r4 = r4 & 255(0xff, float:3.57E-43)
            int r4 = r4 << r14
            long r10 = (long) r4
            r14 = 16711680(0xff0000, double:8.256667E-317)
            long r10 = r10 & r14
            long r8 = r8 + r10
            r3.need = r8
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r4 = 4
            r3.mode = r4
        L_0x05ba:
            int r3 = r2.avail_in
            if (r3 != 0) goto L_0x05bf
            return r6
        L_0x05bf:
            r6 = r5
            int r3 = r2.avail_in
            r4 = 1
            int r3 = r3 - r4
            r2.avail_in = r3
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            long r8 = r3.need
            byte[] r4 = r2.next_in
            int r10 = r2.next_in_index
            int r11 = r10 + 1
            r2.next_in_index = r11
            byte r4 = r4[r10]
            r4 = r4 & 255(0xff, float:3.57E-43)
            int r4 = r4 << r13
            long r10 = (long) r4
            r12 = 65280(0xff00, double:3.22526E-319)
            long r10 = r10 & r12
            long r8 = r8 + r10
            r3.need = r8
            org.jboss.netty.util.internal.jzlib.Inflate r3 = r2.istate
            r3.mode = r0
        L_0x05e9:
            int r0 = r2.avail_in
            if (r0 != 0) goto L_0x05ee
            return r6
        L_0x05ee:
            int r0 = r2.avail_in
            r3 = 1
            int r0 = r0 - r3
            r2.avail_in = r0
            long r3 = r2.total_in
            long r3 = r3 + r18
            r2.total_in = r3
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            long r3 = r0.need
            byte[] r8 = r2.next_in
            int r9 = r2.next_in_index
            int r10 = r9 + 1
            r2.next_in_index = r10
            byte r8 = r8[r9]
            long r8 = (long) r8
            r10 = 255(0xff, double:1.26E-321)
            long r8 = r8 & r10
            long r3 = r3 + r8
            r0.need = r3
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            long r3 = r0.need
            r2.adler = r3
            org.jboss.netty.util.internal.jzlib.Inflate r0 = r2.istate
            r3 = 6
            r0.mode = r3
            r0 = 2
            return r0
        L_0x061c:
            r5 = r22
            r0 = -2
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.Inflate.inflate(org.jboss.netty.util.internal.jzlib.ZStream, int):int");
    }

    static int inflateSetDictionary(ZStream z, byte[] dictionary, int dictLength) {
        int index = 0;
        int length = dictLength;
        if (z == null || z.istate == null || z.istate.mode != 6) {
            return -2;
        }
        if (Adler32.adler32(1, dictionary, 0, dictLength) != z.adler) {
            return -3;
        }
        z.adler = Adler32.adler32(0, (byte[]) null, 0, 0);
        if (length >= (1 << z.istate.wbits)) {
            length = (1 << z.istate.wbits) - 1;
            index = dictLength - length;
        }
        z.istate.blocks.set_dictionary(dictionary, index, length);
        z.istate.mode = 7;
        return 0;
    }

    /* access modifiers changed from: package-private */
    public int inflateSync(ZStream z) {
        if (z == null || z.istate == null) {
            return -2;
        }
        if (z.istate.mode != 13) {
            z.istate.mode = 13;
            z.istate.marker = 0;
        }
        int i = z.avail_in;
        int n = i;
        if (i == 0) {
            return -5;
        }
        int p = z.next_in_index;
        int m = z.istate.marker;
        while (n != 0 && m < 4) {
            if (z.next_in[p] == mark[m]) {
                m++;
            } else if (z.next_in[p] != 0) {
                m = 0;
            } else {
                m = 4 - m;
            }
            p++;
            n--;
        }
        z.total_in += (long) (p - z.next_in_index);
        z.next_in_index = p;
        z.avail_in = n;
        z.istate.marker = m;
        if (m != 4) {
            return -3;
        }
        long r = z.total_in;
        long w = z.total_out;
        inflateReset(z);
        z.total_in = r;
        z.total_out = w;
        z.istate.mode = 7;
        return 0;
    }
}
