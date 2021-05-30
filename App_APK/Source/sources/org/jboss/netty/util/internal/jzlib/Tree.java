package org.jboss.netty.util.internal.jzlib;

import com.google.common.base.Ascii;
import org.bytedeco.javacpp.opencv_core;

final class Tree {
    static final byte[] _dist_code = {0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, 0, 0, 16, 17, Ascii.DC2, Ascii.DC2, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.FS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS, Ascii.GS};
    static final byte[] _length_code = {0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, 13, 13, 13, 13, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SO, Ascii.SI, Ascii.SI, Ascii.SI, Ascii.SI, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, Ascii.DC2, Ascii.DC2, Ascii.DC2, Ascii.DC2, Ascii.DC2, Ascii.DC2, Ascii.DC2, Ascii.DC2, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.SYN, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.ETB, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.CAN, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.EM, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.SUB, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.ESC, Ascii.FS};
    static final int[] base_dist = {0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048, 3072, 4096, 6144, 8192, opencv_core.CV_SEQ_KIND_MASK, 16384, 24576};
    static final int[] base_length = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 20, 24, 28, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 0};
    static final byte[] bl_order = {16, 17, Ascii.DC2, 0, 8, 7, 9, 6, 10, 5, 11, 4, Ascii.FF, 3, 13, 2, Ascii.SO, 1, Ascii.SI};
    static final int[] extra_blbits = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 7};
    static final int[] extra_dbits = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
    static final int[] extra_lbits = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    short[] dyn_tree;
    int max_code;
    StaticTree stat_desc;

    Tree() {
    }

    static int d_code(int dist) {
        return dist < 256 ? _dist_code[dist] : _dist_code[(dist >>> 7) + 256];
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r10v14, resolved type: short} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r10v19, resolved type: short} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r10v20, resolved type: short} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r10v21, resolved type: short} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r10v22, resolved type: short} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void gen_bitlen(org.jboss.netty.util.internal.jzlib.Deflate r20) {
        /*
            r19 = this;
            r0 = r19
            r1 = r20
            short[] r2 = r0.dyn_tree
            org.jboss.netty.util.internal.jzlib.StaticTree r3 = r0.stat_desc
            short[] r3 = r3.static_tree
            org.jboss.netty.util.internal.jzlib.StaticTree r4 = r0.stat_desc
            int[] r4 = r4.extra_bits
            org.jboss.netty.util.internal.jzlib.StaticTree r5 = r0.stat_desc
            int r5 = r5.extra_base
            org.jboss.netty.util.internal.jzlib.StaticTree r6 = r0.stat_desc
            int r6 = r6.max_length
            r7 = 0
            r8 = 0
            r9 = 0
        L_0x0019:
            r10 = 15
            if (r9 > r10) goto L_0x0024
            short[] r10 = r1.bl_count
            r10[r9] = r8
            int r9 = r9 + 1
            goto L_0x0019
        L_0x0024:
            int[] r10 = r1.heap
            int r11 = r1.heap_max
            r10 = r10[r11]
            int r10 = r10 * 2
            int r10 = r10 + 1
            r2[r10] = r8
            int r8 = r1.heap_max
        L_0x0032:
            int r8 = r8 + 1
            r10 = 573(0x23d, float:8.03E-43)
            if (r8 >= r10) goto L_0x008b
            int[] r10 = r1.heap
            r10 = r10[r8]
            int r11 = r10 * 2
            int r11 = r11 + 1
            short r11 = r2[r11]
            int r11 = r11 * 2
            int r11 = r11 + 1
            short r11 = r2[r11]
            int r11 = r11 + 1
            if (r11 <= r6) goto L_0x0050
            r9 = r6
            int r7 = r7 + 1
            goto L_0x0051
        L_0x0050:
            r9 = r11
        L_0x0051:
            int r11 = r10 * 2
            int r11 = r11 + 1
            short r12 = (short) r9
            r2[r11] = r12
            int r11 = r0.max_code
            if (r10 <= r11) goto L_0x005d
            goto L_0x008a
        L_0x005d:
            short[] r11 = r1.bl_count
            short r12 = r11[r9]
            int r12 = r12 + 1
            short r12 = (short) r12
            r11[r9] = r12
            r11 = 0
            if (r10 < r5) goto L_0x006d
            int r12 = r10 - r5
            r11 = r4[r12]
        L_0x006d:
            int r12 = r10 * 2
            short r12 = r2[r12]
            int r13 = r1.opt_len
            int r14 = r9 + r11
            int r14 = r14 * r12
            int r13 = r13 + r14
            r1.opt_len = r13
            if (r3 == 0) goto L_0x008a
            int r13 = r1.static_len
            int r14 = r10 * 2
            int r14 = r14 + 1
            short r14 = r3[r14]
            int r14 = r14 + r11
            int r14 = r14 * r12
            int r13 = r13 + r14
            r1.static_len = r13
        L_0x008a:
            goto L_0x0032
        L_0x008b:
            if (r7 != 0) goto L_0x008e
            return
        L_0x008e:
            int r9 = r6 + -1
        L_0x0090:
            short[] r10 = r1.bl_count
            short r10 = r10[r9]
            if (r10 != 0) goto L_0x0099
            int r9 = r9 + -1
            goto L_0x0090
        L_0x0099:
            short[] r10 = r1.bl_count
            short r11 = r10[r9]
            int r11 = r11 + -1
            short r11 = (short) r11
            r10[r9] = r11
            short[] r10 = r1.bl_count
            int r11 = r9 + 1
            short r12 = r10[r11]
            int r12 = r12 + 2
            short r12 = (short) r12
            r10[r11] = r12
            short[] r10 = r1.bl_count
            short r11 = r10[r6]
            int r11 = r11 + -1
            short r11 = (short) r11
            r10[r6] = r11
            int r7 = r7 + -2
            if (r7 > 0) goto L_0x0114
            r9 = r6
        L_0x00bb:
            if (r9 == 0) goto L_0x010f
            short[] r10 = r1.bl_count
            short r10 = r10[r9]
        L_0x00c1:
            if (r10 == 0) goto L_0x0106
            int[] r11 = r1.heap
            int r8 = r8 + -1
            r11 = r11[r8]
            int r12 = r0.max_code
            if (r11 <= r12) goto L_0x00ce
            goto L_0x00c1
        L_0x00ce:
            int r12 = r11 * 2
            int r12 = r12 + 1
            short r12 = r2[r12]
            if (r12 == r9) goto L_0x00f9
            int r12 = r1.opt_len
            long r12 = (long) r12
            long r14 = (long) r9
            int r16 = r11 * 2
            int r16 = r16 + 1
            short r0 = r2[r16]
            r17 = r3
            r18 = r4
            long r3 = (long) r0
            long r14 = r14 - r3
            int r0 = r11 * 2
            short r0 = r2[r0]
            long r3 = (long) r0
            long r14 = r14 * r3
            long r12 = r12 + r14
            int r0 = (int) r12
            r1.opt_len = r0
            int r0 = r11 * 2
            int r0 = r0 + 1
            short r3 = (short) r9
            r2[r0] = r3
            goto L_0x00fd
        L_0x00f9:
            r17 = r3
            r18 = r4
        L_0x00fd:
            int r10 = r10 + -1
            r3 = r17
            r4 = r18
            r0 = r19
            goto L_0x00c1
        L_0x0106:
            r17 = r3
            r18 = r4
            int r9 = r9 + -1
            r0 = r19
            goto L_0x00bb
        L_0x010f:
            r17 = r3
            r18 = r4
            return
        L_0x0114:
            r0 = r19
            goto L_0x008e
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.Tree.gen_bitlen(org.jboss.netty.util.internal.jzlib.Deflate):void");
    }

    /* access modifiers changed from: package-private */
    public void build_tree(Deflate s) {
        int max_code2;
        int max_code3;
        short[] tree = this.dyn_tree;
        short[] stree = this.stat_desc.static_tree;
        int elems = this.stat_desc.elems;
        s.heap_len = 0;
        s.heap_max = 573;
        int max_code4 = -1;
        for (int n = 0; n < elems; n++) {
            if (tree[n * 2] != 0) {
                int[] iArr = s.heap;
                int i = s.heap_len + 1;
                s.heap_len = i;
                max_code4 = n;
                iArr[i] = n;
                s.depth[n] = 0;
            } else {
                tree[(n * 2) + 1] = 0;
            }
        }
        while (s.heap_len < 2) {
            int[] iArr2 = s.heap;
            int i2 = s.heap_len + 1;
            s.heap_len = i2;
            if (max_code4 < 2) {
                max_code3 = max_code4 + 1;
                max_code2 = max_code3;
            } else {
                max_code2 = max_code4;
                max_code3 = 0;
            }
            iArr2[i2] = max_code3;
            tree[max_code3 * 2] = 1;
            s.depth[max_code3] = 0;
            s.opt_len--;
            if (stree != null) {
                s.static_len -= stree[(max_code3 * 2) + 1];
            }
            max_code4 = max_code2;
        }
        this.max_code = max_code4;
        for (int n2 = s.heap_len / 2; n2 >= 1; n2--) {
            s.pqdownheap(tree, n2);
        }
        int node = elems;
        while (true) {
            int n3 = s.heap[1];
            int[] iArr3 = s.heap;
            int[] iArr4 = s.heap;
            int i3 = s.heap_len;
            s.heap_len = i3 - 1;
            iArr3[1] = iArr4[i3];
            s.pqdownheap(tree, 1);
            int m = s.heap[1];
            int[] iArr5 = s.heap;
            int i4 = s.heap_max - 1;
            s.heap_max = i4;
            iArr5[i4] = n3;
            int[] iArr6 = s.heap;
            int i5 = s.heap_max - 1;
            s.heap_max = i5;
            iArr6[i5] = m;
            tree[node * 2] = (short) (tree[n3 * 2] + tree[m * 2]);
            s.depth[node] = (byte) (Math.max(s.depth[n3], s.depth[m]) + 1);
            short s2 = (short) node;
            tree[(m * 2) + 1] = s2;
            tree[(n3 * 2) + 1] = s2;
            int node2 = node + 1;
            s.heap[1] = node;
            s.pqdownheap(tree, 1);
            if (s.heap_len < 2) {
                int[] iArr7 = s.heap;
                int i6 = s.heap_max - 1;
                s.heap_max = i6;
                iArr7[i6] = s.heap[1];
                gen_bitlen(s);
                gen_codes(tree, max_code4, s.bl_count);
                return;
            }
            node = node2;
        }
    }

    private static void gen_codes(short[] tree, int max_code2, short[] bl_count) {
        short[] next_code = new short[16];
        short code = 0;
        for (int bits = 1; bits <= 15; bits++) {
            short s = (short) ((bl_count[bits - 1] + code) << 1);
            code = s;
            next_code[bits] = s;
        }
        for (int n = 0; n <= max_code2; n++) {
            short len = tree[(n * 2) + 1];
            if (len != 0) {
                short s2 = next_code[len];
                next_code[len] = (short) (s2 + 1);
                tree[n * 2] = (short) bi_reverse(s2, len);
            }
        }
    }

    private static int bi_reverse(int code, int len) {
        int res = 0;
        do {
            code >>>= 1;
            res = (res | (code & 1)) << 1;
            len--;
        } while (len > 0);
        return res >>> 1;
    }
}
