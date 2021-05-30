package org.jboss.netty.util.internal.jzlib;

import android.support.v4.view.MotionEventCompat;
import org.jboss.netty.util.internal.jzlib.JZlib;
import sensor_msgs.NavSatStatus;

final class Deflate {
    private static final int BUSY_STATE = 113;
    private static final int BlockDone = 1;
    private static final int Buf_size = 16;
    private static final int DYN_TREES = 2;
    private static final int END_BLOCK = 256;
    private static final int FAST = 1;
    private static final int FINISH_STATE = 666;
    private static final int FinishDone = 3;
    private static final int FinishStarted = 2;
    private static final int INIT_STATE = 42;
    private static final int MAX_MATCH = 258;
    private static final int MIN_LOOKAHEAD = 262;
    private static final int MIN_MATCH = 3;
    private static final int NeedMore = 0;
    private static final int REPZ_11_138 = 18;
    private static final int REPZ_3_10 = 17;
    private static final int REP_3_6 = 16;
    private static final int SLOW = 2;
    private static final int STATIC_TREES = 1;
    private static final int STORED = 0;
    private static final int STORED_BLOCK = 0;
    private static final int Z_ASCII = 1;
    private static final int Z_BINARY = 0;
    private static final int Z_UNKNOWN = 2;
    private static final Config[] config_table = new Config[10];
    private static final String[] z_errmsg = {"need dictionary", "stream end", "", "file error", "stream error", "data error", "insufficient memory", "buffer error", "incompatible version", ""};
    short bi_buf;
    int bi_valid;
    short[] bl_count = new short[16];
    Tree bl_desc = new Tree();
    short[] bl_tree = new short[78];
    int block_start;
    int d_buf;
    Tree d_desc = new Tree();
    byte data_type;
    byte[] depth = new byte[573];
    short[] dyn_dtree = new short[122];
    short[] dyn_ltree = new short[1146];
    int good_match;
    private int gzipUncompressedBytes;
    int hash_bits;
    int hash_mask;
    int hash_shift;
    int hash_size;
    short[] head;
    int[] heap = new int[573];
    int heap_len;
    int heap_max;
    int ins_h;
    int l_buf;
    Tree l_desc = new Tree();
    int last_eob_len;
    int last_flush;
    int last_lit;
    int level;
    int lit_bufsize;
    int lookahead;
    int match_available;
    int match_length;
    int match_start;
    int matches;
    int max_chain_length;
    int max_lazy_match;
    int nice_match;
    int opt_len;
    int pending;
    byte[] pending_buf;
    int pending_buf_size;
    int pending_out;
    short[] prev;
    int prev_length;
    int prev_match;
    int static_len;
    int status;
    int strategy;
    ZStream strm;
    int strstart;
    int w_bits;
    int w_mask;
    int w_size;
    byte[] window;
    int window_size;
    JZlib.WrapperType wrapperType;
    private boolean wroteTrailer;

    private static final class Config {
        final int func;
        final int good_length;
        final int max_chain;
        final int max_lazy;
        final int nice_length;

        Config(int good_length2, int max_lazy2, int nice_length2, int max_chain2, int func2) {
            this.good_length = good_length2;
            this.max_lazy = max_lazy2;
            this.nice_length = nice_length2;
            this.max_chain = max_chain2;
            this.func = func2;
        }
    }

    static {
        config_table[0] = new Config(0, 0, 0, 0, 0);
        config_table[1] = new Config(4, 4, 8, 4, 1);
        config_table[2] = new Config(4, 5, 16, 8, 1);
        config_table[3] = new Config(4, 6, 32, 32, 1);
        config_table[4] = new Config(4, 4, 16, 16, 2);
        config_table[5] = new Config(8, 16, 32, 32, 2);
        config_table[6] = new Config(8, 16, 128, 128, 2);
        config_table[7] = new Config(8, 32, 128, 256, 2);
        config_table[8] = new Config(32, 128, 258, 1024, 2);
        config_table[9] = new Config(32, 258, 258, 4096, 2);
    }

    Deflate() {
    }

    private void lm_init() {
        this.window_size = this.w_size * 2;
        this.max_lazy_match = config_table[this.level].max_lazy;
        this.good_match = config_table[this.level].good_length;
        this.nice_match = config_table[this.level].nice_length;
        this.max_chain_length = config_table[this.level].max_chain;
        this.strstart = 0;
        this.block_start = 0;
        this.lookahead = 0;
        this.prev_length = 2;
        this.match_length = 2;
        this.match_available = 0;
        this.ins_h = 0;
    }

    private void tr_init() {
        this.l_desc.dyn_tree = this.dyn_ltree;
        this.l_desc.stat_desc = StaticTree.static_l_desc;
        this.d_desc.dyn_tree = this.dyn_dtree;
        this.d_desc.stat_desc = StaticTree.static_d_desc;
        this.bl_desc.dyn_tree = this.bl_tree;
        this.bl_desc.stat_desc = StaticTree.static_bl_desc;
        this.bi_buf = 0;
        this.bi_valid = 0;
        this.last_eob_len = 8;
        init_block();
    }

    private void init_block() {
        for (int i = 0; i < 286; i++) {
            this.dyn_ltree[i * 2] = 0;
        }
        for (int i2 = 0; i2 < 30; i2++) {
            this.dyn_dtree[i2 * 2] = 0;
        }
        for (int i3 = 0; i3 < 19; i3++) {
            this.bl_tree[i3 * 2] = 0;
        }
        this.dyn_ltree[512] = 1;
        this.static_len = 0;
        this.opt_len = 0;
        this.matches = 0;
        this.last_lit = 0;
    }

    /* access modifiers changed from: package-private */
    public void pqdownheap(short[] tree, int k) {
        int v = this.heap[k];
        int j = k << 1;
        while (j <= this.heap_len) {
            if (j < this.heap_len && smaller(tree, this.heap[j + 1], this.heap[j], this.depth)) {
                j++;
            }
            if (smaller(tree, v, this.heap[j], this.depth)) {
                break;
            }
            this.heap[k] = this.heap[j];
            k = j;
            j <<= 1;
        }
        this.heap[k] = v;
    }

    private static boolean smaller(short[] tree, int n, int m, byte[] depth2) {
        short tn2 = tree[n * 2];
        short tm2 = tree[m * 2];
        return tn2 < tm2 || (tn2 == tm2 && depth2[n] <= depth2[m]);
    }

    /* JADX WARNING: Incorrect type for immutable var: ssa=short, code=int, for r2v0, types: [short] */
    /* JADX WARNING: Incorrect type for immutable var: ssa=short, code=int, for r2v2, types: [short] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void scan_tree(short[] r12, int r13) {
        /*
            r11 = this;
            r0 = -1
            r1 = 1
            short r2 = r12[r1]
            r3 = 0
            r4 = 7
            r5 = 4
            if (r2 != 0) goto L_0x000c
            r4 = 138(0x8a, float:1.93E-43)
            r5 = 3
        L_0x000c:
            int r6 = r13 + 1
            int r6 = r6 * 2
            int r6 = r6 + r1
            r7 = -1
            r12[r6] = r7
            r6 = 0
        L_0x0015:
            if (r6 > r13) goto L_0x0077
            r7 = r2
            int r8 = r6 + 1
            int r8 = r8 * 2
            int r8 = r8 + r1
            short r2 = r12[r8]
            int r3 = r3 + 1
            if (r3 >= r4) goto L_0x0026
            if (r7 != r2) goto L_0x0026
            goto L_0x0074
        L_0x0026:
            if (r3 >= r5) goto L_0x0033
            short[] r8 = r11.bl_tree
            int r9 = r7 * 2
            short r10 = r8[r9]
            int r10 = r10 + r3
            short r10 = (short) r10
            r8[r9] = r10
            goto L_0x0065
        L_0x0033:
            if (r7 == 0) goto L_0x004c
            if (r7 == r0) goto L_0x0041
            short[] r8 = r11.bl_tree
            int r9 = r7 * 2
            short r10 = r8[r9]
            int r10 = r10 + r1
            short r10 = (short) r10
            r8[r9] = r10
        L_0x0041:
            short[] r8 = r11.bl_tree
            r9 = 32
            short r10 = r8[r9]
            int r10 = r10 + r1
            short r10 = (short) r10
            r8[r9] = r10
            goto L_0x0065
        L_0x004c:
            r8 = 10
            if (r3 > r8) goto L_0x005b
            short[] r8 = r11.bl_tree
            r9 = 34
            short r10 = r8[r9]
            int r10 = r10 + r1
            short r10 = (short) r10
            r8[r9] = r10
            goto L_0x0065
        L_0x005b:
            short[] r8 = r11.bl_tree
            r9 = 36
            short r10 = r8[r9]
            int r10 = r10 + r1
            short r10 = (short) r10
            r8[r9] = r10
        L_0x0065:
            r3 = 0
            r0 = r7
            if (r2 != 0) goto L_0x006d
            r4 = 138(0x8a, float:1.93E-43)
            r5 = 3
            goto L_0x0074
        L_0x006d:
            if (r7 != r2) goto L_0x0072
            r4 = 6
            r5 = 3
            goto L_0x0074
        L_0x0072:
            r4 = 7
            r5 = 4
        L_0x0074:
            int r6 = r6 + 1
            goto L_0x0015
        L_0x0077:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.Deflate.scan_tree(short[], int):void");
    }

    private int build_bl_tree() {
        scan_tree(this.dyn_ltree, this.l_desc.max_code);
        scan_tree(this.dyn_dtree, this.d_desc.max_code);
        this.bl_desc.build_tree(this);
        int max_blindex = 18;
        while (max_blindex >= 3 && this.bl_tree[(Tree.bl_order[max_blindex] * 2) + 1] == 0) {
            max_blindex--;
        }
        this.opt_len += ((max_blindex + 1) * 3) + 5 + 5 + 4;
        return max_blindex;
    }

    private void send_all_trees(int lcodes, int dcodes, int blcodes) {
        send_bits(lcodes - 257, 5);
        send_bits(dcodes - 1, 5);
        send_bits(blcodes - 4, 4);
        for (int rank = 0; rank < blcodes; rank++) {
            send_bits(this.bl_tree[(Tree.bl_order[rank] * 2) + 1], 3);
        }
        send_tree(this.dyn_ltree, lcodes - 1);
        send_tree(this.dyn_dtree, dcodes - 1);
    }

    /* JADX WARNING: Incorrect type for immutable var: ssa=short, code=int, for r2v0, types: [short] */
    /* JADX WARNING: Incorrect type for immutable var: ssa=short, code=int, for r2v2, types: [short] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void send_tree(short[] r12, int r13) {
        /*
            r11 = this;
            r0 = -1
            r1 = 1
            short r2 = r12[r1]
            r3 = 0
            r4 = 7
            r5 = 4
            if (r2 != 0) goto L_0x000c
            r4 = 138(0x8a, float:1.93E-43)
            r5 = 3
        L_0x000c:
            r6 = 0
        L_0x000d:
            if (r6 > r13) goto L_0x0074
            r7 = r2
            int r8 = r6 + 1
            r9 = 2
            int r8 = r8 * 2
            int r8 = r8 + r1
            short r2 = r12[r8]
            int r3 = r3 + 1
            if (r3 >= r4) goto L_0x001f
            if (r7 != r2) goto L_0x001f
            goto L_0x0071
        L_0x001f:
            if (r3 >= r5) goto L_0x002b
        L_0x0021:
            short[] r8 = r11.bl_tree
            r11.send_code(r7, r8)
            int r3 = r3 + -1
            if (r3 != 0) goto L_0x0021
            goto L_0x0062
        L_0x002b:
            if (r7 == 0) goto L_0x0043
            if (r7 == r0) goto L_0x0036
            short[] r8 = r11.bl_tree
            r11.send_code(r7, r8)
            int r3 = r3 + -1
        L_0x0036:
            r8 = 16
            short[] r10 = r11.bl_tree
            r11.send_code(r8, r10)
            int r8 = r3 + -3
            r11.send_bits(r8, r9)
            goto L_0x0062
        L_0x0043:
            r8 = 10
            if (r3 > r8) goto L_0x0055
            r8 = 17
            short[] r9 = r11.bl_tree
            r11.send_code(r8, r9)
            int r8 = r3 + -3
            r9 = 3
            r11.send_bits(r8, r9)
            goto L_0x0062
        L_0x0055:
            r8 = 18
            short[] r9 = r11.bl_tree
            r11.send_code(r8, r9)
            int r8 = r3 + -11
            r9 = 7
            r11.send_bits(r8, r9)
        L_0x0062:
            r3 = 0
            r0 = r7
            if (r2 != 0) goto L_0x006a
            r4 = 138(0x8a, float:1.93E-43)
            r5 = 3
            goto L_0x0071
        L_0x006a:
            if (r7 != r2) goto L_0x006f
            r4 = 6
            r5 = 3
            goto L_0x0071
        L_0x006f:
            r4 = 7
            r5 = 4
        L_0x0071:
            int r6 = r6 + 1
            goto L_0x000d
        L_0x0074:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.Deflate.send_tree(short[], int):void");
    }

    private void put_byte(byte[] p, int start, int len) {
        System.arraycopy(p, start, this.pending_buf, this.pending, len);
        this.pending += len;
    }

    private void put_byte(byte c) {
        byte[] bArr = this.pending_buf;
        int i = this.pending;
        this.pending = i + 1;
        bArr[i] = c;
    }

    private void put_short(int w) {
        put_byte((byte) w);
        put_byte((byte) (w >>> 8));
    }

    private void putShortMSB(int b) {
        put_byte((byte) (b >> 8));
        put_byte((byte) b);
    }

    private void send_code(int c, short[] tree) {
        int c2 = c * 2;
        send_bits(tree[c2] & 65535, 65535 & tree[c2 + 1]);
    }

    private void send_bits(int value, int length) {
        int len = length;
        if (this.bi_valid > 16 - len) {
            int val = value;
            this.bi_buf = (short) (this.bi_buf | (65535 & (val << this.bi_valid)));
            put_short(this.bi_buf);
            this.bi_buf = (short) (val >>> (16 - this.bi_valid));
            this.bi_valid += len - 16;
            return;
        }
        this.bi_buf = (short) (this.bi_buf | ((value << this.bi_valid) & 65535));
        this.bi_valid += len;
    }

    private void _tr_align() {
        send_bits(2, 3);
        send_code(256, StaticTree.static_ltree);
        bi_flush();
        if (((this.last_eob_len + 1) + 10) - this.bi_valid < 9) {
            send_bits(2, 3);
            send_code(256, StaticTree.static_ltree);
            bi_flush();
        }
        this.last_eob_len = 7;
    }

    private boolean _tr_tally(int dist, int lc) {
        int i = dist;
        int i2 = lc;
        this.pending_buf[this.d_buf + (this.last_lit * 2)] = (byte) (i >>> 8);
        int i3 = 1;
        this.pending_buf[this.d_buf + (this.last_lit * 2) + 1] = (byte) i;
        this.pending_buf[this.l_buf + this.last_lit] = (byte) i2;
        this.last_lit++;
        if (i == 0) {
            short[] sArr = this.dyn_ltree;
            int i4 = i2 * 2;
            sArr[i4] = (short) (sArr[i4] + 1);
        } else {
            this.matches++;
            short[] sArr2 = this.dyn_ltree;
            int i5 = (Tree._length_code[i2] + 256 + 1) * 2;
            sArr2[i5] = (short) (sArr2[i5] + 1);
            short[] sArr3 = this.dyn_dtree;
            int d_code = Tree.d_code(i - 1) * 2;
            sArr3[d_code] = (short) (sArr3[d_code] + 1);
        }
        if ((this.last_lit & 8191) == 0 && this.level > 2) {
            int in_length = this.strstart - this.block_start;
            int out_length = this.last_lit * 8;
            for (int dcode = 0; dcode < 30; dcode++) {
                out_length = (int) (((long) out_length) + (((long) this.dyn_dtree[dcode * 2]) * (((long) Tree.extra_dbits[dcode]) + 5)));
            }
            int out_length2 = out_length >>> 3;
            if (this.matches < this.last_lit / 2 && out_length2 < in_length / 2) {
                return true;
            }
            i3 = 1;
        }
        return this.last_lit == this.lit_bufsize - i3;
    }

    private void compress_block(short[] ltree, short[] dtree) {
        int lx = 0;
        if (this.last_lit != 0) {
            do {
                int dist = ((this.pending_buf[this.d_buf + (lx * 2)] << 8) & MotionEventCompat.ACTION_POINTER_INDEX_MASK) | (this.pending_buf[this.d_buf + (lx * 2) + 1] & 255);
                int lc = this.pending_buf[this.l_buf + lx] & 255;
                lx++;
                if (dist == 0) {
                    send_code(lc, ltree);
                } else {
                    byte code = Tree._length_code[lc];
                    send_code(code + 256 + 1, ltree);
                    int extra = Tree.extra_lbits[code];
                    if (extra != 0) {
                        send_bits(lc - Tree.base_length[code], extra);
                    }
                    int dist2 = dist - 1;
                    int code2 = Tree.d_code(dist2);
                    send_code(code2, dtree);
                    int extra2 = Tree.extra_dbits[code2];
                    if (extra2 != 0) {
                        send_bits(dist2 - Tree.base_dist[code2], extra2);
                    }
                }
            } while (lx < this.last_lit);
        }
        send_code(256, ltree);
        this.last_eob_len = ltree[513];
    }

    private void set_data_type() {
        int ascii_freq = 0;
        int i = 0;
        int n = 0;
        int bin_freq = 0;
        while (n < 7) {
            bin_freq += this.dyn_ltree[n * 2];
            n++;
        }
        while (n < 128) {
            ascii_freq += this.dyn_ltree[n * 2];
            n++;
        }
        while (n < 256) {
            bin_freq += this.dyn_ltree[n * 2];
            n++;
        }
        if (bin_freq <= (ascii_freq >>> 2)) {
            i = 1;
        }
        this.data_type = (byte) i;
    }

    private void bi_flush() {
        if (this.bi_valid == 16) {
            put_short(this.bi_buf);
            this.bi_buf = 0;
            this.bi_valid = 0;
        } else if (this.bi_valid >= 8) {
            put_byte((byte) this.bi_buf);
            this.bi_buf = (short) (this.bi_buf >>> 8);
            this.bi_valid -= 8;
        }
    }

    private void bi_windup() {
        if (this.bi_valid > 8) {
            put_short(this.bi_buf);
        } else if (this.bi_valid > 0) {
            put_byte((byte) this.bi_buf);
        }
        this.bi_buf = 0;
        this.bi_valid = 0;
    }

    private void copy_block(int buf, int len, boolean header) {
        bi_windup();
        this.last_eob_len = 8;
        if (header) {
            put_short((short) len);
            put_short((short) (len ^ -1));
        }
        put_byte(this.window, buf, len);
    }

    private void flush_block_only(boolean eof) {
        _tr_flush_block(this.block_start >= 0 ? this.block_start : -1, this.strstart - this.block_start, eof);
        this.block_start = this.strstart;
        this.strm.flush_pending();
    }

    private int deflate_stored(int flush) {
        int max_block_size = 65535;
        if (65535 > this.pending_buf_size - 5) {
            max_block_size = this.pending_buf_size - 5;
        }
        while (true) {
            if (this.lookahead <= 1) {
                fill_window();
                if (this.lookahead == 0 && flush == 0) {
                    return 0;
                }
                if (this.lookahead == 0) {
                    flush_block_only(flush == 4);
                    if (this.strm.avail_out == 0) {
                        if (flush == 4) {
                            return 2;
                        }
                        return 0;
                    } else if (flush == 4) {
                        return 3;
                    } else {
                        return 1;
                    }
                }
            }
            this.strstart += this.lookahead;
            this.lookahead = 0;
            int max_start = this.block_start + max_block_size;
            if (this.strstart == 0 || this.strstart >= max_start) {
                this.lookahead = this.strstart - max_start;
                this.strstart = max_start;
                flush_block_only(false);
                if (this.strm.avail_out == 0) {
                    return 0;
                }
            }
            if (this.strstart - this.block_start >= this.w_size - 262) {
                flush_block_only(false);
                if (this.strm.avail_out == 0) {
                    return 0;
                }
            }
        }
    }

    private void _tr_stored_block(int buf, int stored_len, boolean eof) {
        send_bits((eof) + false, 3);
        copy_block(buf, stored_len, true);
    }

    private void _tr_flush_block(int buf, int stored_len, boolean eof) {
        int static_lenb;
        int opt_lenb;
        int max_blindex = 0;
        if (this.level > 0) {
            if (this.data_type == 2) {
                set_data_type();
            }
            this.l_desc.build_tree(this);
            this.d_desc.build_tree(this);
            max_blindex = build_bl_tree();
            opt_lenb = ((this.opt_len + 3) + 7) >>> 3;
            static_lenb = ((this.static_len + 3) + 7) >>> 3;
            if (static_lenb <= opt_lenb) {
                opt_lenb = static_lenb;
            }
        } else {
            opt_lenb = stored_len + 5;
            static_lenb = opt_lenb;
        }
        if (stored_len + 4 <= opt_lenb && buf != -1) {
            _tr_stored_block(buf, stored_len, eof);
        } else if (static_lenb == opt_lenb) {
            send_bits(eof + true, 3);
            compress_block(StaticTree.static_ltree, StaticTree.static_dtree);
        } else {
            send_bits((eof) + true, 3);
            send_all_trees(this.l_desc.max_code + 1, this.d_desc.max_code + 1, max_blindex + 1);
            compress_block(this.dyn_ltree, this.dyn_dtree);
        }
        init_block();
        if (eof) {
            bi_windup();
        }
    }

    /* JADX WARNING: Removed duplicated region for block: B:0:0x0000 A[LOOP_START, MTH_ENTER_BLOCK] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void fill_window() {
        /*
            r10 = this;
        L_0x0000:
            int r0 = r10.window_size
            int r1 = r10.lookahead
            int r0 = r0 - r1
            int r1 = r10.strstart
            int r0 = r0 - r1
            r1 = 262(0x106, float:3.67E-43)
            if (r0 != 0) goto L_0x0018
            int r2 = r10.strstart
            if (r2 != 0) goto L_0x0018
            int r2 = r10.lookahead
            if (r2 != 0) goto L_0x0018
            int r0 = r10.w_size
            goto L_0x0086
        L_0x0018:
            r2 = -1
            if (r0 != r2) goto L_0x001e
            int r0 = r0 + -1
            goto L_0x0086
        L_0x001e:
            int r3 = r10.strstart
            int r4 = r10.w_size
            int r5 = r10.w_size
            int r4 = r4 + r5
            int r4 = r4 - r1
            if (r3 < r4) goto L_0x0086
            byte[] r3 = r10.window
            int r4 = r10.w_size
            byte[] r5 = r10.window
            int r6 = r10.w_size
            r7 = 0
            java.lang.System.arraycopy(r3, r4, r5, r7, r6)
            int r3 = r10.match_start
            int r4 = r10.w_size
            int r3 = r3 - r4
            r10.match_start = r3
            int r3 = r10.strstart
            int r4 = r10.w_size
            int r3 = r3 - r4
            r10.strstart = r3
            int r3 = r10.block_start
            int r4 = r10.w_size
            int r3 = r3 - r4
            r10.block_start = r3
            int r3 = r10.hash_size
            r4 = r3
        L_0x004c:
            short[] r5 = r10.head
            int r3 = r3 + r2
            short r5 = r5[r3]
            r6 = 65535(0xffff, float:9.1834E-41)
            r5 = r5 & r6
            short[] r8 = r10.head
            int r9 = r10.w_size
            if (r5 < r9) goto L_0x0061
            int r9 = r10.w_size
            int r9 = r5 - r9
            short r9 = (short) r9
            goto L_0x0062
        L_0x0061:
            r9 = 0
        L_0x0062:
            r8[r3] = r9
            int r4 = r4 + r2
            if (r4 != 0) goto L_0x004c
            int r4 = r10.w_size
            r3 = r4
        L_0x006a:
            short[] r8 = r10.prev
            int r3 = r3 + r2
            short r8 = r8[r3]
            r5 = r8 & r6
            short[] r8 = r10.prev
            int r9 = r10.w_size
            if (r5 < r9) goto L_0x007d
            int r9 = r10.w_size
            int r9 = r5 - r9
            short r9 = (short) r9
            goto L_0x007e
        L_0x007d:
            r9 = 0
        L_0x007e:
            r8[r3] = r9
            int r4 = r4 + r2
            if (r4 != 0) goto L_0x006a
            int r2 = r10.w_size
            int r0 = r0 + r2
        L_0x0086:
            org.jboss.netty.util.internal.jzlib.ZStream r2 = r10.strm
            int r2 = r2.avail_in
            if (r2 != 0) goto L_0x008d
            return
        L_0x008d:
            org.jboss.netty.util.internal.jzlib.ZStream r2 = r10.strm
            byte[] r3 = r10.window
            int r4 = r10.strstart
            int r5 = r10.lookahead
            int r4 = r4 + r5
            int r2 = r2.read_buf(r3, r4, r0)
            int r3 = r10.lookahead
            int r3 = r3 + r2
            r10.lookahead = r3
            int r3 = r10.lookahead
            r4 = 3
            if (r3 < r4) goto L_0x00c3
            byte[] r3 = r10.window
            int r4 = r10.strstart
            byte r3 = r3[r4]
            r3 = r3 & 255(0xff, float:3.57E-43)
            r10.ins_h = r3
            int r3 = r10.ins_h
            int r4 = r10.hash_shift
            int r3 = r3 << r4
            byte[] r4 = r10.window
            int r5 = r10.strstart
            int r5 = r5 + 1
            byte r4 = r4[r5]
            r4 = r4 & 255(0xff, float:3.57E-43)
            r3 = r3 ^ r4
            int r4 = r10.hash_mask
            r3 = r3 & r4
            r10.ins_h = r3
        L_0x00c3:
            int r3 = r10.lookahead
            if (r3 >= r1) goto L_0x00cd
            org.jboss.netty.util.internal.jzlib.ZStream r1 = r10.strm
            int r1 = r1.avail_in
            if (r1 != 0) goto L_0x0000
        L_0x00cd:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.util.internal.jzlib.Deflate.fill_window():void");
    }

    private int deflate_fast(int flush) {
        boolean bflush;
        int i;
        int hash_head = 0;
        while (true) {
            if (this.lookahead < 262) {
                fill_window();
                if (this.lookahead < 262 && flush == 0) {
                    return 0;
                }
                if (this.lookahead == 0) {
                    flush_block_only(flush == 4);
                    if (this.strm.avail_out == 0) {
                        if (flush == 4) {
                            return 2;
                        }
                        return 0;
                    } else if (flush == 4) {
                        return 3;
                    } else {
                        return 1;
                    }
                }
            }
            if (this.lookahead >= 3) {
                this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[(this.strstart + 3) - 1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
                hash_head = this.head[this.ins_h] & 65535;
                this.prev[this.strstart & this.w_mask] = this.head[this.ins_h];
                this.head[this.ins_h] = (short) this.strstart;
            }
            if (!(((long) hash_head) == 0 || ((this.strstart - hash_head) & 65535) > this.w_size - 262 || this.strategy == 2)) {
                this.match_length = longest_match(hash_head);
            }
            if (this.match_length >= 3) {
                bflush = _tr_tally(this.strstart - this.match_start, this.match_length - 3);
                this.lookahead -= this.match_length;
                if (this.match_length > this.max_lazy_match || this.lookahead < 3) {
                    this.strstart += this.match_length;
                    this.match_length = 0;
                    this.ins_h = this.window[this.strstart] & NavSatStatus.STATUS_NO_FIX;
                    this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[this.strstart + 1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
                } else {
                    this.match_length--;
                    do {
                        this.strstart++;
                        this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[(this.strstart + 3) - 1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
                        hash_head = this.head[this.ins_h] & 65535;
                        this.prev[this.strstart & this.w_mask] = this.head[this.ins_h];
                        this.head[this.ins_h] = (short) this.strstart;
                        i = this.match_length - 1;
                        this.match_length = i;
                    } while (i != 0);
                    this.strstart++;
                }
            } else {
                bflush = _tr_tally(0, this.window[this.strstart] & NavSatStatus.STATUS_NO_FIX);
                this.lookahead--;
                this.strstart++;
            }
            if (bflush) {
                flush_block_only(false);
                if (this.strm.avail_out == 0) {
                    return 0;
                }
            }
        }
    }

    private int deflate_slow(int flush) {
        int i;
        int hash_head = 0;
        while (true) {
            if (this.lookahead < 262) {
                fill_window();
                if (this.lookahead < 262 && flush == 0) {
                    return 0;
                }
                if (this.lookahead == 0) {
                    if (this.match_available != 0) {
                        _tr_tally(0, this.window[this.strstart - 1] & NavSatStatus.STATUS_NO_FIX);
                        this.match_available = 0;
                    }
                    flush_block_only(flush == 4);
                    if (this.strm.avail_out == 0) {
                        if (flush == 4) {
                            return 2;
                        }
                        return 0;
                    } else if (flush == 4) {
                        return 3;
                    } else {
                        return 1;
                    }
                }
            }
            if (this.lookahead >= 3) {
                this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[(this.strstart + 3) - 1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
                hash_head = this.head[this.ins_h] & 65535;
                this.prev[this.strstart & this.w_mask] = this.head[this.ins_h];
                this.head[this.ins_h] = (short) this.strstart;
            }
            this.prev_length = this.match_length;
            this.prev_match = this.match_start;
            this.match_length = 2;
            if (hash_head != 0 && this.prev_length < this.max_lazy_match && ((this.strstart - hash_head) & 65535) <= this.w_size - 262) {
                if (this.strategy != 2) {
                    this.match_length = longest_match(hash_head);
                }
                if (this.match_length <= 5 && (this.strategy == 1 || (this.match_length == 3 && this.strstart - this.match_start > 4096))) {
                    this.match_length = 2;
                }
            }
            if (this.prev_length >= 3 && this.match_length <= this.prev_length) {
                int max_insert = (this.strstart + this.lookahead) - 3;
                boolean bflush = _tr_tally((this.strstart - 1) - this.prev_match, this.prev_length - 3);
                this.lookahead -= this.prev_length - 1;
                this.prev_length -= 2;
                do {
                    int i2 = this.strstart + 1;
                    this.strstart = i2;
                    if (i2 <= max_insert) {
                        this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[(this.strstart + 3) - 1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
                        hash_head = this.head[this.ins_h] & 65535;
                        this.prev[this.strstart & this.w_mask] = this.head[this.ins_h];
                        this.head[this.ins_h] = (short) this.strstart;
                    }
                    i = this.prev_length - 1;
                    this.prev_length = i;
                } while (i != 0);
                this.match_available = 0;
                this.match_length = 2;
                this.strstart++;
                if (bflush) {
                    flush_block_only(false);
                    if (this.strm.avail_out == 0) {
                        return 0;
                    }
                } else {
                    continue;
                }
            } else if (this.match_available != 0) {
                if (_tr_tally(0, this.window[this.strstart - 1] & NavSatStatus.STATUS_NO_FIX)) {
                    flush_block_only(false);
                }
                this.strstart++;
                this.lookahead--;
                if (this.strm.avail_out == 0) {
                    return 0;
                }
            } else {
                this.match_available = 1;
                this.strstart++;
                this.lookahead--;
            }
        }
    }

    private int longest_match(int cur_match) {
        int chain_length = this.max_chain_length;
        int scan = this.strstart;
        int best_len = this.prev_length;
        int limit = this.strstart > this.w_size + -262 ? this.strstart - (this.w_size - 262) : 0;
        int nice_match2 = this.nice_match;
        int wmask = this.w_mask;
        int strend = this.strstart + 258;
        byte scan_end1 = this.window[(scan + best_len) - 1];
        byte scan_end = this.window[scan + best_len];
        if (this.prev_length >= this.good_match) {
            chain_length >>= 2;
        }
        if (nice_match2 > this.lookahead) {
            nice_match2 = this.lookahead;
        }
        do {
            int match = cur_match;
            if (this.window[match + best_len] == scan_end && this.window[(match + best_len) - 1] == scan_end1 && this.window[match] == this.window[scan]) {
                int match2 = match + 1;
                if (this.window[match2] == this.window[scan + 1]) {
                    int scan2 = scan + 2;
                    int match3 = match2 + 1;
                    do {
                        scan2++;
                        int match4 = match3 + 1;
                        if (this.window[scan2] != this.window[match4]) {
                            break;
                        }
                        scan2++;
                        int match5 = match4 + 1;
                        if (this.window[scan2] != this.window[match5]) {
                            break;
                        }
                        scan2++;
                        int match6 = match5 + 1;
                        if (this.window[scan2] != this.window[match6]) {
                            break;
                        }
                        scan2++;
                        int match7 = match6 + 1;
                        if (this.window[scan2] != this.window[match7]) {
                            break;
                        }
                        scan2++;
                        int match8 = match7 + 1;
                        if (this.window[scan2] != this.window[match8]) {
                            break;
                        }
                        scan2++;
                        int match9 = match8 + 1;
                        if (this.window[scan2] != this.window[match9]) {
                            break;
                        }
                        scan2++;
                        int match10 = match9 + 1;
                        if (this.window[scan2] != this.window[match10]) {
                            break;
                        }
                        scan2++;
                        match3 = match10 + 1;
                        if (this.window[scan2] != this.window[match3]) {
                            break;
                        }
                    } while (scan2 < strend);
                    int len = 258 - (strend - scan2);
                    scan = strend - 258;
                    if (len > best_len) {
                        this.match_start = cur_match;
                        best_len = len;
                        if (len >= nice_match2) {
                            break;
                        }
                        scan_end1 = this.window[(scan + best_len) - 1];
                        scan_end = this.window[scan + best_len];
                    }
                }
            }
            short s = this.prev[cur_match & wmask] & 65535;
            cur_match = s;
            if (s <= limit) {
                break;
            }
            chain_length--;
        } while (chain_length == 0);
        if (best_len <= this.lookahead) {
            return best_len;
        }
        return this.lookahead;
    }

    /* access modifiers changed from: package-private */
    public int deflateInit(ZStream strm2, int level2, int bits, int memLevel, JZlib.WrapperType wrapperType2) {
        return deflateInit2(strm2, level2, 8, bits, memLevel, 0, wrapperType2);
    }

    private int deflateInit2(ZStream strm2, int level2, int method, int windowBits, int memLevel, int strategy2, JZlib.WrapperType wrapperType2) {
        if (wrapperType2 != JZlib.WrapperType.ZLIB_OR_NONE) {
            strm2.msg = null;
            if (level2 == -1) {
                level2 = 6;
            }
            if (windowBits < 0) {
                throw new IllegalArgumentException("windowBits: " + windowBits);
            } else if (memLevel < 1 || memLevel > 9 || method != 8 || windowBits < 9 || windowBits > 15 || level2 < 0 || level2 > 9 || strategy2 < 0 || strategy2 > 2) {
                return -2;
            } else {
                strm2.dstate = this;
                this.wrapperType = wrapperType2;
                this.w_bits = windowBits;
                this.w_size = 1 << this.w_bits;
                this.w_mask = this.w_size - 1;
                this.hash_bits = memLevel + 7;
                this.hash_size = 1 << this.hash_bits;
                this.hash_mask = this.hash_size - 1;
                this.hash_shift = ((this.hash_bits + 3) - 1) / 3;
                this.window = new byte[(this.w_size * 2)];
                this.prev = new short[this.w_size];
                this.head = new short[this.hash_size];
                this.lit_bufsize = 1 << (memLevel + 6);
                this.pending_buf = new byte[(this.lit_bufsize * 4)];
                this.pending_buf_size = this.lit_bufsize * 4;
                this.d_buf = this.lit_bufsize / 2;
                this.l_buf = this.lit_bufsize * 3;
                this.level = level2;
                this.strategy = strategy2;
                return deflateReset(strm2);
            }
        } else {
            throw new IllegalArgumentException("ZLIB_OR_NONE allowed only for inflate");
        }
    }

    private int deflateReset(ZStream strm2) {
        strm2.total_out = 0;
        strm2.total_in = 0;
        strm2.msg = null;
        this.pending = 0;
        this.pending_out = 0;
        this.wroteTrailer = false;
        this.status = this.wrapperType == JZlib.WrapperType.NONE ? 113 : 42;
        strm2.adler = Adler32.adler32(0, (byte[]) null, 0, 0);
        strm2.crc32 = 0;
        this.gzipUncompressedBytes = 0;
        this.last_flush = 0;
        tr_init();
        lm_init();
        return 0;
    }

    /* access modifiers changed from: package-private */
    public int deflateEnd() {
        if (this.status != 42 && this.status != 113 && this.status != FINISH_STATE) {
            return -2;
        }
        this.pending_buf = null;
        this.head = null;
        this.prev = null;
        this.window = null;
        return this.status == 113 ? -3 : 0;
    }

    /* access modifiers changed from: package-private */
    public int deflateParams(ZStream strm2, int _level, int _strategy) {
        int err = 0;
        if (_level == -1) {
            _level = 6;
        }
        if (_level < 0 || _level > 9 || _strategy < 0 || _strategy > 2) {
            return -2;
        }
        if (!(config_table[this.level].func == config_table[_level].func || strm2.total_in == 0)) {
            err = strm2.deflate(1);
        }
        if (this.level != _level) {
            this.level = _level;
            this.max_lazy_match = config_table[this.level].max_lazy;
            this.good_match = config_table[this.level].good_length;
            this.nice_match = config_table[this.level].nice_length;
            this.max_chain_length = config_table[this.level].max_chain;
        }
        this.strategy = _strategy;
        return err;
    }

    /* access modifiers changed from: package-private */
    public int deflateSetDictionary(ZStream strm2, byte[] dictionary, int dictLength) {
        int length = dictLength;
        int index = 0;
        if (dictionary == null || this.status != 42) {
            return -2;
        }
        strm2.adler = Adler32.adler32(strm2.adler, dictionary, 0, dictLength);
        if (length < 3) {
            return 0;
        }
        if (length > this.w_size - 262) {
            length = this.w_size - 262;
            index = dictLength - length;
        }
        System.arraycopy(dictionary, index, this.window, 0, length);
        this.strstart = length;
        this.block_start = length;
        this.ins_h = this.window[0] & NavSatStatus.STATUS_NO_FIX;
        this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
        for (int n = 0; n <= length - 3; n++) {
            this.ins_h = ((this.ins_h << this.hash_shift) ^ (this.window[(n + 3) - 1] & NavSatStatus.STATUS_NO_FIX)) & this.hash_mask;
            this.prev[this.w_mask & n] = this.head[this.ins_h];
            this.head[this.ins_h] = (short) n;
        }
        return 0;
    }

    /* Debug info: failed to restart local var, previous not found, register: 18 */
    /* access modifiers changed from: package-private */
    public int deflate(ZStream strm2, int flush) {
        ZStream zStream = strm2;
        int i = flush;
        if (i > 4 || i < 0) {
            return -2;
        }
        if (zStream.next_out == null || ((zStream.next_in == null && zStream.avail_in != 0) || (this.status == FINISH_STATE && i != 4))) {
            zStream.msg = z_errmsg[4];
            return -2;
        } else if (zStream.avail_out == 0) {
            zStream.msg = z_errmsg[7];
            return -5;
        } else {
            this.strm = zStream;
            int old_flush = this.last_flush;
            this.last_flush = i;
            if (this.status == 42) {
                switch (this.wrapperType) {
                    case ZLIB:
                        int header = (((this.w_bits - 8) << 4) + 8) << 8;
                        int level_flags = ((this.level - 1) & 255) >> 1;
                        if (level_flags > 3) {
                            level_flags = 3;
                        }
                        int header2 = header | (level_flags << 6);
                        if (this.strstart != 0) {
                            header2 |= 32;
                        }
                        putShortMSB(header2 + (31 - (header2 % 31)));
                        if (this.strstart != 0) {
                            putShortMSB((int) (zStream.adler >>> 16));
                            putShortMSB((int) (zStream.adler & 65535));
                        }
                        zStream.adler = Adler32.adler32(0, (byte[]) null, 0, 0);
                        break;
                    case GZIP:
                        put_byte((byte) 31);
                        put_byte((byte) -117);
                        put_byte((byte) 8);
                        put_byte((byte) 0);
                        put_byte((byte) 0);
                        put_byte((byte) 0);
                        put_byte((byte) 0);
                        put_byte((byte) 0);
                        switch (config_table[this.level].func) {
                            case 1:
                                put_byte((byte) 4);
                                break;
                            case 2:
                                put_byte((byte) 2);
                                break;
                            default:
                                put_byte((byte) 0);
                                break;
                        }
                        put_byte((byte) -1);
                        zStream.crc32 = 0;
                        break;
                }
                this.status = 113;
            }
            if (this.pending != 0) {
                strm2.flush_pending();
                if (zStream.avail_out == 0) {
                    this.last_flush = -1;
                    return 0;
                }
            } else if (zStream.avail_in == 0 && i <= old_flush && i != 4) {
                zStream.msg = z_errmsg[7];
                return -5;
            }
            if (this.status != FINISH_STATE || zStream.avail_in == 0) {
                int old_next_in_index = zStream.next_in_index;
                try {
                    if (!(zStream.avail_in == 0 && this.lookahead == 0 && (i == 0 || this.status == FINISH_STATE))) {
                        int bstate = -1;
                        switch (config_table[this.level].func) {
                            case 0:
                                bstate = deflate_stored(i);
                                break;
                            case 1:
                                bstate = deflate_fast(i);
                                break;
                            case 2:
                                bstate = deflate_slow(i);
                                break;
                        }
                        if (bstate == 2 || bstate == 3) {
                            this.status = FINISH_STATE;
                        }
                        if (bstate != 0) {
                            if (bstate != 2) {
                                if (bstate == 1) {
                                    if (i == 1) {
                                        _tr_align();
                                    } else {
                                        _tr_stored_block(0, 0, false);
                                        if (i == 3) {
                                            for (int i2 = 0; i2 < this.hash_size; i2++) {
                                                this.head[i2] = 0;
                                            }
                                        }
                                    }
                                    strm2.flush_pending();
                                    if (zStream.avail_out == 0) {
                                        this.last_flush = -1;
                                        return 0;
                                    }
                                }
                            }
                        }
                        if (zStream.avail_out == 0) {
                            this.last_flush = -1;
                        }
                        return 0;
                    }
                    this.gzipUncompressedBytes += zStream.next_in_index - old_next_in_index;
                    if (i != 4) {
                        return 0;
                    }
                    if (this.wrapperType == JZlib.WrapperType.NONE || this.wroteTrailer) {
                        return 1;
                    }
                    switch (this.wrapperType) {
                        case ZLIB:
                            putShortMSB((int) (zStream.adler >>> 16));
                            putShortMSB((int) (65535 & zStream.adler));
                            break;
                        case GZIP:
                            put_byte((byte) (zStream.crc32 & 255));
                            put_byte((byte) ((zStream.crc32 >>> 8) & 255));
                            put_byte((byte) ((zStream.crc32 >>> 16) & 255));
                            put_byte((byte) ((zStream.crc32 >>> 24) & 255));
                            put_byte((byte) (this.gzipUncompressedBytes & 255));
                            put_byte((byte) ((this.gzipUncompressedBytes >>> 8) & 255));
                            put_byte((byte) ((this.gzipUncompressedBytes >>> 16) & 255));
                            put_byte((byte) ((this.gzipUncompressedBytes >>> 24) & 255));
                            break;
                    }
                    strm2.flush_pending();
                    this.wroteTrailer = true;
                    return this.pending != 0 ? 0 : 1;
                } finally {
                    this.gzipUncompressedBytes += zStream.next_in_index - old_next_in_index;
                }
            } else {
                zStream.msg = z_errmsg[7];
                return -5;
            }
        }
    }
}
