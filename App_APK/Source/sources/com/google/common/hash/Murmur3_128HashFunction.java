package com.google.common.hash;

import com.google.common.hash.AbstractStreamingHashFunction;
import java.io.Serializable;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

final class Murmur3_128HashFunction extends AbstractStreamingHashFunction implements Serializable {
    private static final long serialVersionUID = 0;
    private final int seed;

    Murmur3_128HashFunction(int seed2) {
        this.seed = seed2;
    }

    public int bits() {
        return 128;
    }

    public Hasher newHasher() {
        return new Murmur3_128Hasher(this.seed);
    }

    private static final class Murmur3_128Hasher extends AbstractStreamingHashFunction.AbstractStreamingHasher {
        long c1 = -8663945395140668459L;
        long c2 = 5545529020109919103L;
        long h1;
        long h2;
        int len;

        Murmur3_128Hasher(int seed) {
            super(16);
            this.h1 = (long) seed;
            this.h2 = (long) seed;
        }

        /* access modifiers changed from: protected */
        public void process(ByteBuffer bb) {
            long k1 = bb.getLong();
            long k2 = bb.getLong();
            this.len += 16;
            bmix64(k1, k2);
        }

        private void bmix64(long k1, long k2) {
            this.h1 ^= Long.rotateLeft(k1 * this.c1, 31) * this.c2;
            this.h1 = Long.rotateLeft(this.h1, 27);
            this.h1 += this.h2;
            this.h1 = (this.h1 * 5) + 1390208809;
            this.h2 ^= Long.rotateLeft(k2 * this.c2, 33) * this.c1;
            this.h2 = Long.rotateLeft(this.h2, 31);
            this.h2 += this.h1;
            this.h2 = (this.h2 * 5) + 944331445;
        }

        /* access modifiers changed from: protected */
        /* JADX WARNING: Code restructure failed: missing block: B:10:0x009d, code lost:
            r0 = r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(6))) << 48);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:11:0x00aa, code lost:
            r0 = r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(5))) << 40);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:12:0x00b6, code lost:
            r0 = r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(4))) << 32);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:13:0x00c2, code lost:
            r0 = r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(3))) << 24);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:14:0x00ce, code lost:
            r0 = r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(2))) << 16);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:15:0x00da, code lost:
            r0 = r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(1))) << 8);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:16:0x00e6, code lost:
            r14.h1 ^= java.lang.Long.rotateLeft((r0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(0))) << 0)) * r14.c1, 31) * r14.c2;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:17:?, code lost:
            return;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:3:0x0030, code lost:
            r2 = r2 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(13))) << 40);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:4:0x003d, code lost:
            r2 = r2 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(12))) << 32);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:5:0x004a, code lost:
            r2 = r2 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(11))) << 24);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:6:0x0057, code lost:
            r2 = r2 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(10))) << 16);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:7:0x0064, code lost:
            r2 = r2 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(9))) << 8);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:8:0x0071, code lost:
            r14.h2 ^= java.lang.Long.rotateLeft((r2 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(8))) << 0)) * r14.c2, 33) * r14.c1;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:9:0x008f, code lost:
            r0 = 0 ^ (((long) com.google.common.primitives.UnsignedBytes.toInt(r15.get(7))) << 56);
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public void processRemaining(java.nio.ByteBuffer r15) {
            /*
                r14 = this;
                r0 = 0
                r2 = 0
                int r4 = r14.len
                int r5 = r15.remaining()
                int r4 = r4 + r5
                r14.len = r4
                int r4 = r15.remaining()
                r5 = 48
                r6 = 40
                r7 = 32
                r8 = 24
                r9 = 16
                r10 = 8
                r11 = 0
                switch(r4) {
                    case 1: goto L_0x00e6;
                    case 2: goto L_0x00da;
                    case 3: goto L_0x00ce;
                    case 4: goto L_0x00c2;
                    case 5: goto L_0x00b6;
                    case 6: goto L_0x00aa;
                    case 7: goto L_0x009d;
                    case 8: goto L_0x008f;
                    case 9: goto L_0x0071;
                    case 10: goto L_0x0064;
                    case 11: goto L_0x0057;
                    case 12: goto L_0x004a;
                    case 13: goto L_0x003d;
                    case 14: goto L_0x0030;
                    case 15: goto L_0x0023;
                    default: goto L_0x0021;
                }
            L_0x0021:
                goto L_0x0104
            L_0x0023:
                r4 = 14
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r5
                long r2 = r2 ^ r12
            L_0x0030:
                r4 = 13
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r6
                long r2 = r2 ^ r12
            L_0x003d:
                r4 = 12
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r7
                long r2 = r2 ^ r12
            L_0x004a:
                r4 = 11
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r8
                long r2 = r2 ^ r12
            L_0x0057:
                r4 = 10
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r9
                long r2 = r2 ^ r12
            L_0x0064:
                r4 = 9
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r10
                long r2 = r2 ^ r12
            L_0x0071:
                byte r4 = r15.get(r10)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r12 = r12 << r11
                long r2 = r2 ^ r12
                long r12 = r14.c2
                long r2 = r2 * r12
                r4 = 33
                long r2 = java.lang.Long.rotateLeft(r2, r4)
                long r12 = r14.c1
                long r2 = r2 * r12
                long r12 = r14.h2
                long r12 = r12 ^ r2
                r14.h2 = r12
            L_0x008f:
                r4 = 7
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                r4 = 56
                long r12 = r12 << r4
                long r0 = r0 ^ r12
            L_0x009d:
                r4 = 6
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r12 = (long) r4
                long r4 = r12 << r5
                long r0 = r0 ^ r4
            L_0x00aa:
                r4 = 5
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r4 = (long) r4
                long r4 = r4 << r6
                long r0 = r0 ^ r4
            L_0x00b6:
                r4 = 4
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r4 = (long) r4
                long r4 = r4 << r7
                long r0 = r0 ^ r4
            L_0x00c2:
                r4 = 3
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r4 = (long) r4
                long r4 = r4 << r8
                long r0 = r0 ^ r4
            L_0x00ce:
                r4 = 2
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r4 = (long) r4
                long r4 = r4 << r9
                long r0 = r0 ^ r4
            L_0x00da:
                r4 = 1
                byte r4 = r15.get(r4)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r4 = (long) r4
                long r4 = r4 << r10
                long r0 = r0 ^ r4
            L_0x00e6:
                byte r4 = r15.get(r11)
                int r4 = com.google.common.primitives.UnsignedBytes.toInt(r4)
                long r4 = (long) r4
                long r4 = r4 << r11
                long r0 = r0 ^ r4
                long r4 = r14.c1
                long r0 = r0 * r4
                r4 = 31
                long r0 = java.lang.Long.rotateLeft(r0, r4)
                long r4 = r14.c2
                long r0 = r0 * r4
                long r4 = r14.h1
                long r4 = r4 ^ r0
                r14.h1 = r4
            L_0x0104:
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.hash.Murmur3_128HashFunction.Murmur3_128Hasher.processRemaining(java.nio.ByteBuffer):void");
        }

        public HashCode makeHash() {
            this.h1 ^= (long) this.len;
            this.h2 ^= (long) this.len;
            this.h1 += this.h2;
            this.h2 += this.h1;
            this.h1 = fmix64(this.h1);
            this.h2 = fmix64(this.h2);
            this.h1 += this.h2;
            this.h2 += this.h1;
            ByteBuffer bb = ByteBuffer.wrap(new byte[16]).order(ByteOrder.LITTLE_ENDIAN);
            bb.putLong(this.h1);
            bb.putLong(this.h2);
            return HashCodes.fromBytesNoCopy(bb.array());
        }

        private long fmix64(long k) {
            long k2 = (k ^ (k >>> 33)) * -49064778989728563L;
            long k3 = (k2 ^ (k2 >>> 33)) * -4265267296055464877L;
            return k3 ^ (k3 >>> 33);
        }
    }
}
