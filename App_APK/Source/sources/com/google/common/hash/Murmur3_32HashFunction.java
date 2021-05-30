package com.google.common.hash;

import com.google.common.hash.AbstractStreamingHashFunction;
import java.io.Serializable;
import java.nio.ByteBuffer;

final class Murmur3_32HashFunction extends AbstractStreamingHashFunction implements Serializable {
    private static final long serialVersionUID = 0;
    private final int seed;

    Murmur3_32HashFunction(int seed2) {
        this.seed = seed2;
    }

    public int bits() {
        return 32;
    }

    public Hasher newHasher() {
        return new Murmur3_32Hasher(this.seed);
    }

    private static final class Murmur3_32Hasher extends AbstractStreamingHashFunction.AbstractStreamingHasher {
        int c1 = -862048943;
        int c2 = 461845907;
        int h1;
        int len;

        Murmur3_32Hasher(int seed) {
            super(4);
            this.h1 = seed;
        }

        /* access modifiers changed from: protected */
        public void process(ByteBuffer bb) {
            int k1 = bb.getInt();
            this.len += 4;
            this.h1 ^= Integer.rotateLeft(k1 * this.c1, 15) * this.c2;
            this.h1 = Integer.rotateLeft(this.h1, 13);
            this.h1 = (this.h1 * 5) - 430675100;
        }

        /* access modifiers changed from: protected */
        /* JADX WARNING: Code restructure failed: missing block: B:3:0x001e, code lost:
            r0 = r0 ^ (com.google.common.primitives.UnsignedBytes.toInt(r3.get(1)) << 8);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:4:0x002a, code lost:
            r0 = r0 ^ com.google.common.primitives.UnsignedBytes.toInt(r3.get(0));
         */
        /* JADX WARNING: Code restructure failed: missing block: B:5:0x0034, code lost:
            r2.h1 ^= java.lang.Integer.rotateLeft(r0 * r2.c1, 15) * r2.c2;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:6:0x0047, code lost:
            return;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public void processRemaining(java.nio.ByteBuffer r3) {
            /*
                r2 = this;
                int r0 = r2.len
                int r1 = r3.remaining()
                int r0 = r0 + r1
                r2.len = r0
                r0 = 0
                int r1 = r3.remaining()
                switch(r1) {
                    case 1: goto L_0x002a;
                    case 2: goto L_0x001e;
                    case 3: goto L_0x0012;
                    default: goto L_0x0011;
                }
            L_0x0011:
                goto L_0x0034
            L_0x0012:
                r1 = 2
                byte r1 = r3.get(r1)
                int r1 = com.google.common.primitives.UnsignedBytes.toInt(r1)
                int r1 = r1 << 16
                r0 = r0 ^ r1
            L_0x001e:
                r1 = 1
                byte r1 = r3.get(r1)
                int r1 = com.google.common.primitives.UnsignedBytes.toInt(r1)
                int r1 = r1 << 8
                r0 = r0 ^ r1
            L_0x002a:
                r1 = 0
                byte r1 = r3.get(r1)
                int r1 = com.google.common.primitives.UnsignedBytes.toInt(r1)
                r0 = r0 ^ r1
            L_0x0034:
                int r1 = r2.c1
                int r0 = r0 * r1
                r1 = 15
                int r0 = java.lang.Integer.rotateLeft(r0, r1)
                int r1 = r2.c2
                int r0 = r0 * r1
                int r1 = r2.h1
                r1 = r1 ^ r0
                r2.h1 = r1
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: com.google.common.hash.Murmur3_32HashFunction.Murmur3_32Hasher.processRemaining(java.nio.ByteBuffer):void");
        }

        public HashCode makeHash() {
            this.h1 ^= this.len;
            this.h1 ^= this.h1 >>> 16;
            this.h1 *= -2048144789;
            this.h1 ^= this.h1 >>> 13;
            this.h1 *= -1028477387;
            this.h1 ^= this.h1 >>> 16;
            return HashCodes.fromInt(this.h1);
        }
    }
}
