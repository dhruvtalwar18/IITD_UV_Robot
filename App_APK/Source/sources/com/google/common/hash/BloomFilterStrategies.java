package com.google.common.hash;

import com.google.common.base.Preconditions;
import com.google.common.hash.BloomFilter;
import com.google.common.math.IntMath;
import java.math.RoundingMode;
import java.util.Arrays;

enum BloomFilterStrategies implements BloomFilter.Strategy {
    MURMUR128_MITZ_32 {
        public <T> boolean put(T object, Funnel<? super T> funnel, int numHashFunctions, BitArray bits) {
            long hash64 = Hashing.murmur3_128().newHasher().putObject(object, funnel).hash().asLong();
            int hash1 = (int) hash64;
            int hash2 = (int) (hash64 >>> 32);
            boolean bitsChanged = false;
            for (int i = 1; i <= numHashFunctions; i++) {
                int nextHash = (i * hash2) + hash1;
                if (nextHash < 0) {
                    nextHash ^= -1;
                }
                bitsChanged |= bits.set(nextHash % bits.size());
            }
            return bitsChanged;
        }

        public <T> boolean mightContain(T object, Funnel<? super T> funnel, int numHashFunctions, BitArray bits) {
            long hash64 = Hashing.murmur3_128().newHasher().putObject(object, funnel).hash().asLong();
            int hash1 = (int) hash64;
            int hash2 = (int) (hash64 >>> 32);
            for (int i = 1; i <= numHashFunctions; i++) {
                int nextHash = (i * hash2) + hash1;
                if (nextHash < 0) {
                    nextHash ^= -1;
                }
                if (!bits.get(nextHash % bits.size())) {
                    return false;
                }
            }
            return true;
        }
    };

    static class BitArray {
        final long[] data;

        BitArray(int bits) {
            this(new long[IntMath.divide(bits, 64, RoundingMode.CEILING)]);
        }

        BitArray(long[] data2) {
            Preconditions.checkArgument(data2.length > 0, "data length is zero!");
            this.data = data2;
        }

        /* access modifiers changed from: package-private */
        public boolean set(int index) {
            boolean wasSet = get(index);
            long[] jArr = this.data;
            int i = index >> 6;
            jArr[i] = jArr[i] | (1 << index);
            return !wasSet;
        }

        /* access modifiers changed from: package-private */
        public boolean get(int index) {
            return (this.data[index >> 6] & (1 << index)) != 0;
        }

        /* access modifiers changed from: package-private */
        public int size() {
            return this.data.length * 64;
        }

        /* access modifiers changed from: package-private */
        public BitArray copy() {
            return new BitArray((long[]) this.data.clone());
        }

        public boolean equals(Object o) {
            if (o instanceof BitArray) {
                return Arrays.equals(this.data, ((BitArray) o).data);
            }
            return false;
        }

        public int hashCode() {
            return Arrays.hashCode(this.data);
        }
    }
}
