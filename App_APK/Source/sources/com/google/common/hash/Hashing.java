package com.google.common.hash;

import com.google.common.annotations.Beta;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.primitives.UnsignedInts;
import java.nio.ByteBuffer;
import java.util.Iterator;

@Beta
public final class Hashing {
    private static final int GOOD_FAST_HASH_SEED = ((int) System.currentTimeMillis());
    private static final HashFunction MD5 = new MessageDigestHashFunction("MD5");
    private static final Murmur3_128HashFunction MURMUR3_128 = new Murmur3_128HashFunction(0);
    private static final Murmur3_32HashFunction MURMUR3_32 = new Murmur3_32HashFunction(0);
    private static final HashFunction SHA_1 = new MessageDigestHashFunction("SHA-1");
    private static final HashFunction SHA_256 = new MessageDigestHashFunction("SHA-256");
    private static final HashFunction SHA_512 = new MessageDigestHashFunction("SHA-512");

    private Hashing() {
    }

    public static HashFunction goodFastHash(int minimumBits) {
        int bits = checkPositiveAndMakeMultipleOf32(minimumBits);
        if (bits == 32) {
            return murmur3_32(GOOD_FAST_HASH_SEED);
        }
        if (bits <= 128) {
            return murmur3_128(GOOD_FAST_HASH_SEED);
        }
        int hashFunctionsNeeded = (bits + 127) / 128;
        HashFunction[] hashFunctions = new HashFunction[hashFunctionsNeeded];
        int seed = GOOD_FAST_HASH_SEED;
        for (int i = 0; i < hashFunctionsNeeded; i++) {
            hashFunctions[i] = murmur3_128(seed);
            seed += 1500450271;
        }
        return new ConcatenatedHashFunction(hashFunctions);
    }

    public static HashFunction murmur3_32(int seed) {
        return new Murmur3_32HashFunction(seed);
    }

    public static HashFunction murmur3_32() {
        return MURMUR3_32;
    }

    public static HashFunction murmur3_128(int seed) {
        return new Murmur3_128HashFunction(seed);
    }

    public static HashFunction murmur3_128() {
        return MURMUR3_128;
    }

    public static HashFunction md5() {
        return MD5;
    }

    public static HashFunction sha1() {
        return SHA_1;
    }

    public static HashFunction sha256() {
        return SHA_256;
    }

    public static HashFunction sha512() {
        return SHA_512;
    }

    public static long padToLong(HashCode hashCode) {
        return hashCode.bits() < 64 ? UnsignedInts.toLong(hashCode.asInt()) : hashCode.asLong();
    }

    public static int consistentHash(HashCode hashCode, int buckets) {
        return consistentHash(padToLong(hashCode), buckets);
    }

    public static int consistentHash(long input, int buckets) {
        int candidate = 0;
        Preconditions.checkArgument(buckets > 0, "buckets must be positive: %s", Integer.valueOf(buckets));
        long h = input;
        while (true) {
            h = (2862933555777941757L * h) + 1;
            double d = (double) (((int) (h >>> 33)) + 1);
            Double.isNaN(d);
            double inv = 2.147483648E9d / d;
            double d2 = (double) (candidate + 1);
            Double.isNaN(d2);
            int next = (int) (d2 * inv);
            if (next < 0 || next >= buckets) {
                return candidate;
            }
            candidate = next;
        }
        return candidate;
    }

    public static HashCode combineOrdered(Iterable<HashCode> hashCodes) {
        Iterator<HashCode> iterator = hashCodes.iterator();
        Preconditions.checkArgument(iterator.hasNext(), "Must be at least 1 hash code to combine.");
        byte[] resultBytes = new byte[(iterator.next().bits() / 8)];
        for (HashCode hashCode : hashCodes) {
            byte[] nextBytes = hashCode.asBytes();
            int i = 0;
            Preconditions.checkArgument(nextBytes.length == resultBytes.length, "All hashcodes must have the same bit length.");
            while (true) {
                int i2 = i;
                if (i2 < nextBytes.length) {
                    resultBytes[i2] = (byte) ((resultBytes[i2] * 37) ^ nextBytes[i2]);
                    i = i2 + 1;
                }
            }
        }
        return HashCodes.fromBytesNoCopy(resultBytes);
    }

    public static HashCode combineUnordered(Iterable<HashCode> hashCodes) {
        Iterator<HashCode> iterator = hashCodes.iterator();
        Preconditions.checkArgument(iterator.hasNext(), "Must be at least 1 hash code to combine.");
        byte[] resultBytes = new byte[(iterator.next().bits() / 8)];
        for (HashCode hashCode : hashCodes) {
            byte[] nextBytes = hashCode.asBytes();
            int i = 0;
            Preconditions.checkArgument(nextBytes.length == resultBytes.length, "All hashcodes must have the same bit length.");
            while (true) {
                int i2 = i;
                if (i2 < nextBytes.length) {
                    resultBytes[i2] = (byte) (resultBytes[i2] + nextBytes[i2]);
                    i = i2 + 1;
                }
            }
        }
        return HashCodes.fromBytesNoCopy(resultBytes);
    }

    static int checkPositiveAndMakeMultipleOf32(int bits) {
        Preconditions.checkArgument(bits > 0, "Number of bits must be positive");
        return (bits + 31) & -32;
    }

    @VisibleForTesting
    static final class ConcatenatedHashFunction extends AbstractCompositeHashFunction {
        private final int bits;

        ConcatenatedHashFunction(HashFunction... functions) {
            super(functions);
            int bitSum = 0;
            for (HashFunction function : functions) {
                bitSum += function.bits();
            }
            this.bits = bitSum;
        }

        /* access modifiers changed from: package-private */
        public HashCode makeHash(Hasher[] hashers) {
            byte[] bytes = new byte[(this.bits / 8)];
            ByteBuffer buffer = ByteBuffer.wrap(bytes);
            for (Hasher hasher : hashers) {
                buffer.put(hasher.hash().asBytes());
            }
            return HashCodes.fromBytesNoCopy(bytes);
        }

        public int bits() {
            return this.bits;
        }
    }
}
