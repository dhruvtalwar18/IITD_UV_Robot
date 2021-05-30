package com.google.common.hash;

import com.google.common.annotations.Beta;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.hash.BloomFilterStrategies;
import java.io.Serializable;
import org.bytedeco.javacpp.opencv_stitching;

@Beta
public final class BloomFilter<T> implements Serializable {
    private static final double LN2 = Math.log(2.0d);
    private static final double LN2_SQUARED = (LN2 * LN2);
    /* access modifiers changed from: private */
    public final BloomFilterStrategies.BitArray bits;
    /* access modifiers changed from: private */
    public final Funnel<T> funnel;
    /* access modifiers changed from: private */
    public final int numHashFunctions;
    /* access modifiers changed from: private */
    public final Strategy strategy;

    interface Strategy extends Serializable {
        <T> boolean mightContain(T t, Funnel<? super T> funnel, int i, BloomFilterStrategies.BitArray bitArray);

        int ordinal();

        <T> boolean put(T t, Funnel<? super T> funnel, int i, BloomFilterStrategies.BitArray bitArray);
    }

    private BloomFilter(BloomFilterStrategies.BitArray bits2, int numHashFunctions2, Funnel<T> funnel2, Strategy strategy2) {
        Preconditions.checkArgument(numHashFunctions2 > 0, "numHashFunctions zero or negative");
        this.bits = (BloomFilterStrategies.BitArray) Preconditions.checkNotNull(bits2);
        this.numHashFunctions = numHashFunctions2;
        this.funnel = (Funnel) Preconditions.checkNotNull(funnel2);
        this.strategy = strategy2;
        if (numHashFunctions2 > 255) {
            throw new AssertionError("Currently we don't allow BloomFilters that would use more than255 hash functions, please contact the guava team");
        }
    }

    public BloomFilter<T> copy() {
        return new BloomFilter<>(this.bits.copy(), this.numHashFunctions, this.funnel, this.strategy);
    }

    public boolean mightContain(T object) {
        return this.strategy.mightContain(object, this.funnel, this.numHashFunctions, this.bits);
    }

    public boolean put(T object) {
        return this.strategy.put(object, this.funnel, this.numHashFunctions, this.bits);
    }

    public boolean equals(Object o) {
        if (!(o instanceof BloomFilter)) {
            return false;
        }
        BloomFilter<?> that = (BloomFilter) o;
        if (this.numHashFunctions == that.numHashFunctions && this.bits.equals(that.bits) && this.funnel == that.funnel && this.strategy == that.strategy) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return this.bits.hashCode();
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public int getHashCount() {
        return this.numHashFunctions;
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public double computeExpectedFalsePositiveRate(int insertions) {
        double d = (double) (-this.numHashFunctions);
        double d2 = (double) insertions;
        double size = (double) this.bits.size();
        Double.isNaN(d2);
        Double.isNaN(size);
        Double.isNaN(d);
        return Math.pow(1.0d - Math.exp(d * (d2 / size)), (double) this.numHashFunctions);
    }

    public static <T> BloomFilter<T> create(Funnel<T> funnel2, int expectedInsertions, double falsePositiveProbability) {
        Preconditions.checkNotNull(funnel2);
        boolean z = false;
        Preconditions.checkArgument(expectedInsertions > 0, "Expected insertions must be positive");
        boolean z2 = falsePositiveProbability > opencv_stitching.Stitcher.ORIG_RESOL;
        if (falsePositiveProbability < 1.0d) {
            z = true;
        }
        Preconditions.checkArgument(z & z2, "False positive probability in (0.0, 1.0)");
        int numBits = optimalNumOfBits(expectedInsertions, falsePositiveProbability);
        return new BloomFilter<>(new BloomFilterStrategies.BitArray(numBits), optimalNumOfHashFunctions(expectedInsertions, numBits), funnel2, BloomFilterStrategies.MURMUR128_MITZ_32);
    }

    public static <T> BloomFilter<T> create(Funnel<T> funnel2, int expectedInsertions) {
        return create(funnel2, expectedInsertions, 0.03d);
    }

    @VisibleForTesting
    static int optimalNumOfHashFunctions(int n, int m) {
        double d = (double) (m / n);
        double d2 = LN2;
        Double.isNaN(d);
        return Math.max(1, (int) Math.round(d * d2));
    }

    @VisibleForTesting
    static int optimalNumOfBits(int n, double p) {
        double d = (double) (-n);
        double log = Math.log(p);
        Double.isNaN(d);
        return (int) ((d * log) / LN2_SQUARED);
    }

    private Object writeReplace() {
        return new SerialForm(this);
    }

    private static class SerialForm<T> implements Serializable {
        private static final long serialVersionUID = 1;
        final long[] data;
        final Funnel<T> funnel;
        final int numHashFunctions;
        final Strategy strategy;

        SerialForm(BloomFilter<T> bf) {
            this.data = bf.bits.data;
            this.numHashFunctions = bf.numHashFunctions;
            this.funnel = bf.funnel;
            this.strategy = bf.strategy;
        }

        /* access modifiers changed from: package-private */
        public Object readResolve() {
            return new BloomFilter(new BloomFilterStrategies.BitArray(this.data), this.numHashFunctions, this.funnel, this.strategy);
        }
    }
}
