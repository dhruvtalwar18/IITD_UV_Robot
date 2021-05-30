package com.google.common.math;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

@GwtCompatible(emulated = true)
@Beta
public final class BigIntegerMath {
    @VisibleForTesting
    static final BigInteger SQRT2_PRECOMPUTED_BITS = new BigInteger("16a09e667f3bcc908b2fb1366ea957d3e3adec17512775099da2f590b0667322a", 16);
    @VisibleForTesting
    static final int SQRT2_PRECOMPUTE_THRESHOLD = 256;

    public static boolean isPowerOfTwo(BigInteger x) {
        Preconditions.checkNotNull(x);
        return x.signum() > 0 && x.getLowestSetBit() == x.bitLength() - 1;
    }

    public static int log2(BigInteger x, RoundingMode mode) {
        MathPreconditions.checkPositive("x", (BigInteger) Preconditions.checkNotNull(x));
        int logFloor = x.bitLength() - 1;
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                MathPreconditions.checkRoundingUnnecessary(isPowerOfTwo(x));
                break;
            case 2:
            case 3:
                break;
            case 4:
            case 5:
                return isPowerOfTwo(x) ? logFloor : logFloor + 1;
            case 6:
            case 7:
            case 8:
                if (logFloor >= 256) {
                    return x.pow(2).bitLength() + -1 < (logFloor * 2) + 1 ? logFloor : logFloor + 1;
                }
                if (x.compareTo(SQRT2_PRECOMPUTED_BITS.shiftRight(256 - logFloor)) <= 0) {
                    return logFloor;
                }
                return logFloor + 1;
            default:
                throw new AssertionError();
        }
        return logFloor;
    }

    /* renamed from: com.google.common.math.BigIntegerMath$1  reason: invalid class name */
    static /* synthetic */ class AnonymousClass1 {
        static final /* synthetic */ int[] $SwitchMap$java$math$RoundingMode = new int[RoundingMode.values().length];

        static {
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.UNNECESSARY.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.DOWN.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.FLOOR.ordinal()] = 3;
            } catch (NoSuchFieldError e3) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.UP.ordinal()] = 4;
            } catch (NoSuchFieldError e4) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.CEILING.ordinal()] = 5;
            } catch (NoSuchFieldError e5) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.HALF_DOWN.ordinal()] = 6;
            } catch (NoSuchFieldError e6) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.HALF_UP.ordinal()] = 7;
            } catch (NoSuchFieldError e7) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.HALF_EVEN.ordinal()] = 8;
            } catch (NoSuchFieldError e8) {
            }
        }
    }

    @GwtIncompatible("TODO")
    public static int log10(BigInteger x, RoundingMode mode) {
        MathPreconditions.checkPositive("x", x);
        if (fitsInLong(x)) {
            return LongMath.log10(x.longValue(), mode);
        }
        List<BigInteger> powersOf10 = new ArrayList<>(10);
        for (BigInteger powerOf10 = BigInteger.TEN; x.compareTo(powerOf10) >= 0; powerOf10 = powerOf10.pow(2)) {
            powersOf10.add(powerOf10);
        }
        BigInteger floorPow = BigInteger.ONE;
        int floorLog = 0;
        for (int i = powersOf10.size() - 1; i >= 0; i--) {
            floorLog *= 2;
            BigInteger tenPow = powersOf10.get(i).multiply(floorPow);
            if (x.compareTo(tenPow) >= 0) {
                floorPow = tenPow;
                floorLog++;
            }
        }
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                MathPreconditions.checkRoundingUnnecessary(floorPow.equals(x));
                break;
            case 2:
            case 3:
                break;
            case 4:
            case 5:
                return floorPow.equals(x) ? floorLog : floorLog + 1;
            case 6:
            case 7:
            case 8:
                return x.pow(2).compareTo(floorPow.pow(2).multiply(BigInteger.TEN)) <= 0 ? floorLog : floorLog + 1;
            default:
                throw new AssertionError();
        }
        return floorLog;
    }

    @GwtIncompatible("TODO")
    public static BigInteger sqrt(BigInteger x, RoundingMode mode) {
        MathPreconditions.checkNonNegative("x", x);
        if (fitsInLong(x)) {
            return BigInteger.valueOf(LongMath.sqrt(x.longValue(), mode));
        }
        BigInteger sqrtFloor = sqrtFloor(x);
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                MathPreconditions.checkRoundingUnnecessary(sqrtFloor.pow(2).equals(x));
                break;
            case 2:
            case 3:
                break;
            case 4:
            case 5:
                return sqrtFloor.pow(2).equals(x) ? sqrtFloor : sqrtFloor.add(BigInteger.ONE);
            case 6:
            case 7:
            case 8:
                return sqrtFloor.pow(2).add(sqrtFloor).compareTo(x) >= 0 ? sqrtFloor : sqrtFloor.add(BigInteger.ONE);
            default:
                throw new AssertionError();
        }
        return sqrtFloor;
    }

    @GwtIncompatible("TODO")
    private static BigInteger sqrtFloor(BigInteger x) {
        BigInteger sqrt0;
        BigInteger sqrt02;
        int log2 = log2(x, RoundingMode.FLOOR);
        if (log2 < 1023) {
            sqrt0 = sqrtApproxWithDoubles(x);
        } else {
            int shift = (log2 - 52) & -2;
            sqrt0 = sqrtApproxWithDoubles(x.shiftRight(shift)).shiftLeft(shift >> 1);
        }
        BigInteger sqrt1 = sqrt0.add(x.divide(sqrt0)).shiftRight(1);
        if (sqrt0.equals(sqrt1)) {
            return sqrt0;
        }
        do {
            sqrt02 = sqrt1;
            sqrt1 = sqrt02.add(x.divide(sqrt02)).shiftRight(1);
        } while (sqrt1.compareTo(sqrt02) < 0);
        return sqrt02;
    }

    @GwtIncompatible("TODO")
    private static BigInteger sqrtApproxWithDoubles(BigInteger x) {
        return DoubleMath.roundToBigInteger(Math.sqrt(DoubleUtils.bigToDouble(x)), RoundingMode.HALF_EVEN);
    }

    @GwtIncompatible("TODO")
    public static BigInteger divide(BigInteger p, BigInteger q, RoundingMode mode) {
        return new BigDecimal(p).divide(new BigDecimal(q), 0, mode).toBigIntegerExact();
    }

    public static BigInteger factorial(int n) {
        int i = n;
        MathPreconditions.checkNonNegative("n", i);
        if (i < LongMath.FACTORIALS.length) {
            return BigInteger.valueOf(LongMath.FACTORIALS[i]);
        }
        ArrayList<BigInteger> bignums = new ArrayList<>(IntMath.divide(IntMath.log2(i, RoundingMode.CEILING) * i, 64, RoundingMode.CEILING));
        int startingNumber = LongMath.FACTORIALS.length;
        long product = LongMath.FACTORIALS[startingNumber - 1];
        int shift = Long.numberOfTrailingZeros(product);
        long product2 = product >> shift;
        int productBits = LongMath.log2(product2, RoundingMode.FLOOR) + 1;
        int bits = LongMath.log2((long) startingNumber, RoundingMode.FLOOR) + 1;
        int nextPowerOfTwo = 1 << (bits - 1);
        for (long num = (long) startingNumber; num <= ((long) i); num++) {
            if ((((long) nextPowerOfTwo) & num) != 0) {
                nextPowerOfTwo <<= 1;
                bits++;
            }
            int tz = Long.numberOfTrailingZeros(num);
            long normalizedNum = num >> tz;
            shift += tz;
            if ((bits - tz) + productBits >= 64) {
                bignums.add(BigInteger.valueOf(product2));
                product2 = 1;
            }
            product2 *= normalizedNum;
            productBits = LongMath.log2(product2, RoundingMode.FLOOR) + 1;
        }
        if (product2 > 1) {
            bignums.add(BigInteger.valueOf(product2));
        }
        return listProduct(bignums).shiftLeft(shift);
    }

    static BigInteger listProduct(List<BigInteger> nums) {
        return listProduct(nums, 0, nums.size());
    }

    static BigInteger listProduct(List<BigInteger> nums, int start, int end) {
        switch (end - start) {
            case 0:
                return BigInteger.ONE;
            case 1:
                return nums.get(start);
            case 2:
                return nums.get(start).multiply(nums.get(start + 1));
            case 3:
                return nums.get(start).multiply(nums.get(start + 1)).multiply(nums.get(start + 2));
            default:
                int m = (end + start) >>> 1;
                return listProduct(nums, start, m).multiply(listProduct(nums, m, end));
        }
    }

    public static BigInteger binomial(int n, int k) {
        MathPreconditions.checkNonNegative("n", n);
        MathPreconditions.checkNonNegative("k", k);
        Preconditions.checkArgument(k <= n, "k (%s) > n (%s)", Integer.valueOf(k), Integer.valueOf(n));
        if (k > (n >> 1)) {
            k = n - k;
        }
        if (k < LongMath.BIGGEST_BINOMIALS.length && n <= LongMath.BIGGEST_BINOMIALS[k]) {
            return BigInteger.valueOf(LongMath.binomial(n, k));
        }
        BigInteger result = BigInteger.ONE;
        for (int i = 0; i < k; i++) {
            result = result.multiply(BigInteger.valueOf((long) (n - i))).divide(BigInteger.valueOf((long) (i + 1)));
        }
        return result;
    }

    @GwtIncompatible("TODO")
    static boolean fitsInLong(BigInteger x) {
        return x.bitLength() <= 63;
    }

    private BigIntegerMath() {
    }
}
