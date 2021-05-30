package com.google.common.math;

import com.google.common.annotations.Beta;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.math.BigInteger;
import java.math.RoundingMode;

@Beta
public final class DoubleMath {
    @VisibleForTesting
    static final double[] EVERY_SIXTEENTH_FACTORIAL = {1.0d, 2.0922789888E13d, 2.631308369336935E35d, 1.2413915592536073E61d, 1.2688693218588417E89d, 7.156945704626381E118d, 9.916779348709496E149d, 1.974506857221074E182d, 3.856204823625804E215d, 5.5502938327393044E249d, 4.7147236359920616E284d};
    private static final double LN_2 = Math.log(2.0d);
    @VisibleForTesting
    static final int MAX_FACTORIAL = 170;
    private static final double MAX_INT_AS_DOUBLE = 2.147483647E9d;
    private static final double MAX_LONG_AS_DOUBLE_PLUS_ONE = 9.223372036854776E18d;
    private static final double MIN_INT_AS_DOUBLE = -2.147483648E9d;
    private static final double MIN_LONG_AS_DOUBLE = -9.223372036854776E18d;

    static double roundIntermediate(double x, RoundingMode mode) {
        if (DoubleUtils.isFinite(x)) {
            switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
                case 1:
                    MathPreconditions.checkRoundingUnnecessary(isMathematicalInteger(x));
                    return x;
                case 2:
                    return x >= 0.0d ? x : Math.floor(x);
                case 3:
                    return x >= 0.0d ? Math.ceil(x) : x;
                case 4:
                    return x;
                case 5:
                    return x >= 0.0d ? Math.ceil(x) : Math.floor(x);
                case 6:
                    return Math.rint(x);
                case 7:
                    if (isMathematicalInteger(x)) {
                        return x;
                    }
                    return x >= 0.0d ? 0.5d + x : x - 0.5d;
                case 8:
                    if (isMathematicalInteger(x)) {
                        return x;
                    }
                    if (x >= 0.0d) {
                        double z = 0.5d + x;
                        return z == x ? x : DoubleUtils.nextDown(z);
                    }
                    double z2 = x - 0.5d;
                    return z2 == x ? x : Math.nextUp(z2);
                default:
                    throw new AssertionError();
            }
        } else {
            throw new ArithmeticException("input is infinite or NaN");
        }
    }

    /* renamed from: com.google.common.math.DoubleMath$1  reason: invalid class name */
    static /* synthetic */ class AnonymousClass1 {
        static final /* synthetic */ int[] $SwitchMap$java$math$RoundingMode = new int[RoundingMode.values().length];

        static {
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.UNNECESSARY.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.FLOOR.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.CEILING.ordinal()] = 3;
            } catch (NoSuchFieldError e3) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.DOWN.ordinal()] = 4;
            } catch (NoSuchFieldError e4) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.UP.ordinal()] = 5;
            } catch (NoSuchFieldError e5) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.HALF_EVEN.ordinal()] = 6;
            } catch (NoSuchFieldError e6) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.HALF_UP.ordinal()] = 7;
            } catch (NoSuchFieldError e7) {
            }
            try {
                $SwitchMap$java$math$RoundingMode[RoundingMode.HALF_DOWN.ordinal()] = 8;
            } catch (NoSuchFieldError e8) {
            }
        }
    }

    public static int roundToInt(double x, RoundingMode mode) {
        double z = roundIntermediate(x, mode);
        boolean z2 = false;
        boolean z3 = z > -2.147483649E9d;
        if (z < 2.147483648E9d) {
            z2 = true;
        }
        MathPreconditions.checkInRange(z2 & z3);
        return (int) z;
    }

    public static long roundToLong(double x, RoundingMode mode) {
        double z = roundIntermediate(x, mode);
        boolean z2 = false;
        boolean z3 = MIN_LONG_AS_DOUBLE - z < 1.0d;
        if (z < MAX_LONG_AS_DOUBLE_PLUS_ONE) {
            z2 = true;
        }
        MathPreconditions.checkInRange(z3 & z2);
        return (long) z;
    }

    public static BigInteger roundToBigInteger(double x, RoundingMode mode) {
        double x2 = roundIntermediate(x, mode);
        boolean z = false;
        boolean z2 = MIN_LONG_AS_DOUBLE - x2 < 1.0d;
        if (x2 < MAX_LONG_AS_DOUBLE_PLUS_ONE) {
            z = true;
        }
        if (z2 && z) {
            return BigInteger.valueOf((long) x2);
        }
        int exponent = Math.getExponent(x2);
        if (exponent < 0) {
            return BigInteger.ZERO;
        }
        BigInteger result = BigInteger.valueOf(DoubleUtils.getSignificand(x2)).shiftLeft(exponent - 52);
        return x2 < 0.0d ? result.negate() : result;
    }

    public static boolean isPowerOfTwo(double x) {
        return x > 0.0d && DoubleUtils.isFinite(x) && LongMath.isPowerOfTwo(DoubleUtils.getSignificand(x));
    }

    public static double log2(double x) {
        return Math.log(x) / LN_2;
    }

    public static int log2(double x, RoundingMode mode) {
        boolean increment = false;
        Preconditions.checkArgument(x > 0.0d && DoubleUtils.isFinite(x), "x must be positive and finite");
        int exponent = Math.getExponent(x);
        if (!DoubleUtils.isNormal(x)) {
            return log2(4.503599627370496E15d * x, mode) - 52;
        }
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                MathPreconditions.checkRoundingUnnecessary(isPowerOfTwo(x));
                break;
            case 2:
                break;
            case 3:
                increment = !isPowerOfTwo(x);
                break;
            case 4:
                if (exponent < 0) {
                    increment = true;
                }
                increment &= true ^ isPowerOfTwo(x);
                break;
            case 5:
                if (exponent >= 0) {
                    increment = true;
                }
                increment &= true ^ isPowerOfTwo(x);
                break;
            case 6:
            case 7:
            case 8:
                double xScaled = DoubleUtils.scaleNormalize(x);
                if (xScaled * xScaled > 2.0d) {
                    increment = true;
                    break;
                }
                break;
            default:
                throw new AssertionError();
        }
        increment = false;
        return increment ? exponent + 1 : exponent;
    }

    public static boolean isMathematicalInteger(double x) {
        return DoubleUtils.isFinite(x) && (x == 0.0d || 52 - Long.numberOfTrailingZeros(DoubleUtils.getSignificand(x)) <= Math.getExponent(x));
    }

    public static double factorial(int n) {
        MathPreconditions.checkNonNegative("n", n);
        if (n > MAX_FACTORIAL) {
            return Double.POSITIVE_INFINITY;
        }
        double accum = 1.0d;
        int i = n & -16;
        while (true) {
            i++;
            if (i > n) {
                return EVERY_SIXTEENTH_FACTORIAL[n >> 4] * accum;
            }
            double d = (double) i;
            Double.isNaN(d);
            accum *= d;
        }
    }

    private DoubleMath() {
    }
}