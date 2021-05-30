package com.google.common.math;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.math.RoundingMode;

@GwtCompatible(emulated = true)
@Beta
public final class IntMath {
    @VisibleForTesting
    static int[] BIGGEST_BINOMIALS = {Integer.MAX_VALUE, Integer.MAX_VALUE, 65536, 2345, 477, 193, 110, 75, 58, 49, 43, 39, 37, 35, 34, 34, 33};
    static final int[] FACTORIALS = {1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880, 3628800, 39916800, 479001600};
    @VisibleForTesting
    static final int FLOOR_SQRT_MAX_INT = 46340;
    @VisibleForTesting
    static final int[] HALF_POWERS_OF_10 = {3, 31, 316, 3162, 31622, 316227, 3162277, 31622776, 316227766, Integer.MAX_VALUE};
    @VisibleForTesting
    static final int MAX_POWER_OF_SQRT2_UNSIGNED = -1257966797;
    @VisibleForTesting
    static final int[] POWERS_OF_10 = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

    public static boolean isPowerOfTwo(int x) {
        boolean z = false;
        boolean z2 = x > 0;
        if (((x - 1) & x) == 0) {
            z = true;
        }
        return z & z2;
    }

    /* renamed from: com.google.common.math.IntMath$1  reason: invalid class name */
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

    public static int log2(int x, RoundingMode mode) {
        MathPreconditions.checkPositive("x", x);
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                MathPreconditions.checkRoundingUnnecessary(isPowerOfTwo(x));
                break;
            case 2:
            case 3:
                break;
            case 4:
            case 5:
                return 32 - Integer.numberOfLeadingZeros(x - 1);
            case 6:
            case 7:
            case 8:
                int leadingZeros = Integer.numberOfLeadingZeros(x);
                int logFloor = 31 - leadingZeros;
                return x <= (MAX_POWER_OF_SQRT2_UNSIGNED >>> leadingZeros) ? logFloor : logFloor + 1;
            default:
                throw new AssertionError();
        }
        return 31 - Integer.numberOfLeadingZeros(x);
    }

    @GwtIncompatible("need BigIntegerMath to adequately test")
    public static int log10(int x, RoundingMode mode) {
        MathPreconditions.checkPositive("x", x);
        int logFloor = log10Floor(x);
        int floorPow = POWERS_OF_10[logFloor];
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                MathPreconditions.checkRoundingUnnecessary(x == floorPow);
                break;
            case 2:
            case 3:
                break;
            case 4:
            case 5:
                return x == floorPow ? logFloor : logFloor + 1;
            case 6:
            case 7:
            case 8:
                return x <= HALF_POWERS_OF_10[logFloor] ? logFloor : logFloor + 1;
            default:
                throw new AssertionError();
        }
        return logFloor;
    }

    private static int log10Floor(int x) {
        for (int i = 1; i < POWERS_OF_10.length; i++) {
            if (x < POWERS_OF_10[i]) {
                return i - 1;
            }
        }
        return POWERS_OF_10.length - 1;
    }

    @GwtIncompatible("failing tests")
    public static int pow(int b, int k) {
        MathPreconditions.checkNonNegative("exponent", k);
        switch (b) {
            case -2:
                if (k < 32) {
                    return (k & 1) == 0 ? 1 << k : -(1 << k);
                }
                return 0;
            case -1:
                if ((k & 1) == 0) {
                    return 1;
                }
                return -1;
            case 0:
                if (k == 0) {
                    return 1;
                }
                return 0;
            case 1:
                return 1;
            case 2:
                if (k < 32) {
                    return 1 << k;
                }
                return 0;
            default:
                int b2 = b;
                int accum = 1;
                while (true) {
                    switch (k) {
                        case 0:
                            return accum;
                        case 1:
                            return b2 * accum;
                        default:
                            accum *= (k & 1) == 0 ? 1 : b2;
                            b2 *= b2;
                            k >>= 1;
                    }
                }
        }
    }

    @GwtIncompatible("need BigIntegerMath to adequately test")
    public static int sqrt(int x, RoundingMode mode) {
        MathPreconditions.checkNonNegative("x", x);
        int sqrtFloor = sqrtFloor(x);
        boolean z = false;
        switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
            case 1:
                if (sqrtFloor * sqrtFloor == x) {
                    z = true;
                }
                MathPreconditions.checkRoundingUnnecessary(z);
                break;
            case 2:
            case 3:
                break;
            case 4:
            case 5:
                return sqrtFloor * sqrtFloor == x ? sqrtFloor : sqrtFloor + 1;
            case 6:
            case 7:
            case 8:
                int halfSquare = (sqrtFloor * sqrtFloor) + sqrtFloor;
                boolean z2 = x <= halfSquare;
                if (halfSquare < 0) {
                    z = true;
                }
                return z | z2 ? sqrtFloor : sqrtFloor + 1;
            default:
                throw new AssertionError();
        }
        return sqrtFloor;
    }

    private static int sqrtFloor(int x) {
        return (int) Math.sqrt((double) x);
    }

    public static int divide(int p, int q, RoundingMode mode) {
        Preconditions.checkNotNull(mode);
        if (q != 0) {
            int div = p / q;
            int rem = p - (q * div);
            if (rem == 0) {
                return div;
            }
            boolean increment = true;
            int signum = ((p ^ q) >> 31) | 1;
            switch (AnonymousClass1.$SwitchMap$java$math$RoundingMode[mode.ordinal()]) {
                case 1:
                    if (rem != 0) {
                        increment = false;
                    }
                    MathPreconditions.checkRoundingUnnecessary(increment);
                    break;
                case 2:
                    break;
                case 3:
                    if (signum >= 0) {
                        increment = false;
                        break;
                    }
                    break;
                case 4:
                    increment = true;
                    break;
                case 5:
                    if (signum <= 0) {
                        increment = false;
                        break;
                    }
                    break;
                case 6:
                case 7:
                case 8:
                    int absRem = Math.abs(rem);
                    int cmpRemToHalfDivisor = absRem - (Math.abs(q) - absRem);
                    if (cmpRemToHalfDivisor != 0) {
                        if (cmpRemToHalfDivisor <= 0) {
                            increment = false;
                            break;
                        }
                    } else if (mode != RoundingMode.HALF_UP) {
                        if (!(mode == RoundingMode.HALF_EVEN) || !((div & 1) != 0)) {
                            increment = false;
                            break;
                        }
                    }
                    break;
                default:
                    throw new AssertionError();
            }
            increment = false;
            return increment ? div + signum : div;
        }
        throw new ArithmeticException("/ by zero");
    }

    public static int mod(int x, int m) {
        if (m > 0) {
            int result = x % m;
            return result >= 0 ? result : result + m;
        }
        throw new ArithmeticException("Modulus " + m + " must be > 0");
    }

    public static int gcd(int a, int b) {
        MathPreconditions.checkNonNegative("a", a);
        MathPreconditions.checkNonNegative("b", b);
        while (b != 0) {
            int t = b;
            b = a % b;
            a = t;
        }
        return a;
    }

    public static int checkedAdd(int a, int b) {
        long result = ((long) a) + ((long) b);
        MathPreconditions.checkNoOverflow(result == ((long) ((int) result)));
        return (int) result;
    }

    public static int checkedSubtract(int a, int b) {
        long result = ((long) a) - ((long) b);
        MathPreconditions.checkNoOverflow(result == ((long) ((int) result)));
        return (int) result;
    }

    public static int checkedMultiply(int a, int b) {
        long result = ((long) a) * ((long) b);
        MathPreconditions.checkNoOverflow(result == ((long) ((int) result)));
        return (int) result;
    }

    @GwtIncompatible("failing tests")
    public static int checkedPow(int b, int k) {
        MathPreconditions.checkNonNegative("exponent", k);
        boolean z = false;
        switch (b) {
            case -2:
                if (k < 32) {
                    z = true;
                }
                MathPreconditions.checkNoOverflow(z);
                return (k & 1) == 0 ? 1 << k : -1 << k;
            case -1:
                if ((k & 1) == 0) {
                    return 1;
                }
                return -1;
            case 0:
                if (k == 0) {
                    return 1;
                }
                return 0;
            case 1:
                return 1;
            case 2:
                if (k < 31) {
                    z = true;
                }
                MathPreconditions.checkNoOverflow(z);
                return 1 << k;
            default:
                int b2 = b;
                int accum = 1;
                while (true) {
                    switch (k) {
                        case 0:
                            return accum;
                        case 1:
                            return checkedMultiply(accum, b2);
                        default:
                            if ((k & 1) != 0) {
                                accum = checkedMultiply(accum, b2);
                            }
                            k >>= 1;
                            if (k > 0) {
                                MathPreconditions.checkNoOverflow((-46340 <= b2) & (b2 <= FLOOR_SQRT_MAX_INT));
                                b2 *= b2;
                            }
                    }
                }
        }
    }

    public static int factorial(int n) {
        MathPreconditions.checkNonNegative("n", n);
        if (n < FACTORIALS.length) {
            return FACTORIALS[n];
        }
        return Integer.MAX_VALUE;
    }

    @GwtIncompatible("need BigIntegerMath to adequately test")
    public static int binomial(int n, int k) {
        MathPreconditions.checkNonNegative("n", n);
        MathPreconditions.checkNonNegative("k", k);
        Preconditions.checkArgument(k <= n, "k (%s) > n (%s)", Integer.valueOf(k), Integer.valueOf(n));
        if (k > (n >> 1)) {
            k = n - k;
        }
        if (k >= BIGGEST_BINOMIALS.length || n > BIGGEST_BINOMIALS[k]) {
            return Integer.MAX_VALUE;
        }
        switch (k) {
            case 0:
                return 1;
            case 1:
                return n;
            default:
                long result = 1;
                for (int i = 0; i < k; i++) {
                    result = (result * ((long) (n - i))) / ((long) (i + 1));
                }
                return (int) result;
        }
    }

    private IntMath() {
    }
}
