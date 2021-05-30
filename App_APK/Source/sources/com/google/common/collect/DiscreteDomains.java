package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import java.io.Serializable;
import java.math.BigInteger;

@GwtCompatible
@Beta
public final class DiscreteDomains {
    private DiscreteDomains() {
    }

    public static DiscreteDomain<Integer> integers() {
        return IntegerDomain.INSTANCE;
    }

    private static final class IntegerDomain extends DiscreteDomain<Integer> implements Serializable {
        /* access modifiers changed from: private */
        public static final IntegerDomain INSTANCE = new IntegerDomain();
        private static final long serialVersionUID = 0;

        private IntegerDomain() {
        }

        public Integer next(Integer value) {
            int i = value.intValue();
            if (i == Integer.MAX_VALUE) {
                return null;
            }
            return Integer.valueOf(i + 1);
        }

        public Integer previous(Integer value) {
            int i = value.intValue();
            if (i == Integer.MIN_VALUE) {
                return null;
            }
            return Integer.valueOf(i - 1);
        }

        public long distance(Integer start, Integer end) {
            return ((long) end.intValue()) - ((long) start.intValue());
        }

        public Integer minValue() {
            return Integer.MIN_VALUE;
        }

        public Integer maxValue() {
            return Integer.MAX_VALUE;
        }

        private Object readResolve() {
            return INSTANCE;
        }
    }

    public static DiscreteDomain<Long> longs() {
        return LongDomain.INSTANCE;
    }

    private static final class LongDomain extends DiscreteDomain<Long> implements Serializable {
        /* access modifiers changed from: private */
        public static final LongDomain INSTANCE = new LongDomain();
        private static final long serialVersionUID = 0;

        private LongDomain() {
        }

        public Long next(Long value) {
            long l = value.longValue();
            if (l == Long.MAX_VALUE) {
                return null;
            }
            return Long.valueOf(1 + l);
        }

        public Long previous(Long value) {
            long l = value.longValue();
            if (l == Long.MIN_VALUE) {
                return null;
            }
            return Long.valueOf(l - 1);
        }

        public long distance(Long start, Long end) {
            long result = end.longValue() - start.longValue();
            if (end.longValue() > start.longValue() && result < 0) {
                return Long.MAX_VALUE;
            }
            if (end.longValue() >= start.longValue() || result <= 0) {
                return result;
            }
            return Long.MIN_VALUE;
        }

        public Long minValue() {
            return Long.MIN_VALUE;
        }

        public Long maxValue() {
            return Long.MAX_VALUE;
        }

        private Object readResolve() {
            return INSTANCE;
        }
    }

    static DiscreteDomain<BigInteger> bigIntegers() {
        return BigIntegerDomain.INSTANCE;
    }

    private static final class BigIntegerDomain extends DiscreteDomain<BigInteger> implements Serializable {
        /* access modifiers changed from: private */
        public static final BigIntegerDomain INSTANCE = new BigIntegerDomain();
        private static final BigInteger MAX_LONG = BigInteger.valueOf(Long.MAX_VALUE);
        private static final BigInteger MIN_LONG = BigInteger.valueOf(Long.MIN_VALUE);
        private static final long serialVersionUID = 0;

        private BigIntegerDomain() {
        }

        public BigInteger next(BigInteger value) {
            return value.add(BigInteger.ONE);
        }

        public BigInteger previous(BigInteger value) {
            return value.subtract(BigInteger.ONE);
        }

        public long distance(BigInteger start, BigInteger end) {
            return start.subtract(end).max(MIN_LONG).min(MAX_LONG).longValue();
        }

        private Object readResolve() {
            return INSTANCE;
        }
    }
}
