package com.google.common.primitives;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import java.io.Serializable;
import java.math.BigInteger;
import javax.annotation.Nullable;

@GwtCompatible(serializable = true)
@Beta
public final class UnsignedLong extends Number implements Comparable<UnsignedLong>, Serializable {
    public static final UnsignedLong MAX_VALUE = new UnsignedLong(-1);
    public static final UnsignedLong ONE = new UnsignedLong(1);
    private static final long UNSIGNED_MASK = Long.MAX_VALUE;
    public static final UnsignedLong ZERO = new UnsignedLong(0);
    private final long value;

    private UnsignedLong(long value2) {
        this.value = value2;
    }

    public static UnsignedLong asUnsigned(long value2) {
        return new UnsignedLong(value2);
    }

    public static UnsignedLong valueOf(BigInteger value2) {
        Preconditions.checkNotNull(value2);
        Preconditions.checkArgument(value2.signum() >= 0 && value2.bitLength() <= 64, "value (%s) is outside the range for an unsigned long value", value2);
        return asUnsigned(value2.longValue());
    }

    public static UnsignedLong valueOf(String string) {
        return valueOf(string, 10);
    }

    public static UnsignedLong valueOf(String string, int radix) {
        return asUnsigned(UnsignedLongs.parseUnsignedLong(string, radix));
    }

    public UnsignedLong add(UnsignedLong val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(this.value + val.value);
    }

    public UnsignedLong subtract(UnsignedLong val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(this.value - val.value);
    }

    public UnsignedLong multiply(UnsignedLong val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(this.value * val.value);
    }

    public UnsignedLong divide(UnsignedLong val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(UnsignedLongs.divide(this.value, val.value));
    }

    public UnsignedLong remainder(UnsignedLong val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(UnsignedLongs.remainder(this.value, val.value));
    }

    public int intValue() {
        return (int) this.value;
    }

    public long longValue() {
        return this.value;
    }

    public float floatValue() {
        float fValue = (float) (this.value & UNSIGNED_MASK);
        if (this.value < 0) {
            return fValue + 9.223372E18f;
        }
        return fValue;
    }

    public double doubleValue() {
        double dValue = (double) (this.value & UNSIGNED_MASK);
        if (this.value >= 0) {
            return dValue;
        }
        Double.isNaN(dValue);
        return dValue + 9.223372036854776E18d;
    }

    public BigInteger bigIntegerValue() {
        BigInteger bigInt = BigInteger.valueOf(this.value & UNSIGNED_MASK);
        if (this.value < 0) {
            return bigInt.setBit(63);
        }
        return bigInt;
    }

    public int compareTo(UnsignedLong o) {
        Preconditions.checkNotNull(o);
        return UnsignedLongs.compare(this.value, o.value);
    }

    public int hashCode() {
        return Longs.hashCode(this.value);
    }

    public boolean equals(@Nullable Object obj) {
        if (!(obj instanceof UnsignedLong) || this.value != ((UnsignedLong) obj).value) {
            return false;
        }
        return true;
    }

    public String toString() {
        return UnsignedLongs.toString(this.value);
    }

    public String toString(int radix) {
        return UnsignedLongs.toString(this.value, radix);
    }
}
