package com.google.common.primitives;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Preconditions;
import java.math.BigInteger;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
@Beta
public final class UnsignedInteger extends Number implements Comparable<UnsignedInteger> {
    public static final UnsignedInteger MAX_VALUE = asUnsigned(-1);
    public static final UnsignedInteger ONE = asUnsigned(1);
    public static final UnsignedInteger ZERO = asUnsigned(0);
    private final int value;

    private UnsignedInteger(int value2) {
        this.value = value2 & -1;
    }

    public static UnsignedInteger asUnsigned(int value2) {
        return new UnsignedInteger(value2);
    }

    public static UnsignedInteger valueOf(long value2) {
        Preconditions.checkArgument((4294967295L & value2) == value2, "value (%s) is outside the range for an unsigned integer value", Long.valueOf(value2));
        return asUnsigned((int) value2);
    }

    public static UnsignedInteger valueOf(BigInteger value2) {
        Preconditions.checkNotNull(value2);
        Preconditions.checkArgument(value2.signum() >= 0 && value2.bitLength() <= 32, "value (%s) is outside the range for an unsigned integer value", value2);
        return asUnsigned(value2.intValue());
    }

    public static UnsignedInteger valueOf(String string) {
        return valueOf(string, 10);
    }

    public static UnsignedInteger valueOf(String string, int radix) {
        return asUnsigned(UnsignedInts.parseUnsignedInt(string, radix));
    }

    public UnsignedInteger add(UnsignedInteger val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(this.value + val.value);
    }

    public UnsignedInteger subtract(UnsignedInteger val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(this.value - val.value);
    }

    @GwtIncompatible("Does not truncate correctly")
    public UnsignedInteger multiply(UnsignedInteger val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(this.value * val.value);
    }

    public UnsignedInteger divide(UnsignedInteger val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(UnsignedInts.divide(this.value, val.value));
    }

    public UnsignedInteger remainder(UnsignedInteger val) {
        Preconditions.checkNotNull(val);
        return asUnsigned(UnsignedInts.remainder(this.value, val.value));
    }

    public int intValue() {
        return this.value;
    }

    public long longValue() {
        return UnsignedInts.toLong(this.value);
    }

    public float floatValue() {
        return (float) longValue();
    }

    public double doubleValue() {
        return (double) longValue();
    }

    public BigInteger bigIntegerValue() {
        return BigInteger.valueOf(longValue());
    }

    public int compareTo(UnsignedInteger other) {
        Preconditions.checkNotNull(other);
        return UnsignedInts.compare(this.value, other.value);
    }

    public int hashCode() {
        return this.value;
    }

    public boolean equals(@Nullable Object obj) {
        if (!(obj instanceof UnsignedInteger) || this.value != ((UnsignedInteger) obj).value) {
            return false;
        }
        return true;
    }

    public String toString() {
        return toString(10);
    }

    public String toString(int radix) {
        return UnsignedInts.toString(this.value, radix);
    }
}
