package com.google.common.hash;

import com.google.common.annotations.Beta;
import com.google.common.base.Ascii;
import com.google.common.base.Preconditions;
import com.google.common.primitives.Ints;
import java.security.MessageDigest;

@Beta
public abstract class HashCode {
    private static final char[] hexDigits = "0123456789abcdef".toCharArray();

    public abstract byte[] asBytes();

    public abstract int asInt();

    public abstract long asLong();

    public abstract int bits();

    HashCode() {
    }

    public int writeBytesTo(byte[] dest, int offset, int maxLength) {
        byte[] hash = asBytes();
        int maxLength2 = Ints.min(maxLength, hash.length);
        Preconditions.checkPositionIndexes(offset, offset + maxLength2, dest.length);
        System.arraycopy(hash, 0, dest, offset, maxLength2);
        return maxLength2;
    }

    public boolean equals(Object object) {
        if (object instanceof HashCode) {
            return MessageDigest.isEqual(asBytes(), ((HashCode) object).asBytes());
        }
        return false;
    }

    public int hashCode() {
        return asInt();
    }

    public String toString() {
        byte[] bytes = asBytes();
        StringBuilder sb = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) {
            sb.append(hexDigits[(b >> 4) & 15]);
            sb.append(hexDigits[b & Ascii.SI]);
        }
        return sb.toString();
    }
}
