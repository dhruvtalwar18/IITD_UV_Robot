package com.google.common.hash;

import com.google.common.base.Charsets;
import java.nio.charset.Charset;

abstract class AbstractHasher implements Hasher {
    AbstractHasher() {
    }

    public final Hasher putBoolean(boolean b) {
        return putByte(b);
    }

    public final Hasher putDouble(double d) {
        return putLong(Double.doubleToRawLongBits(d));
    }

    public final Hasher putFloat(float f) {
        return putInt(Float.floatToRawIntBits(f));
    }

    public Hasher putString(CharSequence charSequence) {
        return putString(charSequence, Charsets.UTF_16LE);
    }

    public Hasher putString(CharSequence charSequence, Charset charset) {
        return putBytes(charSequence.toString().getBytes(charset));
    }
}
