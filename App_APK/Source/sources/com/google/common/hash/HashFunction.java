package com.google.common.hash;

import com.google.common.annotations.Beta;
import java.nio.charset.Charset;

@Beta
public interface HashFunction {
    int bits();

    HashCode hashBytes(byte[] bArr);

    HashCode hashBytes(byte[] bArr, int i, int i2);

    HashCode hashInt(int i);

    HashCode hashLong(long j);

    HashCode hashString(CharSequence charSequence);

    HashCode hashString(CharSequence charSequence, Charset charset);

    Hasher newHasher();

    Hasher newHasher(int i);
}
