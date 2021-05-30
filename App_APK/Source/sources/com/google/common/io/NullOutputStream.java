package com.google.common.io;

import com.google.common.annotations.Beta;
import java.io.OutputStream;

@Beta
public final class NullOutputStream extends OutputStream {
    public void write(int b) {
    }

    public void write(byte[] b, int off, int len) {
    }
}
