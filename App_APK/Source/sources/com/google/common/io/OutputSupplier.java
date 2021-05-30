package com.google.common.io;

import java.io.IOException;

public interface OutputSupplier<T> {
    T getOutput() throws IOException;
}
