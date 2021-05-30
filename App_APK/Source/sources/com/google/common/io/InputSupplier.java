package com.google.common.io;

import java.io.IOException;

public interface InputSupplier<T> {
    T getInput() throws IOException;
}
