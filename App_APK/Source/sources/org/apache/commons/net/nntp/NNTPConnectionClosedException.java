package org.apache.commons.net.nntp;

import java.io.IOException;

public final class NNTPConnectionClosedException extends IOException {
    public NNTPConnectionClosedException() {
    }

    public NNTPConnectionClosedException(String message) {
        super(message);
    }
}
