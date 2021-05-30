package org.apache.commons.io;

import java.io.IOException;

public class IOExceptionWithCause extends IOException {
    private static final long serialVersionUID = 1;

    public IOExceptionWithCause(String message, Throwable cause) {
        super(message);
        initCause(cause);
    }

    /* JADX INFO: super call moved to the top of the method (can break code semantics) */
    public IOExceptionWithCause(Throwable cause) {
        super(cause == null ? null : cause.toString());
        initCause(cause);
    }
}
