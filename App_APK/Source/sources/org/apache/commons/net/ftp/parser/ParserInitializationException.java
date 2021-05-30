package org.apache.commons.net.ftp.parser;

public class ParserInitializationException extends RuntimeException {
    private final Throwable rootCause;

    public ParserInitializationException(String message) {
        super(message);
        this.rootCause = null;
    }

    public ParserInitializationException(String message, Throwable rootCause2) {
        super(message);
        this.rootCause = rootCause2;
    }

    public Throwable getRootCause() {
        return this.rootCause;
    }
}
