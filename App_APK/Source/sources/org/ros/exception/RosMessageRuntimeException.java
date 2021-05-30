package org.ros.exception;

public class RosMessageRuntimeException extends RuntimeException {
    public RosMessageRuntimeException(Throwable throwable) {
        super(throwable);
    }

    public RosMessageRuntimeException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public RosMessageRuntimeException(String message) {
        super(message);
    }
}
