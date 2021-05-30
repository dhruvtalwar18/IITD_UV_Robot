package org.ros.exception;

public class RosException extends Exception {
    public RosException(Throwable throwable) {
        super(throwable);
    }

    public RosException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public RosException(String message) {
        super(message);
    }
}
