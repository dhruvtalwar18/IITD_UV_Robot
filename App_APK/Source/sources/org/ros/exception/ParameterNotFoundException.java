package org.ros.exception;

public class ParameterNotFoundException extends RosRuntimeException {
    public ParameterNotFoundException(String message) {
        super(message);
    }

    public ParameterNotFoundException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public ParameterNotFoundException(Throwable throwable) {
        super(throwable);
    }
}
