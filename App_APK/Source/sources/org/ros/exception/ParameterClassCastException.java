package org.ros.exception;

public class ParameterClassCastException extends RosRuntimeException {
    public ParameterClassCastException(String message) {
        super(message);
    }

    public ParameterClassCastException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public ParameterClassCastException(Throwable throwable) {
        super(throwable);
    }
}
