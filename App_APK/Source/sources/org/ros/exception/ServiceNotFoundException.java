package org.ros.exception;

public class ServiceNotFoundException extends RosException {
    public ServiceNotFoundException(Throwable throwable) {
        super(throwable);
    }

    public ServiceNotFoundException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public ServiceNotFoundException(String message) {
        super(message);
    }
}
