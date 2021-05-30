package org.ros.exception;

public class DuplicateServiceException extends RosRuntimeException {
    public DuplicateServiceException(String message) {
        super(message);
    }
}
