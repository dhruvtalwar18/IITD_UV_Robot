package org.ros.exception;

import org.ros.internal.node.response.StatusCode;

public class RemoteException extends RosRuntimeException {
    private final StatusCode statusCode;

    public RemoteException(StatusCode statusCode2, String message) {
        super(message);
        this.statusCode = statusCode2;
    }

    public StatusCode getStatusCode() {
        return this.statusCode;
    }
}
