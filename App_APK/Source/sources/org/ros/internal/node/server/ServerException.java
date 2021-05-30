package org.ros.internal.node.server;

public class ServerException extends Exception {
    private static final long serialVersionUID = 1;

    public ServerException(String message) {
        super(message);
    }

    public ServerException(Exception e) {
        super(e);
    }
}
