package org.ros.internal.transport;

public interface ClientHandshake {
    String getErrorMessage();

    ConnectionHeader getOutgoingConnectionHeader();

    boolean handshake(ConnectionHeader connectionHeader);
}
