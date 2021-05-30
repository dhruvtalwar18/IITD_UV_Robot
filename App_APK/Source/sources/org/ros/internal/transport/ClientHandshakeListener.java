package org.ros.internal.transport;

public interface ClientHandshakeListener {
    void onFailure(ConnectionHeader connectionHeader, String str);

    void onSuccess(ConnectionHeader connectionHeader, ConnectionHeader connectionHeader2);
}
