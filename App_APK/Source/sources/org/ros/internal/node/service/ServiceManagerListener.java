package org.ros.internal.node.service;

public interface ServiceManagerListener {
    void onServiceServerAdded(DefaultServiceServer<?, ?> defaultServiceServer);

    void onServiceServerRemoved(DefaultServiceServer<?, ?> defaultServiceServer);
}
