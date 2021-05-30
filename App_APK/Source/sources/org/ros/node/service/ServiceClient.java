package org.ros.node.service;

import java.net.URI;
import org.ros.namespace.GraphName;

public interface ServiceClient<T, S> {
    void call(T t, ServiceResponseListener<S> serviceResponseListener);

    void connect(URI uri);

    GraphName getName();

    boolean isConnected();

    T newMessage();

    void shutdown();
}
