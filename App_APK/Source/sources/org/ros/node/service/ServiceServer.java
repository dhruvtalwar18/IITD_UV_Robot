package org.ros.node.service;

import java.net.URI;
import org.ros.namespace.GraphName;

public interface ServiceServer<T, S> {
    void addListener(ServiceServerListener<T, S> serviceServerListener);

    GraphName getName();

    URI getUri();

    void shutdown();
}
