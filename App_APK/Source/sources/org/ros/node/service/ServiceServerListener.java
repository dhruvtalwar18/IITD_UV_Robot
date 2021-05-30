package org.ros.node.service;

import org.ros.internal.node.RegistrantListener;

public interface ServiceServerListener<T, S> extends RegistrantListener<ServiceServer<T, S>> {
    void onShutdown(ServiceServer<T, S> serviceServer);
}
