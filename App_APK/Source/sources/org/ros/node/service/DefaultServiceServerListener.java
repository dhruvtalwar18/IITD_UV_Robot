package org.ros.node.service;

public class DefaultServiceServerListener<T, S> implements ServiceServerListener<T, S> {
    public void onMasterRegistrationSuccess(ServiceServer<T, S> serviceServer) {
    }

    public void onMasterRegistrationFailure(ServiceServer<T, S> serviceServer) {
    }

    public void onMasterUnregistrationSuccess(ServiceServer<T, S> serviceServer) {
    }

    public void onMasterUnregistrationFailure(ServiceServer<T, S> serviceServer) {
    }

    public void onShutdown(ServiceServer<T, S> serviceServer) {
    }
}
