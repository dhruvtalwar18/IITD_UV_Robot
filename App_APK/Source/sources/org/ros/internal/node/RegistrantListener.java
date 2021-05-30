package org.ros.internal.node;

public interface RegistrantListener<T> {
    void onMasterRegistrationFailure(T t);

    void onMasterRegistrationSuccess(T t);

    void onMasterUnregistrationFailure(T t);

    void onMasterUnregistrationSuccess(T t);
}
