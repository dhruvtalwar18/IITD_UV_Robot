package org.ros.node.service;

import org.ros.exception.RemoteException;

public interface ServiceResponseListener<MessageType> {
    void onFailure(RemoteException remoteException);

    void onSuccess(MessageType messagetype);
}
