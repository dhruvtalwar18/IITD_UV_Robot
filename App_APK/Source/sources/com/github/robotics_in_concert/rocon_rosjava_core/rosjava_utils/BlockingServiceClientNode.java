package com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.NodeNameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class BlockingServiceClientNode<RequestType, ResponseType> {
    /* access modifiers changed from: private */
    public String errorMessage;
    /* access modifiers changed from: private */
    public ResponseType response;
    /* access modifiers changed from: private */
    public ServiceClient<RequestType, ResponseType> srvClient;

    public BlockingServiceClientNode(ConnectedNode connectedNode, String serviceName, String serviceType, RequestType request) throws ServiceNotFoundException {
        try {
            this.srvClient = connectedNode.newServiceClient(NodeNameResolver.newRoot().resolve(serviceName).toString(), serviceType);
            this.srvClient.call(request, setupListener());
        } catch (ServiceNotFoundException e) {
            throw e;
        }
    }

    public void waitForResponse() {
        synchronized (this.srvClient) {
            try {
                if (this.response == null && this.errorMessage != "") {
                    this.srvClient.wait(4000);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private ServiceResponseListener<ResponseType> setupListener() {
        return new ServiceResponseListener<ResponseType>() {
            public void onSuccess(ResponseType r) {
                synchronized (BlockingServiceClientNode.this.srvClient) {
                    Object unused = BlockingServiceClientNode.this.response = r;
                    BlockingServiceClientNode.this.srvClient.notifyAll();
                }
            }

            public void onFailure(RemoteException e) {
                synchronized (BlockingServiceClientNode.this.srvClient) {
                    String unused = BlockingServiceClientNode.this.errorMessage = e.getMessage();
                    BlockingServiceClientNode.this.srvClient.notifyAll();
                }
            }
        };
    }

    public ResponseType getResponse() {
        return this.response;
    }

    public String getErrorMessage() {
        return this.errorMessage;
    }
}
