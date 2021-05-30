package org.apache.xmlrpc.client;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.XmlRpcController;
import org.apache.xmlrpc.common.XmlRpcWorker;

public class XmlRpcClientWorker implements XmlRpcWorker {
    /* access modifiers changed from: private */
    public final XmlRpcClientWorkerFactory factory;

    public XmlRpcClientWorker(XmlRpcClientWorkerFactory pFactory) {
        this.factory = pFactory;
    }

    public XmlRpcController getController() {
        return this.factory.getController();
    }

    public Object execute(XmlRpcRequest pRequest) throws XmlRpcException {
        try {
            return ((XmlRpcClient) getController()).getTransportFactory().getTransport().sendRequest(pRequest);
        } finally {
            this.factory.releaseWorker(this);
        }
    }

    /* access modifiers changed from: protected */
    public Thread newThread(Runnable pRunnable) {
        Thread result = new Thread(pRunnable);
        result.setDaemon(true);
        return result;
    }

    public void execute(final XmlRpcRequest pRequest, final AsyncCallback pCallback) {
        newThread(new Runnable() {
            public void run() {
                Object result = null;
                Throwable th = null;
                try {
                    result = ((XmlRpcClient) XmlRpcClientWorker.this.getController()).getTransportFactory().getTransport().sendRequest(pRequest);
                } catch (Throwable t) {
                    th = t;
                }
                XmlRpcClientWorker.this.factory.releaseWorker(XmlRpcClientWorker.this);
                if (th == null) {
                    pCallback.handleResult(pRequest, result);
                } else {
                    pCallback.handleError(pRequest, th);
                }
            }
        }).start();
    }
}
