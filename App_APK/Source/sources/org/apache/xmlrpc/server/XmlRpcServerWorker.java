package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.XmlRpcController;
import org.apache.xmlrpc.common.XmlRpcWorker;

public class XmlRpcServerWorker implements XmlRpcWorker {
    private final XmlRpcServerWorkerFactory factory;

    public XmlRpcServerWorker(XmlRpcServerWorkerFactory pFactory) {
        this.factory = pFactory;
    }

    public XmlRpcController getController() {
        return this.factory.getController();
    }

    public Object execute(XmlRpcRequest pRequest) throws XmlRpcException {
        return ((XmlRpcServer) getController()).getHandlerMapping().getHandler(pRequest.getMethodName()).execute(pRequest);
    }
}
