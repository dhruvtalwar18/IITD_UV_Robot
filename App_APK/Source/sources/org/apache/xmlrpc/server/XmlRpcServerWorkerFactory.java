package org.apache.xmlrpc.server;

import org.apache.xmlrpc.common.XmlRpcWorker;
import org.apache.xmlrpc.common.XmlRpcWorkerFactory;

public class XmlRpcServerWorkerFactory extends XmlRpcWorkerFactory {
    public XmlRpcServerWorkerFactory(XmlRpcServer pServer) {
        super(pServer);
    }

    /* access modifiers changed from: protected */
    public XmlRpcWorker newWorker() {
        return new XmlRpcServerWorker(this);
    }
}
