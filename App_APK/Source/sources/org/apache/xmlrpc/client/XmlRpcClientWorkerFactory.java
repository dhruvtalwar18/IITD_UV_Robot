package org.apache.xmlrpc.client;

import org.apache.xmlrpc.common.XmlRpcWorker;
import org.apache.xmlrpc.common.XmlRpcWorkerFactory;

public class XmlRpcClientWorkerFactory extends XmlRpcWorkerFactory {
    public XmlRpcClientWorkerFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    /* access modifiers changed from: protected */
    public XmlRpcWorker newWorker() {
        return new XmlRpcClientWorker(this);
    }
}
