package org.apache.xmlrpc.client;

import org.apache.xmlrpc.XmlRpcRequest;

public interface AsyncCallback {
    void handleError(XmlRpcRequest xmlRpcRequest, Throwable th);

    void handleResult(XmlRpcRequest xmlRpcRequest, Object obj);
}
