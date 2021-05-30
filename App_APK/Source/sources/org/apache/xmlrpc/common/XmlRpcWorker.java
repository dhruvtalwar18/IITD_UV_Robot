package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;

public interface XmlRpcWorker {
    Object execute(XmlRpcRequest xmlRpcRequest) throws XmlRpcException;

    XmlRpcController getController();
}
