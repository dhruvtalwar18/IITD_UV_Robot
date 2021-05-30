package org.apache.xmlrpc;

public interface XmlRpcHandler {
    Object execute(XmlRpcRequest xmlRpcRequest) throws XmlRpcException;
}
