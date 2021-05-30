package org.apache.xmlrpc.client;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;

public interface XmlRpcTransport {
    Object sendRequest(XmlRpcRequest xmlRpcRequest) throws XmlRpcException;
}
