package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;

public interface XmlRpcStreamRequestProcessor extends XmlRpcRequestProcessor {
    void execute(XmlRpcStreamRequestConfig xmlRpcStreamRequestConfig, ServerStreamConnection serverStreamConnection) throws XmlRpcException;
}
