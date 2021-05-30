package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;

public interface XmlRpcRequestProcessor {
    Object execute(XmlRpcRequest xmlRpcRequest) throws XmlRpcException;

    TypeConverterFactory getTypeConverterFactory();
}
