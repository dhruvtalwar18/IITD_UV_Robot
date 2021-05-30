package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcHandler;

public interface XmlRpcHandlerMapping {
    XmlRpcHandler getHandler(String str) throws XmlRpcNoSuchHandlerException, XmlRpcException;
}
