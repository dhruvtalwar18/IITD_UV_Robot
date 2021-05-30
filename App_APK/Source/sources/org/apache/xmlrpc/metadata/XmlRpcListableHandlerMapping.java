package org.apache.xmlrpc.metadata;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.server.XmlRpcHandlerMapping;

public interface XmlRpcListableHandlerMapping extends XmlRpcHandlerMapping {
    String[] getListMethods() throws XmlRpcException;

    String getMethodHelp(String str) throws XmlRpcException;

    String[][] getMethodSignature(String str) throws XmlRpcException;
}
