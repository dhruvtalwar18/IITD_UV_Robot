package org.apache.xmlrpc.metadata;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcHandler;

public interface XmlRpcMetaDataHandler extends XmlRpcHandler {
    String getMethodHelp() throws XmlRpcException;

    String[][] getSignatures() throws XmlRpcException;
}
