package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcNoSuchHandlerException extends XmlRpcException {
    private static final long serialVersionUID = 3257002138218344501L;

    public XmlRpcNoSuchHandlerException(String pMessage) {
        super(0, pMessage);
    }
}
