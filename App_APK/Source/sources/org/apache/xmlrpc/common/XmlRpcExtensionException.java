package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcExtensionException extends XmlRpcException {
    private static final long serialVersionUID = 3617014169594311221L;

    public XmlRpcExtensionException(String pMessage) {
        super(0, pMessage);
    }
}
