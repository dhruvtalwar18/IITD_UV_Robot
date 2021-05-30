package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcLoadException extends XmlRpcException {
    private static final long serialVersionUID = 4050760511635272755L;

    public XmlRpcLoadException(String pMessage) {
        super(0, pMessage, (Throwable) null);
    }
}
