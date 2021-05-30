package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcInvocationException extends XmlRpcException {
    private static final long serialVersionUID = 7439737967784966169L;

    public XmlRpcInvocationException(int pCode, String pMessage, Throwable pLinkedException) {
        super(pCode, pMessage, pLinkedException);
    }

    public XmlRpcInvocationException(String pMessage, Throwable pLinkedException) {
        super(pMessage, pLinkedException);
    }
}
