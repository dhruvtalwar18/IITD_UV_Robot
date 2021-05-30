package org.apache.xmlrpc.client;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcClientException extends XmlRpcException {
    private static final long serialVersionUID = 3545798797134608691L;

    public XmlRpcClientException(String pMessage, Throwable pCause) {
        super(0, pMessage, pCause);
    }
}
