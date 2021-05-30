package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcNotAuthorizedException extends XmlRpcException {
    private static final long serialVersionUID = 3258410629709574201L;

    public XmlRpcNotAuthorizedException(String pMessage) {
        super(0, pMessage);
    }
}
