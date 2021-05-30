package org.ros.internal.node.xmlrpc;

public class XmlRpcTimeoutException extends RuntimeException {
    public XmlRpcTimeoutException() {
    }

    public XmlRpcTimeoutException(Exception e) {
        super(e);
    }
}
