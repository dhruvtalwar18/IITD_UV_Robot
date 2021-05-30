package org.apache.xmlrpc.util;

import java.io.IOException;

public class XmlRpcIOException extends IOException {
    private static final long serialVersionUID = -7704704099502077919L;
    private final Throwable linkedException;

    public XmlRpcIOException(Throwable t) {
        super(t.getMessage());
        this.linkedException = t;
    }

    public Throwable getLinkedException() {
        return this.linkedException;
    }
}
