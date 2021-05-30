package org.apache.xmlrpc.client;

import org.apache.xmlrpc.XmlRpcException;

public class XmlRpcHttpTransportException extends XmlRpcException {
    private static final long serialVersionUID = -6933992871198450027L;
    private final int status;
    private final String statusMessage;

    public XmlRpcHttpTransportException(int pCode, String pMessage) {
        this(pCode, pMessage, "HTTP server returned unexpected status: " + pMessage);
    }

    public XmlRpcHttpTransportException(int httpStatusCode, String httpStatusMessage, String message) {
        super(message);
        this.status = httpStatusCode;
        this.statusMessage = httpStatusMessage;
    }

    public int getStatusCode() {
        return this.status;
    }

    public String getStatusMessage() {
        return this.statusMessage;
    }
}
