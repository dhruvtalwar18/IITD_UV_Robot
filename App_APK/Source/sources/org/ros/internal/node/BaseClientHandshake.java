package org.ros.internal.node;

import org.ros.internal.transport.ClientHandshake;
import org.ros.internal.transport.ConnectionHeader;

public abstract class BaseClientHandshake implements ClientHandshake {
    private String errorMessage;
    protected final ConnectionHeader outgoingConnectionHeader;

    public BaseClientHandshake(ConnectionHeader outgoingConnectionHeader2) {
        this.outgoingConnectionHeader = outgoingConnectionHeader2;
    }

    public ConnectionHeader getOutgoingConnectionHeader() {
        return this.outgoingConnectionHeader;
    }

    public String getErrorMessage() {
        return this.errorMessage;
    }

    /* access modifiers changed from: protected */
    public void setErrorMessage(String errorMessage2) {
        this.errorMessage = errorMessage2;
    }
}
