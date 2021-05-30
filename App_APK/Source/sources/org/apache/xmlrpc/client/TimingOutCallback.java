package org.apache.xmlrpc.client;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;

public class TimingOutCallback implements AsyncCallback {
    private Throwable error;
    private boolean responseSeen;
    private Object result;
    private final long timeout;

    public static class TimeoutException extends XmlRpcException {
        private static final long serialVersionUID = 4875266372372105081L;

        public TimeoutException(int pCode, String message) {
            super(pCode, message);
        }
    }

    public TimingOutCallback(long pTimeout) {
        this.timeout = pTimeout;
    }

    public synchronized Object waitForResponse() throws Throwable {
        if (!this.responseSeen) {
            wait(this.timeout);
            if (!this.responseSeen) {
                throw new TimeoutException(0, "No response after waiting for " + this.timeout + " milliseconds.");
            }
        }
        if (this.error == null) {
        } else {
            throw this.error;
        }
        return this.result;
    }

    public synchronized void handleError(XmlRpcRequest pRequest, Throwable pError) {
        this.responseSeen = true;
        this.error = pError;
        notify();
    }

    public synchronized void handleResult(XmlRpcRequest pRequest, Object pResult) {
        this.responseSeen = true;
        this.result = pResult;
        notify();
    }
}
