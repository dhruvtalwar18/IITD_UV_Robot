package org.apache.commons.httpclient;

public class DefaultMethodRetryHandler implements MethodRetryHandler {
    private boolean requestSentRetryEnabled = false;
    private int retryCount = 3;

    public boolean retryMethod(HttpMethod method, HttpConnection connection, HttpRecoverableException recoverableException, int executionCount, boolean requestSent) {
        return (!requestSent || this.requestSentRetryEnabled) && executionCount <= this.retryCount;
    }

    public boolean isRequestSentRetryEnabled() {
        return this.requestSentRetryEnabled;
    }

    public int getRetryCount() {
        return this.retryCount;
    }

    public void setRequestSentRetryEnabled(boolean requestSentRetryEnabled2) {
        this.requestSentRetryEnabled = requestSentRetryEnabled2;
    }

    public void setRetryCount(int retryCount2) {
        this.retryCount = retryCount2;
    }
}
