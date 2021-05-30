package org.apache.xmlrpc.webserver;

import org.apache.xmlrpc.common.XmlRpcHttpRequestConfigImpl;

public class RequestData extends XmlRpcHttpRequestConfigImpl {
    private final Connection connection;
    private int contentLength = -1;
    private String httpVersion;
    private boolean keepAlive;
    private String method;
    private boolean success;

    public RequestData(Connection pConnection) {
        this.connection = pConnection;
    }

    public Connection getConnection() {
        return this.connection;
    }

    public boolean isKeepAlive() {
        return this.keepAlive;
    }

    public void setKeepAlive(boolean pKeepAlive) {
        this.keepAlive = pKeepAlive;
    }

    public String getHttpVersion() {
        return this.httpVersion;
    }

    public void setHttpVersion(String pHttpVersion) {
        this.httpVersion = pHttpVersion;
    }

    public int getContentLength() {
        return this.contentLength;
    }

    public void setContentLength(int pContentLength) {
        this.contentLength = pContentLength;
    }

    public boolean isByteArrayRequired() {
        return isKeepAlive() || !isEnabledForExtensions() || !isContentLengthOptional();
    }

    public String getMethod() {
        return this.method;
    }

    public void setMethod(String pMethod) {
        this.method = pMethod;
    }

    public boolean isSuccess() {
        return this.success;
    }

    public void setSuccess(boolean pSuccess) {
        this.success = pSuccess;
    }
}
