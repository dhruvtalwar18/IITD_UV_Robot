package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcConfigImpl;

public class XmlRpcHttpRequestConfigImpl extends XmlRpcConfigImpl implements XmlRpcHttpRequestConfig {
    private String basicPassword;
    private String basicUserName;
    private int connectionTimeout = 0;
    private boolean enabledForExceptions;
    private boolean gzipCompressing;
    private boolean gzipRequesting;
    private int replyTimeout = 0;

    public void setGzipCompressing(boolean pCompressing) {
        this.gzipCompressing = pCompressing;
    }

    public boolean isGzipCompressing() {
        return this.gzipCompressing;
    }

    public void setGzipRequesting(boolean pRequesting) {
        this.gzipRequesting = pRequesting;
    }

    public boolean isGzipRequesting() {
        return this.gzipRequesting;
    }

    public void setBasicUserName(String pUser) {
        this.basicUserName = pUser;
    }

    public String getBasicUserName() {
        return this.basicUserName;
    }

    public void setBasicPassword(String pPassword) {
        this.basicPassword = pPassword;
    }

    public String getBasicPassword() {
        return this.basicPassword;
    }

    public void setConnectionTimeout(int pTimeout) {
        this.connectionTimeout = pTimeout;
    }

    public int getConnectionTimeout() {
        return this.connectionTimeout;
    }

    public void setReplyTimeout(int pTimeout) {
        this.replyTimeout = pTimeout;
    }

    public int getReplyTimeout() {
        return this.replyTimeout;
    }

    public void setEnabledForExceptions(boolean pEnabledForExceptions) {
        this.enabledForExceptions = pEnabledForExceptions;
    }

    public boolean isEnabledForExceptions() {
        return this.enabledForExceptions;
    }
}
