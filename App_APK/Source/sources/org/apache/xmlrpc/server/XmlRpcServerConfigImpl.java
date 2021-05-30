package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcConfigImpl;

public class XmlRpcServerConfigImpl extends XmlRpcConfigImpl implements XmlRpcServerConfig, XmlRpcHttpServerConfig {
    private boolean isEnabledForExceptions;
    private boolean isKeepAliveEnabled;

    public void setKeepAliveEnabled(boolean pKeepAliveEnabled) {
        this.isKeepAliveEnabled = pKeepAliveEnabled;
    }

    public boolean isKeepAliveEnabled() {
        return this.isKeepAliveEnabled;
    }

    public void setEnabledForExceptions(boolean pEnabledForExceptions) {
        this.isEnabledForExceptions = pEnabledForExceptions;
    }

    public boolean isEnabledForExceptions() {
        return this.isEnabledForExceptions;
    }
}
