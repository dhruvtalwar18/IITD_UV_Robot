package org.apache.xmlrpc.server;

import org.apache.xmlrpc.common.XmlRpcHttpConfig;

public interface XmlRpcHttpServerConfig extends XmlRpcServerConfig, XmlRpcHttpConfig {
    boolean isEnabledForExceptions();

    boolean isKeepAliveEnabled();
}
