package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcRequestConfig;

public interface XmlRpcStreamRequestConfig extends XmlRpcStreamConfig, XmlRpcRequestConfig {
    boolean isEnabledForExceptions();

    boolean isGzipCompressing();

    boolean isGzipRequesting();
}
