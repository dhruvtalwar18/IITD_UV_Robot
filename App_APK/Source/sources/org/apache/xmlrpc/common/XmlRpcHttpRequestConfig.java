package org.apache.xmlrpc.common;

public interface XmlRpcHttpRequestConfig extends XmlRpcStreamRequestConfig, XmlRpcHttpConfig {
    String getBasicPassword();

    String getBasicUserName();

    int getConnectionTimeout();

    int getReplyTimeout();
}
