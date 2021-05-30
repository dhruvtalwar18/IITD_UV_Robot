package org.apache.commons.httpclient;

import org.apache.commons.httpclient.protocol.Protocol;

public class ProxyHost extends HttpHost {
    public ProxyHost(ProxyHost httpproxy) {
        super((HttpHost) httpproxy);
    }

    public ProxyHost(String hostname, int port) {
        super(hostname, port, Protocol.getProtocol("http"));
    }

    public ProxyHost(String hostname) {
        this(hostname, -1);
    }

    public Object clone() throws CloneNotSupportedException {
        return (ProxyHost) super.clone();
    }
}
