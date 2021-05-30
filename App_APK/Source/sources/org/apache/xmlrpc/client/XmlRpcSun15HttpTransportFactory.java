package org.apache.xmlrpc.client;

import java.net.InetSocketAddress;
import java.net.Proxy;

public class XmlRpcSun15HttpTransportFactory extends XmlRpcSun14HttpTransportFactory {
    private Proxy proxy;

    public XmlRpcSun15HttpTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public void setProxy(String proxyHost, int proxyPort) {
        setProxy(new Proxy(Proxy.Type.HTTP, new InetSocketAddress(proxyHost, proxyPort)));
    }

    public void setProxy(Proxy pProxy) {
        this.proxy = pProxy;
    }

    public XmlRpcTransport getTransport() {
        XmlRpcSun15HttpTransport transport = new XmlRpcSun15HttpTransport(getClient());
        transport.setSSLSocketFactory(getSSLSocketFactory());
        transport.setProxy(this.proxy);
        return transport;
    }
}
