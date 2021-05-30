package org.apache.xmlrpc.client;

import javax.net.ssl.SSLSocketFactory;

public class XmlRpcSun14HttpTransportFactory extends XmlRpcTransportFactoryImpl {
    private SSLSocketFactory sslSocketFactory;

    public XmlRpcSun14HttpTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public void setSSLSocketFactory(SSLSocketFactory pSocketFactory) {
        this.sslSocketFactory = pSocketFactory;
    }

    public SSLSocketFactory getSSLSocketFactory() {
        return this.sslSocketFactory;
    }

    public XmlRpcTransport getTransport() {
        XmlRpcSun14HttpTransport transport = new XmlRpcSun14HttpTransport(getClient());
        transport.setSSLSocketFactory(this.sslSocketFactory);
        return transport;
    }
}
