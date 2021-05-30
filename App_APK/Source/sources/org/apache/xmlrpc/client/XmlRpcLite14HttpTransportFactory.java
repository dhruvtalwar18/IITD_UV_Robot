package org.apache.xmlrpc.client;

import javax.net.ssl.SSLSocketFactory;

public class XmlRpcLite14HttpTransportFactory extends XmlRpcLiteHttpTransportFactory {
    private SSLSocketFactory sslSocketFactory;

    public XmlRpcLite14HttpTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public SSLSocketFactory getSSLSocketFactory() {
        return this.sslSocketFactory;
    }

    public void setSSLSocketFactory(SSLSocketFactory pSSLSocketFactory) {
        this.sslSocketFactory = pSSLSocketFactory;
    }

    public XmlRpcTransport getTransport() {
        XmlRpcLite14HttpTransport transport = new XmlRpcLite14HttpTransport(getClient());
        transport.setSSLSocketFactory(this.sslSocketFactory);
        return transport;
    }
}
