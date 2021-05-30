package org.apache.xmlrpc.client;

import org.apache.commons.httpclient.HttpClient;

public class XmlRpcCommonsTransportFactory extends XmlRpcTransportFactoryImpl {
    private HttpClient httpClient;

    public XmlRpcCommonsTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public XmlRpcTransport getTransport() {
        return new XmlRpcCommonsTransport(this);
    }

    public void setHttpClient(HttpClient pHttpClient) {
        this.httpClient = pHttpClient;
    }

    public HttpClient getHttpClient() {
        return this.httpClient;
    }
}
