package org.apache.xmlrpc.client;

public abstract class XmlRpcTransportFactoryImpl implements XmlRpcTransportFactory {
    private final XmlRpcClient client;

    protected XmlRpcTransportFactoryImpl(XmlRpcClient pClient) {
        this.client = pClient;
    }

    public XmlRpcClient getClient() {
        return this.client;
    }
}
