package org.apache.xmlrpc.client;

public abstract class XmlRpcTransportImpl implements XmlRpcTransport {
    private final XmlRpcClient client;

    protected XmlRpcTransportImpl(XmlRpcClient pClient) {
        this.client = pClient;
    }

    public XmlRpcClient getClient() {
        return this.client;
    }
}
