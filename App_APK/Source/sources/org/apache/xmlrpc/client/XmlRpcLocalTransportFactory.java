package org.apache.xmlrpc.client;

public class XmlRpcLocalTransportFactory extends XmlRpcTransportFactoryImpl {
    private final XmlRpcTransport LOCAL_TRANSPORT = new XmlRpcLocalTransport(getClient());

    public XmlRpcLocalTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public XmlRpcTransport getTransport() {
        return this.LOCAL_TRANSPORT;
    }
}
