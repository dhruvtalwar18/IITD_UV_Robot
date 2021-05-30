package org.apache.xmlrpc.client;

public class XmlRpcLiteHttpTransportFactory extends XmlRpcTransportFactoryImpl {
    public XmlRpcLiteHttpTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public XmlRpcTransport getTransport() {
        return new XmlRpcLiteHttpTransport(getClient());
    }
}
