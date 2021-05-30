package org.apache.xmlrpc.client;

public class XmlRpcSunHttpTransportFactory extends XmlRpcTransportFactoryImpl {
    public XmlRpcSunHttpTransportFactory(XmlRpcClient pClient) {
        super(pClient);
    }

    public XmlRpcTransport getTransport() {
        return new XmlRpcSunHttpTransport(getClient());
    }
}
