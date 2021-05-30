package org.apache.xmlrpc.client;

import org.apache.xmlrpc.common.XmlRpcStreamRequestProcessor;

public class XmlRpcLocalStreamTransportFactory extends XmlRpcStreamTransportFactory {
    private final XmlRpcStreamRequestProcessor server;

    public XmlRpcLocalStreamTransportFactory(XmlRpcClient pClient, XmlRpcStreamRequestProcessor pServer) {
        super(pClient);
        this.server = pServer;
    }

    public XmlRpcTransport getTransport() {
        return new XmlRpcLocalStreamTransport(getClient(), this.server);
    }
}
