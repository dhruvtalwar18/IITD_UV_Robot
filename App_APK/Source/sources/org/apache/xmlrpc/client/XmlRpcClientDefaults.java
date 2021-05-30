package org.apache.xmlrpc.client;

import org.apache.xmlrpc.serializer.DefaultXMLWriterFactory;
import org.apache.xmlrpc.serializer.XmlWriterFactory;

public class XmlRpcClientDefaults {
    private static final XmlWriterFactory xmlWriterFactory = new DefaultXMLWriterFactory();

    public static XmlRpcTransportFactory newTransportFactory(XmlRpcClient pClient) {
        try {
            return new XmlRpcSun15HttpTransportFactory(pClient);
        } catch (Throwable th) {
            return new XmlRpcSunHttpTransportFactory(pClient);
        }
    }

    public static XmlRpcClientConfig newXmlRpcClientConfig() {
        return new XmlRpcClientConfigImpl();
    }

    public static XmlWriterFactory newXmlWriterFactory() {
        return xmlWriterFactory;
    }
}
