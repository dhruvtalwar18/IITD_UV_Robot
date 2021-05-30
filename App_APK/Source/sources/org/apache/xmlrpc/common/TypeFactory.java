package org.apache.xmlrpc.common;

import org.apache.ws.commons.util.NamespaceContextImpl;
import org.apache.xmlrpc.parser.TypeParser;
import org.apache.xmlrpc.serializer.TypeSerializer;
import org.xml.sax.SAXException;

public interface TypeFactory {
    TypeParser getParser(XmlRpcStreamConfig xmlRpcStreamConfig, NamespaceContextImpl namespaceContextImpl, String str, String str2);

    TypeSerializer getSerializer(XmlRpcStreamConfig xmlRpcStreamConfig, Object obj) throws SAXException;
}
