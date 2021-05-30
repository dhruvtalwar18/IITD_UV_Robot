package org.apache.xmlrpc.serializer;

import java.io.OutputStream;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.xml.sax.ContentHandler;

public interface XmlWriterFactory {
    ContentHandler getXmlWriter(XmlRpcStreamConfig xmlRpcStreamConfig, OutputStream outputStream) throws XmlRpcException;
}
