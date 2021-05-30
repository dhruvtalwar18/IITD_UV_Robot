package org.apache.xmlrpc.serializer;

import java.io.OutputStream;
import java.io.StringWriter;
import org.apache.ws.commons.serialize.CharSetXMLWriter;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.xml.sax.ContentHandler;
import org.xml.sax.helpers.AttributesImpl;

public class DefaultXMLWriterFactory implements XmlWriterFactory {
    private final XmlWriterFactory factory;

    public DefaultXMLWriterFactory() {
        XmlWriterFactory xwf;
        try {
            CharSetXMLWriter csw = new CharSetXMLWriter();
            csw.setWriter(new StringWriter());
            csw.startDocument();
            csw.startElement("", "test", "test", new AttributesImpl());
            csw.endElement("", "test", "test");
            csw.endDocument();
            xwf = new CharSetXmlWriterFactory();
        } catch (Throwable th) {
            xwf = new BaseXmlWriterFactory();
        }
        this.factory = xwf;
    }

    public ContentHandler getXmlWriter(XmlRpcStreamConfig pConfig, OutputStream pStream) throws XmlRpcException {
        return this.factory.getXmlWriter(pConfig, pStream);
    }
}
