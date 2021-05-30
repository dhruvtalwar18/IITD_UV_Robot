package org.apache.xmlrpc.serializer;

import java.io.BufferedWriter;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import org.apache.ws.commons.serialize.XMLWriter;
import org.apache.ws.commons.serialize.XMLWriterImpl;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.xml.sax.ContentHandler;

public class BaseXmlWriterFactory implements XmlWriterFactory {
    /* access modifiers changed from: protected */
    public XMLWriter newXmlWriter() {
        return new XMLWriterImpl();
    }

    public ContentHandler getXmlWriter(XmlRpcStreamConfig pConfig, OutputStream pStream) throws XmlRpcException {
        XMLWriter xw = newXmlWriter();
        xw.setDeclarating(true);
        String enc = pConfig.getEncoding();
        if (enc == null) {
            enc = "UTF-8";
        }
        xw.setEncoding(enc);
        xw.setIndenting(false);
        xw.setFlushing(true);
        try {
            xw.setWriter(new BufferedWriter(new OutputStreamWriter(pStream, enc)));
            return xw;
        } catch (UnsupportedEncodingException e) {
            throw new XmlRpcException("Unsupported encoding: " + enc, (Throwable) e);
        }
    }
}
