package org.apache.xmlrpc.jaxb;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.UnmarshallerHandler;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.parser.ExtParser;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class JaxbParser extends ExtParser {
    private final JAXBContext context;
    private UnmarshallerHandler handler;

    public JaxbParser(JAXBContext pContext) {
        this.context = pContext;
    }

    /* access modifiers changed from: protected */
    public ContentHandler getExtHandler() throws SAXException {
        try {
            this.handler = this.context.createUnmarshaller().getUnmarshallerHandler();
            return this.handler;
        } catch (JAXBException e) {
            throw new SAXException(e);
        }
    }

    /* access modifiers changed from: protected */
    public String getTagName() {
        return JaxbSerializer.JAXB_TAG;
    }

    public Object getResult() throws XmlRpcException {
        try {
            return this.handler.getResult();
        } catch (JAXBException e) {
            throw new XmlRpcException("Failed to create result object: " + e.getMessage(), (Throwable) e);
        }
    }
}
