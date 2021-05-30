package org.apache.xmlrpc.serializer;

import org.xml.sax.Attributes;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.AttributesImpl;

public abstract class TypeSerializerImpl implements TypeSerializer {
    public static final String VALUE_TAG = "value";
    protected static final Attributes ZERO_ATTRIBUTES = new AttributesImpl();

    /* access modifiers changed from: protected */
    public void write(ContentHandler pHandler, String pTagName, String pValue) throws SAXException {
        write(pHandler, pTagName, pValue.toCharArray());
    }

    /* access modifiers changed from: protected */
    public void write(ContentHandler pHandler, String pTagName, char[] pValue) throws SAXException {
        pHandler.startElement("", VALUE_TAG, VALUE_TAG, ZERO_ATTRIBUTES);
        if (pTagName != null) {
            pHandler.startElement("", pTagName, pTagName, ZERO_ATTRIBUTES);
        }
        pHandler.characters(pValue, 0, pValue.length);
        if (pTagName != null) {
            pHandler.endElement("", pTagName, pTagName);
        }
        pHandler.endElement("", VALUE_TAG, VALUE_TAG);
    }

    /* access modifiers changed from: protected */
    public void write(ContentHandler pHandler, String pLocalName, String pQName, String pValue) throws SAXException {
        pHandler.startElement("", VALUE_TAG, VALUE_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement(XmlRpcWriter.EXTENSIONS_URI, pLocalName, pQName, ZERO_ATTRIBUTES);
        char[] value = pValue.toCharArray();
        pHandler.characters(value, 0, value.length);
        pHandler.endElement(XmlRpcWriter.EXTENSIONS_URI, pLocalName, pQName);
        pHandler.endElement("", VALUE_TAG, VALUE_TAG);
    }
}
