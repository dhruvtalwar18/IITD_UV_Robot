package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public abstract class ExtSerializer implements TypeSerializer {
    /* access modifiers changed from: protected */
    public abstract String getTagName();

    /* access modifiers changed from: protected */
    public abstract void serialize(ContentHandler contentHandler, Object obj) throws SAXException;

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        String tag = getTagName();
        String exTag = "ex:" + getTagName();
        pHandler.startElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.ZERO_ATTRIBUTES);
        pHandler.startElement(XmlRpcWriter.EXTENSIONS_URI, tag, exTag, TypeSerializerImpl.ZERO_ATTRIBUTES);
        serialize(pHandler, pObject);
        pHandler.endElement(XmlRpcWriter.EXTENSIONS_URI, tag, exTag);
        pHandler.endElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG);
    }
}
