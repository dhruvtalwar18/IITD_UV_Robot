package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class NullSerializer extends TypeSerializerImpl {
    public static final String EX_NIL_TAG = "ex:nil";
    public static final String NIL_TAG = "nil";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        pHandler.startElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement(XmlRpcWriter.EXTENSIONS_URI, NIL_TAG, EX_NIL_TAG, ZERO_ATTRIBUTES);
        pHandler.endElement(XmlRpcWriter.EXTENSIONS_URI, NIL_TAG, EX_NIL_TAG);
        pHandler.endElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG);
    }
}
