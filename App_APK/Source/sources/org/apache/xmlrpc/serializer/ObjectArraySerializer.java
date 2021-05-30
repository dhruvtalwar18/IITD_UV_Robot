package org.apache.xmlrpc.serializer;

import org.apache.xmlrpc.common.TypeFactory;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class ObjectArraySerializer extends TypeSerializerImpl {
    public static final String ARRAY_TAG = "array";
    public static final String DATA_TAG = "data";
    private final XmlRpcStreamConfig config;
    private final TypeFactory typeFactory;

    public ObjectArraySerializer(TypeFactory pTypeFactory, XmlRpcStreamConfig pConfig) {
        this.typeFactory = pTypeFactory;
        this.config = pConfig;
    }

    /* access modifiers changed from: protected */
    public void writeObject(ContentHandler pHandler, Object pObject) throws SAXException {
        TypeSerializer ts = this.typeFactory.getSerializer(this.config, pObject);
        if (ts != null) {
            ts.write(pHandler, pObject);
            return;
        }
        throw new SAXException("Unsupported Java type: " + pObject.getClass().getName());
    }

    /* access modifiers changed from: protected */
    public void writeData(ContentHandler pHandler, Object pObject) throws SAXException {
        Object[] data = (Object[]) pObject;
        for (Object writeObject : data) {
            writeObject(pHandler, writeObject);
        }
    }

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        pHandler.startElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement("", ARRAY_TAG, ARRAY_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement("", DATA_TAG, DATA_TAG, ZERO_ATTRIBUTES);
        writeData(pHandler, pObject);
        pHandler.endElement("", DATA_TAG, DATA_TAG);
        pHandler.endElement("", ARRAY_TAG, ARRAY_TAG);
        pHandler.endElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG);
    }
}
