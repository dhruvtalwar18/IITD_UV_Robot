package org.apache.xmlrpc.serializer;

import java.util.Map;
import org.apache.xmlrpc.common.TypeFactory;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class MapSerializer extends TypeSerializerImpl {
    public static final String MEMBER_TAG = "member";
    public static final String NAME_TAG = "name";
    public static final String STRUCT_TAG = "struct";
    private final XmlRpcStreamConfig config;
    private final TypeFactory typeFactory;

    public MapSerializer(TypeFactory pTypeFactory, XmlRpcStreamConfig pConfig) {
        this.typeFactory = pTypeFactory;
        this.config = pConfig;
    }

    /* access modifiers changed from: protected */
    public void writeEntry(ContentHandler pHandler, Object pKey, Object pValue) throws SAXException {
        pHandler.startElement("", MEMBER_TAG, MEMBER_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement("", "name", "name", ZERO_ATTRIBUTES);
        if (!this.config.isEnabledForExtensions() || (pKey instanceof String)) {
            String key = pKey.toString();
            pHandler.characters(key.toCharArray(), 0, key.length());
        } else {
            writeValue(pHandler, pKey);
        }
        pHandler.endElement("", "name", "name");
        writeValue(pHandler, pValue);
        pHandler.endElement("", MEMBER_TAG, MEMBER_TAG);
    }

    private void writeValue(ContentHandler pHandler, Object pValue) throws SAXException {
        TypeSerializer ts = this.typeFactory.getSerializer(this.config, pValue);
        if (ts != null) {
            ts.write(pHandler, pValue);
            return;
        }
        throw new SAXException("Unsupported Java type: " + pValue.getClass().getName());
    }

    /* access modifiers changed from: protected */
    public void writeData(ContentHandler pHandler, Object pData) throws SAXException {
        for (Map.Entry entry : ((Map) pData).entrySet()) {
            writeEntry(pHandler, entry.getKey(), entry.getValue());
        }
    }

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        pHandler.startElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement("", STRUCT_TAG, STRUCT_TAG, ZERO_ATTRIBUTES);
        writeData(pHandler, pObject);
        pHandler.endElement("", STRUCT_TAG, STRUCT_TAG);
        pHandler.endElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG);
    }
}
