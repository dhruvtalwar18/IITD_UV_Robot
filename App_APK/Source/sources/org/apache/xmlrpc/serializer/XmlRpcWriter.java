package org.apache.xmlrpc.serializer;

import java.io.ByteArrayOutputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;
import java.util.Map;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.XmlRpcRequestConfig;
import org.apache.xmlrpc.common.TypeFactory;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.xml.sax.Attributes;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.AttributesImpl;

public class XmlRpcWriter {
    public static final String EXTENSIONS_URI = "http://ws.apache.org/xmlrpc/namespaces/extensions";
    private static final Attributes ZERO_ATTRIBUTES = new AttributesImpl();
    private final XmlRpcStreamConfig config;
    private final ContentHandler handler;
    private final TypeFactory typeFactory;

    public XmlRpcWriter(XmlRpcStreamConfig pConfig, ContentHandler pHandler, TypeFactory pTypeFactory) {
        this.config = pConfig;
        this.handler = pHandler;
        this.typeFactory = pTypeFactory;
    }

    public void write(XmlRpcRequest pRequest) throws SAXException {
        this.handler.startDocument();
        boolean extensions = pRequest.getConfig().isEnabledForExtensions();
        if (extensions) {
            this.handler.startPrefixMapping("ex", EXTENSIONS_URI);
        }
        this.handler.startElement("", "methodCall", "methodCall", ZERO_ATTRIBUTES);
        this.handler.startElement("", "methodName", "methodName", ZERO_ATTRIBUTES);
        String s = pRequest.getMethodName();
        int i = 0;
        this.handler.characters(s.toCharArray(), 0, s.length());
        this.handler.endElement("", "methodName", "methodName");
        this.handler.startElement("", "params", "params", ZERO_ATTRIBUTES);
        int num = pRequest.getParameterCount();
        while (true) {
            int i2 = i;
            if (i2 >= num) {
                break;
            }
            this.handler.startElement("", "param", "param", ZERO_ATTRIBUTES);
            writeValue(pRequest.getParameter(i2));
            this.handler.endElement("", "param", "param");
            i = i2 + 1;
        }
        this.handler.endElement("", "params", "params");
        this.handler.endElement("", "methodCall", "methodCall");
        if (extensions) {
            this.handler.endPrefixMapping("ex");
        }
        this.handler.endDocument();
    }

    public void write(XmlRpcRequestConfig pConfig, Object pResult) throws SAXException {
        this.handler.startDocument();
        boolean extensions = pConfig.isEnabledForExtensions();
        if (extensions) {
            this.handler.startPrefixMapping("ex", EXTENSIONS_URI);
        }
        this.handler.startElement("", "methodResponse", "methodResponse", ZERO_ATTRIBUTES);
        this.handler.startElement("", "params", "params", ZERO_ATTRIBUTES);
        this.handler.startElement("", "param", "param", ZERO_ATTRIBUTES);
        writeValue(pResult);
        this.handler.endElement("", "param", "param");
        this.handler.endElement("", "params", "params");
        this.handler.endElement("", "methodResponse", "methodResponse");
        if (extensions) {
            this.handler.endPrefixMapping("ex");
        }
        this.handler.endDocument();
    }

    public void write(XmlRpcRequestConfig pConfig, int pCode, String pMessage) throws SAXException {
        write(pConfig, pCode, pMessage, (Throwable) null);
    }

    public void write(XmlRpcRequestConfig pConfig, int pCode, String pMessage, Throwable pThrowable) throws SAXException {
        this.handler.startDocument();
        boolean extensions = pConfig.isEnabledForExtensions();
        if (extensions) {
            this.handler.startPrefixMapping("ex", EXTENSIONS_URI);
        }
        this.handler.startElement("", "methodResponse", "methodResponse", ZERO_ATTRIBUTES);
        this.handler.startElement("", "fault", "fault", ZERO_ATTRIBUTES);
        Map map = new HashMap();
        map.put("faultCode", new Integer(pCode));
        map.put("faultString", pMessage == null ? "" : pMessage);
        if (pThrowable != null && extensions && (pConfig instanceof XmlRpcStreamRequestConfig) && ((XmlRpcStreamRequestConfig) pConfig).isEnabledForExceptions()) {
            try {
                ByteArrayOutputStream baos = new ByteArrayOutputStream();
                ObjectOutputStream oos = new ObjectOutputStream(baos);
                oos.writeObject(pThrowable);
                oos.close();
                baos.close();
                map.put("faultCause", baos.toByteArray());
            } catch (Throwable th) {
            }
        }
        writeValue(map);
        this.handler.endElement("", "fault", "fault");
        this.handler.endElement("", "methodResponse", "methodResponse");
        if (extensions) {
            this.handler.endPrefixMapping("ex");
        }
        this.handler.endDocument();
    }

    /* access modifiers changed from: protected */
    public void writeValue(Object pObject) throws SAXException {
        TypeSerializer serializer = this.typeFactory.getSerializer(this.config, pObject);
        if (serializer != null) {
            serializer.write(this.handler, pObject);
            return;
        }
        throw new SAXException("Unsupported Java type: " + pObject.getClass().getName());
    }
}
