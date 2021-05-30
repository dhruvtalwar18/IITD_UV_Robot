package org.apache.xmlrpc.parser;

import java.util.HashMap;
import java.util.Map;
import javax.xml.namespace.QName;
import org.apache.ws.commons.util.NamespaceContextImpl;
import org.apache.xmlrpc.common.TypeFactory;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.apache.xmlrpc.serializer.MapSerializer;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class MapParser extends RecursiveTypeParserImpl {
    private boolean doneValue;
    private boolean inName;
    private boolean inValue;
    private int level = 0;
    private Map map;
    private StringBuffer nameBuffer = new StringBuffer();
    private Object nameObject;

    public MapParser(XmlRpcStreamConfig pConfig, NamespaceContextImpl pContext, TypeFactory pFactory) {
        super(pConfig, pContext, pFactory);
    }

    /* access modifiers changed from: protected */
    public void addResult(Object pResult) throws SAXException {
        if (this.inName) {
            this.nameObject = pResult;
        } else if (this.nameObject == null) {
            throw new SAXParseException("Invalid state: Expected name", getDocumentLocator());
        } else if (!this.map.containsKey(this.nameObject)) {
            this.map.put(this.nameObject, pResult);
        } else {
            throw new SAXParseException("Duplicate name: " + this.nameObject, getDocumentLocator());
        }
    }

    public void startDocument() throws SAXException {
        super.startDocument();
        this.level = 0;
        this.map = new HashMap();
        this.inName = false;
        this.inValue = false;
    }

    public void characters(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (!this.inName || this.inValue) {
            super.characters(pChars, pOffset, pLength);
        } else {
            this.nameBuffer.append(pChars, pOffset, pLength);
        }
    }

    public void ignorableWhitespace(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (this.inName) {
            characters(pChars, pOffset, pLength);
        } else {
            super.ignorableWhitespace(pChars, pOffset, pLength);
        }
    }

    public void startElement(String pURI, String pLocalName, String pQName, Attributes pAttrs) throws SAXException {
        int i = this.level;
        this.level = i + 1;
        switch (i) {
            case 0:
                if (!"".equals(pURI) || !MapSerializer.STRUCT_TAG.equals(pLocalName)) {
                    throw new SAXParseException("Expected struct, got " + new QName(pURI, pLocalName), getDocumentLocator());
                }
                return;
            case 1:
                if (!"".equals(pURI) || !MapSerializer.MEMBER_TAG.equals(pLocalName)) {
                    throw new SAXParseException("Expected member, got " + new QName(pURI, pLocalName), getDocumentLocator());
                }
                this.inValue = false;
                this.inName = false;
                this.doneValue = false;
                this.nameObject = null;
                this.nameBuffer.setLength(0);
                return;
            case 2:
                if (this.doneValue) {
                    throw new SAXParseException("Expected /member, got " + new QName(pURI, pLocalName), getDocumentLocator());
                } else if (!"".equals(pURI) || !"name".equals(pLocalName)) {
                    if ("".equals(pURI) && TypeSerializerImpl.VALUE_TAG.equals(pLocalName)) {
                        if (this.nameObject != null) {
                            this.inValue = true;
                            startValueTag();
                            return;
                        }
                        throw new SAXParseException("Expected name, got " + new QName(pURI, pLocalName), getDocumentLocator());
                    }
                    return;
                } else if (this.nameObject == null) {
                    this.inName = true;
                    return;
                } else {
                    throw new SAXParseException("Expected value, got " + new QName(pURI, pLocalName), getDocumentLocator());
                }
            case 3:
                if (!this.inName || !"".equals(pURI) || !TypeSerializerImpl.VALUE_TAG.equals(pLocalName)) {
                    super.startElement(pURI, pLocalName, pQName, pAttrs);
                    return;
                } else if (this.cfg.isEnabledForExtensions()) {
                    this.inValue = true;
                    startValueTag();
                    return;
                } else {
                    throw new SAXParseException("Expected /name, got " + new QName(pURI, pLocalName), getDocumentLocator());
                }
            default:
                super.startElement(pURI, pLocalName, pQName, pAttrs);
                return;
        }
    }

    public void endElement(String pURI, String pLocalName, String pQName) throws SAXException {
        int i = this.level - 1;
        this.level = i;
        switch (i) {
            case 0:
                setResult(this.map);
                return;
            case 1:
                return;
            case 2:
                if (this.inName) {
                    int i2 = 0;
                    this.inName = false;
                    if (this.nameObject == null) {
                        this.nameObject = this.nameBuffer.toString();
                        return;
                    }
                    while (i2 < this.nameBuffer.length()) {
                        if (Character.isWhitespace(this.nameBuffer.charAt(i2))) {
                            i2++;
                        } else {
                            throw new SAXParseException("Unexpected non-whitespace character in member name", getDocumentLocator());
                        }
                    }
                    return;
                } else if (this.inValue != 0) {
                    endValueTag();
                    this.doneValue = true;
                    return;
                } else {
                    return;
                }
            case 3:
                if (!this.inName || !this.inValue || !"".equals(pURI) || !TypeSerializerImpl.VALUE_TAG.equals(pLocalName)) {
                    super.endElement(pURI, pLocalName, pQName);
                    return;
                } else {
                    endValueTag();
                    return;
                }
            default:
                super.endElement(pURI, pLocalName, pQName);
                return;
        }
    }
}
