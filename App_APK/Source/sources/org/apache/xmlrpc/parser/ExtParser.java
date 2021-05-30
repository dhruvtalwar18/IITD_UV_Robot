package org.apache.xmlrpc.parser;

import java.util.ArrayList;
import java.util.List;
import javax.xml.namespace.QName;
import org.apache.xmlrpc.serializer.XmlRpcWriter;
import org.xml.sax.Attributes;
import org.xml.sax.ContentHandler;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public abstract class ExtParser implements TypeParser {
    private ContentHandler handler;
    private int level = 0;
    private Locator locator;
    private final List prefixes = new ArrayList();

    /* access modifiers changed from: protected */
    public abstract ContentHandler getExtHandler() throws SAXException;

    /* access modifiers changed from: protected */
    public abstract String getTagName();

    public void endDocument() throws SAXException {
    }

    public void startDocument() throws SAXException {
    }

    public void characters(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (this.handler != null) {
            this.handler.characters(pChars, pOffset, pLength);
        } else if (!TypeParserImpl.isEmpty(pChars, pOffset, pLength)) {
            throw new SAXParseException("Unexpected non-whitespace content: " + new String(pChars, pOffset, pLength), this.locator);
        }
    }

    public void ignorableWhitespace(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (this.handler != null) {
            ignorableWhitespace(pChars, pOffset, pLength);
        }
    }

    public void endPrefixMapping(String pPrefix) throws SAXException {
        if (this.handler != null) {
            this.handler.endPrefixMapping(pPrefix);
        }
    }

    public void skippedEntity(String pName) throws SAXException {
        if (this.handler != null) {
            this.handler.skippedEntity(pName);
            return;
        }
        throw new SAXParseException("Don't know how to handle entity " + pName, this.locator);
    }

    public void setDocumentLocator(Locator pLocator) {
        this.locator = pLocator;
        if (this.handler != null) {
            this.handler.setDocumentLocator(pLocator);
        }
    }

    public void processingInstruction(String pTarget, String pData) throws SAXException {
        if (this.handler != null) {
            this.handler.processingInstruction(pTarget, pData);
        }
    }

    public void startPrefixMapping(String pPrefix, String pURI) throws SAXException {
        if (this.handler == null) {
            this.prefixes.add(pPrefix);
            this.prefixes.add(pURI);
            return;
        }
        this.handler.startPrefixMapping(pPrefix, pURI);
    }

    public void startElement(String pURI, String pLocalName, String pQName, Attributes pAttrs) throws SAXException {
        int i = this.level;
        this.level = i + 1;
        if (i != 0) {
            this.handler.startElement(pURI, pLocalName, pQName, pAttrs);
            return;
        }
        String tag = getTagName();
        if (!XmlRpcWriter.EXTENSIONS_URI.equals(pURI) || !tag.equals(pLocalName)) {
            throw new SAXParseException("Expected " + new QName(XmlRpcWriter.EXTENSIONS_URI, tag) + ", got " + new QName(pURI, pLocalName), this.locator);
        }
        this.handler = getExtHandler();
        this.handler.startDocument();
        for (int i2 = 0; i2 < this.prefixes.size(); i2 += 2) {
            this.handler.startPrefixMapping((String) this.prefixes.get(i2), (String) this.prefixes.get(i2 + 1));
        }
    }

    public void endElement(String pURI, String pLocalName, String pQName) throws SAXException {
        int i = this.level - 1;
        this.level = i;
        if (i != 0) {
            this.handler.endElement(pURI, pLocalName, pQName);
            return;
        }
        for (int i2 = 0; i2 < this.prefixes.size(); i2 += 2) {
            this.handler.endPrefixMapping((String) this.prefixes.get(i2));
        }
        this.handler.endDocument();
        this.handler = null;
    }
}
