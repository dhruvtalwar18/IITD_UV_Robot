package org.apache.xmlrpc.parser;

import javax.xml.namespace.QName;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public abstract class AtomicParser extends TypeParserImpl {
    private int level;
    protected StringBuffer sb;

    /* access modifiers changed from: protected */
    public abstract void setResult(String str) throws SAXException;

    protected AtomicParser() {
    }

    public void startDocument() throws SAXException {
        this.level = 0;
    }

    public void characters(char[] pChars, int pStart, int pLength) throws SAXException {
        if (this.sb != null) {
            this.sb.append(pChars, pStart, pLength);
        } else if (!isEmpty(pChars, pStart, pLength)) {
            throw new SAXParseException("Unexpected non-whitespace characters", getDocumentLocator());
        }
    }

    public void endElement(String pURI, String pLocalName, String pQName) throws SAXException {
        int i = this.level - 1;
        this.level = i;
        if (i == 0) {
            setResult(this.sb.toString());
            return;
        }
        throw new SAXParseException("Unexpected end tag in atomic element: " + new QName(pURI, pLocalName), getDocumentLocator());
    }

    public void startElement(String pURI, String pLocalName, String pQName, Attributes pAttrs) throws SAXException {
        int i = this.level;
        this.level = i + 1;
        if (i == 0) {
            this.sb = new StringBuffer();
            return;
        }
        throw new SAXParseException("Unexpected start tag in atomic element: " + new QName(pURI, pLocalName), getDocumentLocator());
    }
}
