package org.apache.xmlrpc.parser;

import org.apache.xmlrpc.XmlRpcException;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public abstract class TypeParserImpl implements TypeParser {
    private Locator locator;
    private Object result;

    public void setResult(Object pResult) {
        this.result = pResult;
    }

    public Object getResult() throws XmlRpcException {
        return this.result;
    }

    public Locator getDocumentLocator() {
        return this.locator;
    }

    public void setDocumentLocator(Locator pLocator) {
        this.locator = pLocator;
    }

    public void processingInstruction(String pTarget, String pData) throws SAXException {
    }

    public void skippedEntity(String pName) throws SAXException {
        throw new SAXParseException("Don't know how to handle entity " + pName, getDocumentLocator());
    }

    public void startPrefixMapping(String pPrefix, String pURI) throws SAXException {
    }

    public void endPrefixMapping(String pPrefix) throws SAXException {
    }

    public void endDocument() throws SAXException {
    }

    public void startDocument() throws SAXException {
    }

    protected static boolean isEmpty(char[] pChars, int pStart, int pLength) {
        for (int i = 0; i < pLength; i++) {
            if (!Character.isWhitespace(pChars[pStart + i])) {
                return false;
            }
        }
        return true;
    }

    public void characters(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (!isEmpty(pChars, pOffset, pLength)) {
            throw new SAXParseException("Unexpected non-whitespace character data", getDocumentLocator());
        }
    }

    public void ignorableWhitespace(char[] pChars, int pOffset, int pLength) throws SAXException {
    }
}
