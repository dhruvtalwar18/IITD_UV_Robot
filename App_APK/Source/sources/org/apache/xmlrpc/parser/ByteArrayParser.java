package org.apache.xmlrpc.parser;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import javax.xml.namespace.QName;
import org.apache.ws.commons.util.Base64;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class ByteArrayParser extends TypeParserImpl {
    /* access modifiers changed from: private */
    public ByteArrayOutputStream baos;
    private Base64.Decoder decoder;
    private int level;

    public void startDocument() throws SAXException {
        this.level = 0;
    }

    public void characters(char[] pChars, int pStart, int pLength) throws SAXException {
        if (this.baos != null) {
            try {
                this.decoder.write(pChars, pStart, pLength);
            } catch (IOException e) {
                throw new SAXParseException("Failed to decode base64 stream.", getDocumentLocator(), e);
            }
        } else if (!isEmpty(pChars, pStart, pLength)) {
            throw new SAXParseException("Unexpected non-whitespace characters", getDocumentLocator());
        }
    }

    public void endElement(String pURI, String pLocalName, String pQName) throws SAXException {
        int i = this.level - 1;
        this.level = i;
        if (i == 0) {
            try {
                this.decoder.flush();
                setResult(this.baos.toByteArray());
            } catch (IOException e) {
                throw new SAXParseException("Failed to decode base64 stream.", getDocumentLocator(), e);
            }
        } else {
            throw new SAXParseException("Unexpected end tag in atomic element: " + new QName(pURI, pLocalName), getDocumentLocator());
        }
    }

    public void startElement(String pURI, String pLocalName, String pQName, Attributes pAttrs) throws SAXException {
        int i = this.level;
        this.level = i + 1;
        if (i == 0) {
            this.baos = new ByteArrayOutputStream();
            this.decoder = new Base64.Decoder(1024) {
                /* access modifiers changed from: protected */
                public void writeBuffer(byte[] pBytes, int pOffset, int pLen) throws IOException {
                    ByteArrayParser.this.baos.write(pBytes, pOffset, pLen);
                }
            };
            return;
        }
        throw new SAXParseException("Unexpected start tag in atomic element: " + new QName(pURI, pLocalName), getDocumentLocator());
    }
}
