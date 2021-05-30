package org.apache.ws.commons.serialize;

import java.io.IOException;
import java.io.Writer;
import java.util.HashMap;
import java.util.Map;
import org.xml.sax.Attributes;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;

public class XMLWriterImpl implements XMLWriter {
    private static final int STATE_IN_ELEMENT = 2;
    private static final int STATE_IN_START_ELEMENT = 1;
    private static final int STATE_OUTSIDE = 0;
    int curIndent = 0;
    private boolean declarating;
    private Map delayedPrefixes;
    private String encoding;
    private boolean flushing;
    private String indentString;
    private boolean indenting;
    private Locator l;
    private String lineFeed;
    private int state;
    private Writer w;

    public void setEncoding(String pEncoding) {
        this.encoding = pEncoding;
    }

    public String getEncoding() {
        return this.encoding;
    }

    public void setDeclarating(boolean pDeclarating) {
        this.declarating = pDeclarating;
    }

    public boolean isDeclarating() {
        return this.declarating;
    }

    public void setIndenting(boolean pIndenting) {
        this.indenting = pIndenting;
    }

    public boolean isIndenting() {
        return this.indenting;
    }

    public void setIndentString(String pIndentString) {
        this.indentString = pIndentString;
    }

    public String getIndentString() {
        return this.indentString;
    }

    public void setLineFeed(String pLineFeed) {
        this.lineFeed = pLineFeed;
    }

    public String getLineFeed() {
        return this.lineFeed;
    }

    public void setFlushing(boolean pFlushing) {
        this.flushing = pFlushing;
    }

    public boolean isFlushing() {
        return this.flushing;
    }

    public void setWriter(Writer pWriter) {
        this.w = pWriter;
    }

    public Writer getWriter() {
        return this.w;
    }

    public void setDocumentLocator(Locator pLocator) {
        this.l = pLocator;
    }

    public Locator getDocumentLocator() {
        return this.l;
    }

    public void startPrefixMapping(String prefix, String namespaceURI) throws SAXException {
        String prefix2;
        if (this.delayedPrefixes == null) {
            this.delayedPrefixes = new HashMap();
        }
        if (!"".equals(prefix)) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("xmlns:");
            stringBuffer.append(prefix);
            prefix2 = stringBuffer.toString();
        } else if (!namespaceURI.equals(prefix)) {
            prefix2 = "xmlns";
        } else {
            return;
        }
        this.delayedPrefixes.put(prefix2, namespaceURI);
    }

    public void endPrefixMapping(String prefix) throws SAXException {
        String prefix2;
        if (this.delayedPrefixes != null) {
            if ("".equals(prefix)) {
                prefix2 = "xmlns";
            } else {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("xmlns:");
                stringBuffer.append(prefix);
                prefix2 = stringBuffer.toString();
            }
            this.delayedPrefixes.remove(prefix2);
        }
    }

    public void startDocument() throws SAXException {
        String lf;
        if (this.delayedPrefixes != null) {
            this.delayedPrefixes.clear();
        }
        this.state = 0;
        this.curIndent = 0;
        if (isDeclarating() && this.w != null) {
            try {
                this.w.write("<?xml version=\"1.0\"");
                String enc = getEncoding();
                if (enc != null) {
                    this.w.write(" encoding=\"");
                    this.w.write(enc);
                    this.w.write("\"");
                }
                this.w.write("?>");
                if (isIndenting() && (lf = getLineFeed()) != null) {
                    this.w.write(lf);
                }
            } catch (IOException e) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Failed to write XML declaration: ");
                stringBuffer.append(e.getMessage());
                throw new SAXException(stringBuffer.toString(), e);
            }
        }
    }

    public void endDocument() throws SAXException {
        if (isFlushing() && this.w != null) {
            try {
                this.w.flush();
            } catch (IOException e) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Failed to flush target writer: ");
                stringBuffer.append(e.getMessage());
                throw new SAXException(stringBuffer.toString(), e);
            }
        }
    }

    public void ignorableWhitespace(char[] ch, int start, int length) throws SAXException {
        characters(ch, start, length);
    }

    private void stopTerminator() throws IOException {
        if (this.state == 1) {
            if (this.w != null) {
                this.w.write(62);
            }
            this.state = 2;
        }
    }

    public void characters(char[] ch, int start, int length) throws SAXException {
        try {
            stopTerminator();
            if (this.w != null) {
                int end = start + length;
                for (int i = start; i < end; i++) {
                    char c = ch[i];
                    if (c != 13) {
                        if (c == '&') {
                            this.w.write("&amp;");
                        } else if (c == '<') {
                            this.w.write("&lt;");
                        } else if (c != '>') {
                            switch (c) {
                                case 9:
                                case 10:
                                    break;
                                default:
                                    if (!canEncode(c)) {
                                        this.w.write("&#");
                                        this.w.write(Integer.toString(c));
                                        this.w.write(";");
                                        break;
                                    } else {
                                        this.w.write(c);
                                        continue;
                                    }
                            }
                        } else {
                            this.w.write("&gt;");
                        }
                    }
                    this.w.write(c);
                }
            }
        } catch (IOException e) {
            throw new SAXException(e);
        }
    }

    public boolean canEncode(char c) {
        return c == 10 || (c >= ' ' && c < 127);
    }

    public void endElement(String namespaceURI, String localName, String qName) throws SAXException {
        if (isIndenting()) {
            this.curIndent--;
        }
        if (this.w != null) {
            try {
                if (this.state == 1) {
                    this.w.write("/>");
                    this.state = 0;
                } else {
                    if (this.state == 0) {
                        indentMe();
                    }
                    this.w.write("</");
                    this.w.write(qName);
                    this.w.write(62);
                }
                this.state = 0;
            } catch (IOException e) {
                throw new SAXException(e);
            }
        }
    }

    private void indentMe() throws IOException {
        if (this.w != null && isIndenting()) {
            String s = getLineFeed();
            if (s != null) {
                this.w.write(s);
            }
            String s2 = getIndentString();
            if (s2 != null) {
                for (int i = 0; i < this.curIndent; i++) {
                    this.w.write(s2);
                }
            }
        }
    }

    private void writeCData(String v) throws IOException {
        int len = v.length();
        for (int j = 0; j < len; j++) {
            char c = v.charAt(j);
            if (c == '\"') {
                this.w.write("&quot;");
            } else if (c == '<') {
                this.w.write("&lt;");
            } else if (c != '>') {
                switch (c) {
                    case '&':
                        this.w.write("&amp;");
                        break;
                    case '\'':
                        this.w.write("&apos;");
                        break;
                    default:
                        if (!canEncode(c)) {
                            this.w.write("&#");
                            this.w.write(Integer.toString(c));
                            this.w.write(59);
                            break;
                        } else {
                            this.w.write(c);
                            break;
                        }
                }
            } else {
                this.w.write("&gt;");
            }
        }
    }

    public void startElement(String namespaceURI, String localName, String qName, Attributes attr) throws SAXException {
        try {
            stopTerminator();
            if (isIndenting()) {
                if (this.curIndent > 0) {
                    indentMe();
                }
                this.curIndent++;
            }
            if (this.w != null) {
                this.w.write(60);
                this.w.write(qName);
                if (attr != null) {
                    int i = attr.getLength();
                    while (i > 0) {
                        this.w.write(32);
                        i--;
                        String name = attr.getQName(i);
                        this.w.write(name);
                        if (this.delayedPrefixes != null) {
                            this.delayedPrefixes.remove(name);
                        }
                        this.w.write("=\"");
                        writeCData(attr.getValue(i));
                        this.w.write(34);
                    }
                }
                if (this.delayedPrefixes != null && this.delayedPrefixes.size() > 0) {
                    for (Map.Entry entry : this.delayedPrefixes.entrySet()) {
                        this.w.write(32);
                        this.w.write((String) entry.getKey());
                        this.w.write("=\"");
                        this.w.write((String) entry.getValue());
                        this.w.write(34);
                    }
                    this.delayedPrefixes.clear();
                }
            }
            this.state = 1;
        } catch (IOException e) {
            throw new SAXException(e);
        }
    }

    public void skippedEntity(String ent) throws SAXException {
        throw new SAXException("Don't know how to skip entities");
    }

    public void processingInstruction(String target, String data) throws SAXException {
        try {
            stopTerminator();
            if (this.w != null) {
                this.w.write("<?");
                this.w.write(target);
                this.w.write(32);
                this.w.write(data);
                this.w.write("?>");
            }
        } catch (IOException e) {
            throw new SAXException(e);
        }
    }
}
