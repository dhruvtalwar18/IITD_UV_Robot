package org.apache.commons.httpclient.methods;

import java.io.IOException;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import org.apache.commons.httpclient.HeaderElement;
import org.apache.commons.httpclient.NameValuePair;

public class StringRequestEntity implements RequestEntity {
    private String charset;
    private byte[] content;
    private String contentType;

    public StringRequestEntity(String content2) {
        if (content2 != null) {
            this.contentType = null;
            this.charset = null;
            this.content = content2.getBytes();
            return;
        }
        throw new IllegalArgumentException("The content cannot be null");
    }

    public StringRequestEntity(String content2, String contentType2, String charset2) throws UnsupportedEncodingException {
        if (content2 != null) {
            this.contentType = contentType2;
            this.charset = charset2;
            if (contentType2 != null) {
                HeaderElement[] values = HeaderElement.parseElements(contentType2);
                NameValuePair charsetPair = null;
                for (HeaderElement parameterByName : values) {
                    NameValuePair parameterByName2 = parameterByName.getParameterByName("charset");
                    charsetPair = parameterByName2;
                    if (parameterByName2 != null) {
                        break;
                    }
                }
                if (charset2 == null && charsetPair != null) {
                    this.charset = charsetPair.getValue();
                } else if (charset2 != null && charsetPair == null) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append(contentType2);
                    stringBuffer.append("; charset=");
                    stringBuffer.append(charset2);
                    this.contentType = stringBuffer.toString();
                }
            }
            if (this.charset != null) {
                this.content = content2.getBytes(this.charset);
            } else {
                this.content = content2.getBytes();
            }
        } else {
            throw new IllegalArgumentException("The content cannot be null");
        }
    }

    public String getContentType() {
        return this.contentType;
    }

    public boolean isRepeatable() {
        return true;
    }

    public void writeRequest(OutputStream out) throws IOException {
        if (out != null) {
            out.write(this.content);
            out.flush();
            return;
        }
        throw new IllegalArgumentException("Output stream may not be null");
    }

    public long getContentLength() {
        return (long) this.content.length;
    }

    public String getContent() {
        if (this.charset == null) {
            return new String(this.content);
        }
        try {
            return new String(this.content, this.charset);
        } catch (UnsupportedEncodingException e) {
            return new String(this.content);
        }
    }

    public String getCharset() {
        return this.charset;
    }
}
