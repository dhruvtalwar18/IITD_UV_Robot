package org.apache.commons.httpclient.methods;

import java.io.IOException;
import java.io.OutputStream;

public class ByteArrayRequestEntity implements RequestEntity {
    private byte[] content;
    private String contentType;

    public ByteArrayRequestEntity(byte[] content2) {
        this(content2, (String) null);
    }

    public ByteArrayRequestEntity(byte[] content2, String contentType2) {
        if (content2 != null) {
            this.content = content2;
            this.contentType = contentType2;
            return;
        }
        throw new IllegalArgumentException("The content cannot be null");
    }

    public boolean isRepeatable() {
        return true;
    }

    public String getContentType() {
        return this.contentType;
    }

    public void writeRequest(OutputStream out) throws IOException {
        out.write(this.content);
    }

    public long getContentLength() {
        return (long) this.content.length;
    }

    public byte[] getContent() {
        return this.content;
    }
}
