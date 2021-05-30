package org.apache.commons.httpclient.methods;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class InputStreamRequestEntity implements RequestEntity {
    public static final int CONTENT_LENGTH_AUTO = -2;
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$InputStreamRequestEntity;
    private byte[] buffer;
    private InputStream content;
    private long contentLength;
    private String contentType;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$InputStreamRequestEntity == null) {
            cls = class$("org.apache.commons.httpclient.methods.InputStreamRequestEntity");
            class$org$apache$commons$httpclient$methods$InputStreamRequestEntity = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$InputStreamRequestEntity;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public InputStreamRequestEntity(InputStream content2) {
        this(content2, (String) null);
    }

    public InputStreamRequestEntity(InputStream content2, String contentType2) {
        this(content2, -2, contentType2);
    }

    public InputStreamRequestEntity(InputStream content2, long contentLength2) {
        this(content2, contentLength2, (String) null);
    }

    public InputStreamRequestEntity(InputStream content2, long contentLength2, String contentType2) {
        this.buffer = null;
        if (content2 != null) {
            this.content = content2;
            this.contentLength = contentLength2;
            this.contentType = contentType2;
            return;
        }
        throw new IllegalArgumentException("The content cannot be null");
    }

    public String getContentType() {
        return this.contentType;
    }

    private void bufferContent() {
        if (this.buffer == null && this.content != null) {
            try {
                ByteArrayOutputStream tmp = new ByteArrayOutputStream();
                byte[] data = new byte[4096];
                while (true) {
                    int read = this.content.read(data);
                    int l = read;
                    if (read >= 0) {
                        tmp.write(data, 0, l);
                    } else {
                        this.buffer = tmp.toByteArray();
                        this.content = null;
                        this.contentLength = (long) this.buffer.length;
                        return;
                    }
                }
            } catch (IOException e) {
                LOG.error(e.getMessage(), e);
                this.buffer = null;
                this.content = null;
                this.contentLength = 0;
            }
        }
    }

    public boolean isRepeatable() {
        return this.buffer != null;
    }

    public void writeRequest(OutputStream out) throws IOException {
        if (this.content != null) {
            byte[] tmp = new byte[4096];
            int total = 0;
            while (true) {
                int read = this.content.read(tmp);
                int i = read;
                if (read >= 0) {
                    out.write(tmp, 0, i);
                    total += i;
                } else {
                    return;
                }
            }
        } else if (this.buffer != null) {
            out.write(this.buffer);
        } else {
            throw new IllegalStateException("Content must be set before entity is written");
        }
    }

    public long getContentLength() {
        if (this.contentLength == -2 && this.buffer == null) {
            bufferContent();
        }
        return this.contentLength;
    }

    public InputStream getContent() {
        return this.content;
    }
}
