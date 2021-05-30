package org.apache.commons.httpclient.methods.multipart;

import java.io.IOException;
import java.io.OutputStream;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class StringPart extends PartBase {
    public static final String DEFAULT_CHARSET = "US-ASCII";
    public static final String DEFAULT_CONTENT_TYPE = "text/plain";
    public static final String DEFAULT_TRANSFER_ENCODING = "8bit";
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$multipart$StringPart;
    private byte[] content;
    private String value;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$multipart$StringPart == null) {
            cls = class$("org.apache.commons.httpclient.methods.multipart.StringPart");
            class$org$apache$commons$httpclient$methods$multipart$StringPart = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$multipart$StringPart;
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

    /* JADX INFO: super call moved to the top of the method (can break code semantics) */
    public StringPart(String name, String value2, String charset) {
        super(name, "text/plain", charset == null ? "US-ASCII" : charset, DEFAULT_TRANSFER_ENCODING);
        if (value2 == null) {
            throw new IllegalArgumentException("Value may not be null");
        } else if (value2.indexOf(0) == -1) {
            this.value = value2;
        } else {
            throw new IllegalArgumentException("NULs may not be present in string parts");
        }
    }

    public StringPart(String name, String value2) {
        this(name, value2, (String) null);
    }

    private byte[] getContent() {
        if (this.content == null) {
            this.content = EncodingUtil.getBytes(this.value, getCharSet());
        }
        return this.content;
    }

    /* access modifiers changed from: protected */
    public void sendData(OutputStream out) throws IOException {
        LOG.trace("enter sendData(OutputStream)");
        out.write(getContent());
    }

    /* access modifiers changed from: protected */
    public long lengthOfData() throws IOException {
        LOG.trace("enter lengthOfData()");
        return (long) getContent().length;
    }

    public void setCharSet(String charSet) {
        super.setCharSet(charSet);
        this.content = null;
    }
}
