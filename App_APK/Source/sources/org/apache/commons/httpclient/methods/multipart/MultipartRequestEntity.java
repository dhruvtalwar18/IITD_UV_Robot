package org.apache.commons.httpclient.methods.multipart;

import java.io.IOException;
import java.io.OutputStream;
import java.util.Random;
import org.apache.commons.httpclient.methods.RequestEntity;
import org.apache.commons.httpclient.params.HttpMethodParams;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class MultipartRequestEntity implements RequestEntity {
    private static byte[] MULTIPART_CHARS = EncodingUtil.getAsciiBytes("-_1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ");
    private static final String MULTIPART_FORM_CONTENT_TYPE = "multipart/form-data";
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$multipart$MultipartRequestEntity;
    private static final Log log;
    private byte[] multipartBoundary;
    private HttpMethodParams params;
    protected Part[] parts;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$multipart$MultipartRequestEntity == null) {
            cls = class$("org.apache.commons.httpclient.methods.multipart.MultipartRequestEntity");
            class$org$apache$commons$httpclient$methods$multipart$MultipartRequestEntity = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$multipart$MultipartRequestEntity;
        }
        log = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    private static byte[] generateMultipartBoundary() {
        Random rand = new Random();
        byte[] bytes = new byte[(rand.nextInt(11) + 30)];
        for (int i = 0; i < bytes.length; i++) {
            bytes[i] = MULTIPART_CHARS[rand.nextInt(MULTIPART_CHARS.length)];
        }
        return bytes;
    }

    public MultipartRequestEntity(Part[] parts2, HttpMethodParams params2) {
        if (parts2 == null) {
            throw new IllegalArgumentException("parts cannot be null");
        } else if (params2 != null) {
            this.parts = parts2;
            this.params = params2;
        } else {
            throw new IllegalArgumentException("params cannot be null");
        }
    }

    /* access modifiers changed from: protected */
    public byte[] getMultipartBoundary() {
        if (this.multipartBoundary == null) {
            String temp = (String) this.params.getParameter(HttpMethodParams.MULTIPART_BOUNDARY);
            if (temp != null) {
                this.multipartBoundary = EncodingUtil.getAsciiBytes(temp);
            } else {
                this.multipartBoundary = generateMultipartBoundary();
            }
        }
        return this.multipartBoundary;
    }

    public boolean isRepeatable() {
        for (Part isRepeatable : this.parts) {
            if (!isRepeatable.isRepeatable()) {
                return false;
            }
        }
        return true;
    }

    public void writeRequest(OutputStream out) throws IOException {
        Part.sendParts(out, this.parts, getMultipartBoundary());
    }

    public long getContentLength() {
        try {
            return Part.getLengthOfParts(this.parts, getMultipartBoundary());
        } catch (Exception e) {
            log.error("An exception occurred while getting the length of the parts", e);
            return 0;
        }
    }

    public String getContentType() {
        StringBuffer buffer = new StringBuffer("multipart/form-data");
        buffer.append("; boundary=");
        buffer.append(EncodingUtil.getAsciiString(getMultipartBoundary()));
        return buffer.toString();
    }
}
