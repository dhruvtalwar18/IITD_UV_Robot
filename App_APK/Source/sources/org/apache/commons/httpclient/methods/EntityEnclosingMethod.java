package org.apache.commons.httpclient.methods;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import org.apache.commons.httpclient.ChunkedOutputStream;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.HttpVersion;
import org.apache.commons.httpclient.ProtocolException;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.handler.codec.http.HttpHeaders;

public abstract class EntityEnclosingMethod extends ExpectContinueMethod {
    public static final long CONTENT_LENGTH_AUTO = -2;
    public static final long CONTENT_LENGTH_CHUNKED = -1;
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$EntityEnclosingMethod;
    private boolean chunked = false;
    private int repeatCount = 0;
    private long requestContentLength = -2;
    private RequestEntity requestEntity;
    private InputStream requestStream = null;
    private String requestString = null;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$EntityEnclosingMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.EntityEnclosingMethod");
            class$org$apache$commons$httpclient$methods$EntityEnclosingMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$EntityEnclosingMethod;
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

    public EntityEnclosingMethod() {
        setFollowRedirects(false);
    }

    public EntityEnclosingMethod(String uri) {
        super(uri);
        setFollowRedirects(false);
    }

    /* access modifiers changed from: protected */
    public boolean hasRequestContent() {
        LOG.trace("enter EntityEnclosingMethod.hasRequestContent()");
        return (this.requestEntity == null && this.requestStream == null && this.requestString == null) ? false : true;
    }

    /* access modifiers changed from: protected */
    public void clearRequestBody() {
        LOG.trace("enter EntityEnclosingMethod.clearRequestBody()");
        this.requestStream = null;
        this.requestString = null;
        this.requestEntity = null;
    }

    /* access modifiers changed from: protected */
    public byte[] generateRequestBody() {
        LOG.trace("enter EntityEnclosingMethod.renerateRequestBody()");
        return null;
    }

    /* access modifiers changed from: protected */
    public RequestEntity generateRequestEntity() {
        byte[] requestBody = generateRequestBody();
        if (requestBody != null) {
            this.requestEntity = new ByteArrayRequestEntity(requestBody);
        } else if (this.requestStream != null) {
            this.requestEntity = new InputStreamRequestEntity(this.requestStream, this.requestContentLength);
            this.requestStream = null;
        } else if (this.requestString != null) {
            String charset = getRequestCharSet();
            try {
                this.requestEntity = new StringRequestEntity(this.requestString, (String) null, charset);
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append(charset);
                    stringBuffer.append(" not supported");
                    log.warn(stringBuffer.toString());
                }
                try {
                    this.requestEntity = new StringRequestEntity(this.requestString, (String) null, (String) null);
                } catch (UnsupportedEncodingException e2) {
                }
            }
        }
        return this.requestEntity;
    }

    public boolean getFollowRedirects() {
        return false;
    }

    public void setFollowRedirects(boolean followRedirects) {
        if (!followRedirects) {
            super.setFollowRedirects(false);
            return;
        }
        throw new IllegalArgumentException("Entity enclosing requests cannot be redirected without user intervention");
    }

    public void setRequestContentLength(int length) {
        LOG.trace("enter EntityEnclosingMethod.setRequestContentLength(int)");
        this.requestContentLength = (long) length;
    }

    public String getRequestCharSet() {
        if (getRequestHeader("Content-Type") != null) {
            return super.getRequestCharSet();
        }
        if (this.requestEntity != null) {
            return getContentCharSet(new Header("Content-Type", this.requestEntity.getContentType()));
        }
        return super.getRequestCharSet();
    }

    public void setRequestContentLength(long length) {
        LOG.trace("enter EntityEnclosingMethod.setRequestContentLength(int)");
        this.requestContentLength = length;
    }

    public void setContentChunked(boolean chunked2) {
        this.chunked = chunked2;
    }

    /* access modifiers changed from: protected */
    public long getRequestContentLength() {
        LOG.trace("enter EntityEnclosingMethod.getRequestContentLength()");
        if (!hasRequestContent()) {
            return 0;
        }
        if (this.chunked) {
            return -1;
        }
        if (this.requestEntity == null) {
            this.requestEntity = generateRequestEntity();
        }
        if (this.requestEntity == null) {
            return 0;
        }
        return this.requestEntity.getContentLength();
    }

    /* access modifiers changed from: protected */
    public void addRequestHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        RequestEntity requestEntity2;
        LOG.trace("enter EntityEnclosingMethod.addRequestHeaders(HttpState, HttpConnection)");
        super.addRequestHeaders(state, conn);
        addContentLengthRequestHeader(state, conn);
        if (getRequestHeader("Content-Type") == null && (requestEntity2 = getRequestEntity()) != null && requestEntity2.getContentType() != null) {
            setRequestHeader("Content-Type", requestEntity2.getContentType());
        }
    }

    /* access modifiers changed from: protected */
    public void addContentLengthRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter EntityEnclosingMethod.addContentLengthRequestHeader(HttpState, HttpConnection)");
        if (getRequestHeader("content-length") == null && getRequestHeader("Transfer-Encoding") == null) {
            long len = getRequestContentLength();
            if (len >= 0) {
                addRequestHeader("Content-Length", String.valueOf(len));
            } else if (getEffectiveVersion().greaterEquals(HttpVersion.HTTP_1_1)) {
                addRequestHeader("Transfer-Encoding", HttpHeaders.Values.CHUNKED);
            } else {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(getEffectiveVersion());
                stringBuffer.append(" does not support chunk encoding");
                throw new ProtocolException(stringBuffer.toString());
            }
        }
    }

    public void setRequestBody(InputStream body) {
        LOG.trace("enter EntityEnclosingMethod.setRequestBody(InputStream)");
        clearRequestBody();
        this.requestStream = body;
    }

    public void setRequestBody(String body) {
        LOG.trace("enter EntityEnclosingMethod.setRequestBody(String)");
        clearRequestBody();
        this.requestString = body;
    }

    /* access modifiers changed from: protected */
    public boolean writeRequestBody(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter EntityEnclosingMethod.writeRequestBody(HttpState, HttpConnection)");
        if (!hasRequestContent()) {
            LOG.debug("Request body has not been specified");
            return true;
        }
        if (this.requestEntity == null) {
            this.requestEntity = generateRequestEntity();
        }
        if (this.requestEntity == null) {
            LOG.debug("Request body is empty");
            return true;
        }
        long contentLength = getRequestContentLength();
        if (this.repeatCount <= 0 || this.requestEntity.isRepeatable()) {
            this.repeatCount++;
            OutputStream outstream = conn.getRequestOutputStream();
            if (contentLength < 0) {
                outstream = new ChunkedOutputStream(outstream);
            }
            this.requestEntity.writeRequest(outstream);
            if (outstream instanceof ChunkedOutputStream) {
                ((ChunkedOutputStream) outstream).finish();
            }
            outstream.flush();
            LOG.debug("Request body sent");
            return true;
        }
        throw new ProtocolException("Unbuffered entity enclosing request can not be repeated.");
    }

    public void recycle() {
        LOG.trace("enter EntityEnclosingMethod.recycle()");
        clearRequestBody();
        this.requestContentLength = -2;
        this.repeatCount = 0;
        this.chunked = false;
        super.recycle();
    }

    public RequestEntity getRequestEntity() {
        return generateRequestEntity();
    }

    public void setRequestEntity(RequestEntity requestEntity2) {
        clearRequestBody();
        this.requestEntity = requestEntity2;
    }
}
