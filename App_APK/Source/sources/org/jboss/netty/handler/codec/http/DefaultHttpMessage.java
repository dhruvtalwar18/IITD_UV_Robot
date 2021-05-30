package org.jboss.netty.handler.codec.http;

import java.util.List;
import java.util.Map;
import java.util.Set;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.util.internal.StringUtil;

public class DefaultHttpMessage implements HttpMessage {
    private boolean chunked;
    private ChannelBuffer content = ChannelBuffers.EMPTY_BUFFER;
    private final HttpHeaders headers = new HttpHeaders();
    private HttpVersion version;

    protected DefaultHttpMessage(HttpVersion version2) {
        setProtocolVersion(version2);
    }

    public void addHeader(String name, Object value) {
        this.headers.addHeader(name, value);
    }

    public void setHeader(String name, Object value) {
        this.headers.setHeader(name, value);
    }

    public void setHeader(String name, Iterable<?> values) {
        this.headers.setHeader(name, values);
    }

    public void removeHeader(String name) {
        this.headers.removeHeader(name);
    }

    @Deprecated
    public long getContentLength() {
        return HttpHeaders.getContentLength(this);
    }

    @Deprecated
    public long getContentLength(long defaultValue) {
        return HttpHeaders.getContentLength(this, defaultValue);
    }

    public boolean isChunked() {
        if (this.chunked) {
            return true;
        }
        return HttpCodecUtil.isTransferEncodingChunked(this);
    }

    public void setChunked(boolean chunked2) {
        this.chunked = chunked2;
        if (chunked2) {
            setContent(ChannelBuffers.EMPTY_BUFFER);
        }
    }

    @Deprecated
    public boolean isKeepAlive() {
        return HttpHeaders.isKeepAlive(this);
    }

    public void clearHeaders() {
        this.headers.clearHeaders();
    }

    public void setContent(ChannelBuffer content2) {
        if (content2 == null) {
            content2 = ChannelBuffers.EMPTY_BUFFER;
        }
        if (!content2.readable() || !isChunked()) {
            this.content = content2;
            return;
        }
        throw new IllegalArgumentException("non-empty content disallowed if this.chunked == true");
    }

    public String getHeader(String name) {
        return this.headers.getHeader(name);
    }

    public List<String> getHeaders(String name) {
        return this.headers.getHeaders(name);
    }

    public List<Map.Entry<String, String>> getHeaders() {
        return this.headers.getHeaders();
    }

    public boolean containsHeader(String name) {
        return this.headers.containsHeader(name);
    }

    public Set<String> getHeaderNames() {
        return this.headers.getHeaderNames();
    }

    public HttpVersion getProtocolVersion() {
        return this.version;
    }

    public void setProtocolVersion(HttpVersion version2) {
        if (version2 != null) {
            this.version = version2;
            return;
        }
        throw new NullPointerException("version");
    }

    public ChannelBuffer getContent() {
        return this.content;
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append(getClass().getSimpleName());
        buf.append("(version: ");
        buf.append(getProtocolVersion().getText());
        buf.append(", keepAlive: ");
        buf.append(isKeepAlive());
        buf.append(", chunked: ");
        buf.append(isChunked());
        buf.append(')');
        buf.append(StringUtil.NEWLINE);
        appendHeaders(buf);
        buf.setLength(buf.length() - StringUtil.NEWLINE.length());
        return buf.toString();
    }

    /* access modifiers changed from: package-private */
    public void appendHeaders(StringBuilder buf) {
        for (Map.Entry<String, String> e : getHeaders()) {
            buf.append(e.getKey());
            buf.append(": ");
            buf.append(e.getValue());
            buf.append(StringUtil.NEWLINE);
        }
    }
}
