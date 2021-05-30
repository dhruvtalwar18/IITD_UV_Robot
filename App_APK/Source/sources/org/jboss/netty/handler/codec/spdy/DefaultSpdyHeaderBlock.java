package org.jboss.netty.handler.codec.spdy;

import java.util.List;
import java.util.Map;
import java.util.Set;
import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdyHeaderBlock implements SpdyHeaderBlock {
    private final SpdyHeaders headers = new SpdyHeaders();
    private boolean invalid;

    protected DefaultSpdyHeaderBlock() {
    }

    public boolean isInvalid() {
        return this.invalid;
    }

    public void setInvalid() {
        this.invalid = true;
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

    public void clearHeaders() {
        this.headers.clearHeaders();
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

    /* access modifiers changed from: protected */
    public void appendHeaders(StringBuilder buf) {
        for (Map.Entry<String, String> e : getHeaders()) {
            buf.append("    ");
            buf.append(e.getKey());
            buf.append(": ");
            buf.append(e.getValue());
            buf.append(StringUtil.NEWLINE);
        }
    }
}
