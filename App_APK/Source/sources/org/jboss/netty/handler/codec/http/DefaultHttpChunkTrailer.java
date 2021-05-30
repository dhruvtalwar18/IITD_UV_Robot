package org.jboss.netty.handler.codec.http;

import java.util.List;
import java.util.Map;
import java.util.Set;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

public class DefaultHttpChunkTrailer implements HttpChunkTrailer {
    private final HttpHeaders headers = new HttpHeaders() {
        /* access modifiers changed from: package-private */
        public void validateHeaderName(String name) {
            super.validateHeaderName(name);
            if (name.equalsIgnoreCase("Content-Length") || name.equalsIgnoreCase("Transfer-Encoding") || name.equalsIgnoreCase("Trailer")) {
                throw new IllegalArgumentException("prohibited trailing header: " + name);
            }
        }
    };

    public boolean isLast() {
        return true;
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

    public ChannelBuffer getContent() {
        return ChannelBuffers.EMPTY_BUFFER;
    }

    public void setContent(ChannelBuffer content) {
        throw new IllegalStateException("read-only");
    }
}
