package org.jboss.netty.handler.codec.http;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

public interface HttpChunk {
    public static final HttpChunkTrailer LAST_CHUNK = new HttpChunkTrailer() {
        public ChannelBuffer getContent() {
            return ChannelBuffers.EMPTY_BUFFER;
        }

        public void setContent(ChannelBuffer content) {
            throw new IllegalStateException("read-only");
        }

        public boolean isLast() {
            return true;
        }

        public void addHeader(String name, Object value) {
            throw new IllegalStateException("read-only");
        }

        public void clearHeaders() {
        }

        public boolean containsHeader(String name) {
            return false;
        }

        public String getHeader(String name) {
            return null;
        }

        public Set<String> getHeaderNames() {
            return Collections.emptySet();
        }

        public List<String> getHeaders(String name) {
            return Collections.emptyList();
        }

        public List<Map.Entry<String, String>> getHeaders() {
            return Collections.emptyList();
        }

        public void removeHeader(String name) {
        }

        public void setHeader(String name, Object value) {
            throw new IllegalStateException("read-only");
        }

        public void setHeader(String name, Iterable<?> iterable) {
            throw new IllegalStateException("read-only");
        }
    };

    ChannelBuffer getContent();

    boolean isLast();

    void setContent(ChannelBuffer channelBuffer);
}
