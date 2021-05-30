package org.jboss.netty.handler.codec.http;

import java.io.UnsupportedEncodingException;
import java.util.Map;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.oneone.OneToOneEncoder;
import org.jboss.netty.util.CharsetUtil;

public abstract class HttpMessageEncoder extends OneToOneEncoder {
    private static final byte[] CRLF = {13, 10};
    private static final ChannelBuffer LAST_CHUNK = ChannelBuffers.copiedBuffer((CharSequence) "0\r\n\r\n", CharsetUtil.US_ASCII);
    private volatile boolean chunked;

    /* access modifiers changed from: protected */
    public abstract void encodeInitialLine(ChannelBuffer channelBuffer, HttpMessage httpMessage) throws Exception;

    protected HttpMessageEncoder() {
    }

    /* access modifiers changed from: protected */
    public Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        boolean chunked2;
        if (msg instanceof HttpMessage) {
            HttpMessage m = (HttpMessage) msg;
            if (m.isChunked()) {
                if (!HttpCodecUtil.isTransferEncodingChunked(m)) {
                    m.addHeader("Transfer-Encoding", HttpHeaders.Values.CHUNKED);
                }
                this.chunked = true;
                chunked2 = true;
            } else {
                chunked2 = HttpCodecUtil.isTransferEncodingChunked(m);
                this.chunked = chunked2;
            }
            ChannelBuffer header = ChannelBuffers.dynamicBuffer(channel.getConfig().getBufferFactory());
            encodeInitialLine(header, m);
            encodeHeaders(header, m);
            header.writeByte(13);
            header.writeByte(10);
            ChannelBuffer content = m.getContent();
            if (!content.readable()) {
                return header;
            }
            if (!chunked2) {
                return ChannelBuffers.wrappedBuffer(header, content);
            }
            throw new IllegalArgumentException("HttpMessage.content must be empty if Transfer-Encoding is chunked.");
        } else if (!(msg instanceof HttpChunk)) {
            return msg;
        } else {
            HttpChunk chunk = (HttpChunk) msg;
            if (this.chunked) {
                if (chunk.isLast()) {
                    this.chunked = false;
                    if (!(chunk instanceof HttpChunkTrailer)) {
                        return LAST_CHUNK.duplicate();
                    }
                    ChannelBuffer trailer = ChannelBuffers.dynamicBuffer(channel.getConfig().getBufferFactory());
                    trailer.writeByte(48);
                    trailer.writeByte(13);
                    trailer.writeByte(10);
                    encodeTrailingHeaders(trailer, (HttpChunkTrailer) chunk);
                    trailer.writeByte(13);
                    trailer.writeByte(10);
                    return trailer;
                }
                ChannelBuffer content2 = chunk.getContent();
                int contentLength = content2.readableBytes();
                return ChannelBuffers.wrappedBuffer(ChannelBuffers.copiedBuffer((CharSequence) Integer.toHexString(contentLength), CharsetUtil.US_ASCII), ChannelBuffers.wrappedBuffer(CRLF), content2.slice(content2.readerIndex(), contentLength), ChannelBuffers.wrappedBuffer(CRLF));
            } else if (chunk.isLast()) {
                return null;
            } else {
                return chunk.getContent();
            }
        }
    }

    private static void encodeHeaders(ChannelBuffer buf, HttpMessage message) {
        try {
            for (Map.Entry<String, String> h : message.getHeaders()) {
                encodeHeader(buf, h.getKey(), h.getValue());
            }
        } catch (UnsupportedEncodingException e) {
            throw ((Error) new Error().initCause(e));
        }
    }

    private static void encodeTrailingHeaders(ChannelBuffer buf, HttpChunkTrailer trailer) {
        try {
            for (Map.Entry<String, String> h : trailer.getHeaders()) {
                encodeHeader(buf, h.getKey(), h.getValue());
            }
        } catch (UnsupportedEncodingException e) {
            throw ((Error) new Error().initCause(e));
        }
    }

    private static void encodeHeader(ChannelBuffer buf, String header, String value) throws UnsupportedEncodingException {
        buf.writeBytes(header.getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(58);
        buf.writeByte(32);
        buf.writeBytes(value.getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(13);
        buf.writeByte(10);
    }
}
