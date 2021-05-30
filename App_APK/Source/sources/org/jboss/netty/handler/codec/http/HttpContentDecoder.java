package org.jboss.netty.handler.codec.http;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;
import org.jboss.netty.handler.codec.embedder.DecoderEmbedder;

public abstract class HttpContentDecoder extends SimpleChannelUpstreamHandler {
    private DecoderEmbedder<ChannelBuffer> decoder;

    /* access modifiers changed from: protected */
    public abstract DecoderEmbedder<ChannelBuffer> newContentDecoder(String str) throws Exception;

    protected HttpContentDecoder() {
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        String contentEncoding;
        Object msg = e.getMessage();
        if ((msg instanceof HttpResponse) && ((HttpResponse) msg).getStatus().getCode() == 100) {
            ctx.sendUpstream(e);
        } else if (msg instanceof HttpMessage) {
            HttpMessage m = (HttpMessage) msg;
            this.decoder = null;
            String contentEncoding2 = m.getHeader("Content-Encoding");
            if (contentEncoding2 != null) {
                contentEncoding = contentEncoding2.trim();
            } else {
                contentEncoding = "identity";
            }
            if (m.isChunked() || m.getContent().readable()) {
                DecoderEmbedder<ChannelBuffer> newContentDecoder = newContentDecoder(contentEncoding);
                this.decoder = newContentDecoder;
                if (newContentDecoder != null) {
                    m.setHeader("Content-Encoding", (Object) getTargetContentEncoding(contentEncoding));
                    if (!m.isChunked()) {
                        ChannelBuffer content = ChannelBuffers.wrappedBuffer(decode(m.getContent()), finishDecode());
                        m.setContent(content);
                        if (m.containsHeader("Content-Length")) {
                            m.setHeader("Content-Length", (Object) Integer.toString(content.readableBytes()));
                        }
                    }
                }
            }
            ctx.sendUpstream(e);
        } else if (msg instanceof HttpChunk) {
            HttpChunk c = (HttpChunk) msg;
            ChannelBuffer content2 = c.getContent();
            if (this.decoder == null) {
                ctx.sendUpstream(e);
            } else if (!c.isLast()) {
                ChannelBuffer content3 = decode(content2);
                if (content3.readable()) {
                    c.setContent(content3);
                    ctx.sendUpstream(e);
                }
            } else {
                ChannelBuffer lastProduct = finishDecode();
                if (lastProduct.readable()) {
                    Channels.fireMessageReceived(ctx, (Object) new DefaultHttpChunk(lastProduct), e.getRemoteAddress());
                }
                ctx.sendUpstream(e);
            }
        } else {
            ctx.sendUpstream(e);
        }
    }

    /* access modifiers changed from: protected */
    public String getTargetContentEncoding(String contentEncoding) throws Exception {
        return "identity";
    }

    private ChannelBuffer decode(ChannelBuffer buf) {
        this.decoder.offer(buf);
        return ChannelBuffers.wrappedBuffer((ChannelBuffer[]) this.decoder.pollAll(new ChannelBuffer[this.decoder.size()]));
    }

    private ChannelBuffer finishDecode() {
        ChannelBuffer result;
        if (this.decoder.finish()) {
            result = ChannelBuffers.wrappedBuffer((ChannelBuffer[]) this.decoder.pollAll(new ChannelBuffer[this.decoder.size()]));
        } else {
            result = ChannelBuffers.EMPTY_BUFFER;
        }
        this.decoder = null;
        return result;
    }
}
