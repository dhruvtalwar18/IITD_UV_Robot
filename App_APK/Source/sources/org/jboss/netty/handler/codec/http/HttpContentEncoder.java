package org.jboss.netty.handler.codec.http;

import java.util.Queue;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelHandler;
import org.jboss.netty.handler.codec.embedder.EncoderEmbedder;
import org.jboss.netty.util.internal.QueueFactory;

public abstract class HttpContentEncoder extends SimpleChannelHandler {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private final Queue<String> acceptEncodingQueue = QueueFactory.createQueue(String.class);
    private volatile EncoderEmbedder<ChannelBuffer> encoder;

    /* access modifiers changed from: protected */
    public abstract String getTargetContentEncoding(String str) throws Exception;

    /* access modifiers changed from: protected */
    public abstract EncoderEmbedder<ChannelBuffer> newContentEncoder(HttpMessage httpMessage, String str) throws Exception;

    protected HttpContentEncoder() {
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        Object msg = e.getMessage();
        if (!(msg instanceof HttpMessage)) {
            ctx.sendUpstream(e);
            return;
        }
        String acceptedEncoding = ((HttpMessage) msg).getHeader("Accept-Encoding");
        if (acceptedEncoding == null) {
            acceptedEncoding = "identity";
        }
        boolean offer = this.acceptEncodingQueue.offer(acceptedEncoding);
        ctx.sendUpstream(e);
    }

    public void writeRequested(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        Object msg = e.getMessage();
        if ((msg instanceof HttpResponse) && ((HttpResponse) msg).getStatus().getCode() == 100) {
            ctx.sendDownstream(e);
        } else if (msg instanceof HttpMessage) {
            HttpMessage m = (HttpMessage) msg;
            this.encoder = null;
            String acceptEncoding = this.acceptEncodingQueue.poll();
            if (acceptEncoding != null) {
                String contentEncoding = m.getHeader("Content-Encoding");
                if (contentEncoding == null || "identity".equalsIgnoreCase(contentEncoding)) {
                    if (m.isChunked() || m.getContent().readable()) {
                        EncoderEmbedder<ChannelBuffer> newContentEncoder = newContentEncoder(m, acceptEncoding);
                        this.encoder = newContentEncoder;
                        if (newContentEncoder != null) {
                            m.setHeader("Content-Encoding", (Object) getTargetContentEncoding(acceptEncoding));
                            if (!m.isChunked()) {
                                ChannelBuffer content = ChannelBuffers.wrappedBuffer(encode(m.getContent()), finishEncode());
                                m.setContent(content);
                                if (m.containsHeader("Content-Length")) {
                                    m.setHeader("Content-Length", (Object) Integer.toString(content.readableBytes()));
                                }
                            }
                        }
                    }
                    ctx.sendDownstream(e);
                    return;
                }
                ctx.sendDownstream(e);
                return;
            }
            throw new IllegalStateException("cannot send more responses than requests");
        } else if (msg instanceof HttpChunk) {
            HttpChunk c = (HttpChunk) msg;
            ChannelBuffer content2 = c.getContent();
            if (this.encoder == null) {
                ctx.sendDownstream(e);
            } else if (!c.isLast()) {
                ChannelBuffer content3 = encode(content2);
                if (content3.readable()) {
                    c.setContent(content3);
                    ctx.sendDownstream(e);
                }
            } else {
                ChannelBuffer lastProduct = finishEncode();
                if (lastProduct.readable()) {
                    Channels.write(ctx, Channels.succeededFuture(e.getChannel()), new DefaultHttpChunk(lastProduct), e.getRemoteAddress());
                }
                ctx.sendDownstream(e);
            }
        } else {
            ctx.sendDownstream(e);
        }
    }

    private ChannelBuffer encode(ChannelBuffer buf) {
        this.encoder.offer(buf);
        return ChannelBuffers.wrappedBuffer((ChannelBuffer[]) this.encoder.pollAll(new ChannelBuffer[this.encoder.size()]));
    }

    private ChannelBuffer finishEncode() {
        ChannelBuffer result;
        if (this.encoder.finish()) {
            result = ChannelBuffers.wrappedBuffer((ChannelBuffer[]) this.encoder.pollAll(new ChannelBuffer[this.encoder.size()]));
        } else {
            result = ChannelBuffers.EMPTY_BUFFER;
        }
        this.encoder = null;
        return result;
    }
}
