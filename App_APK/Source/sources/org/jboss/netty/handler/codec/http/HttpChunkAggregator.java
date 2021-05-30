package org.jboss.netty.handler.codec.http;

import java.util.List;
import java.util.Map;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.buffer.CompositeChannelBuffer;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.LifeCycleAwareChannelHandler;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;
import org.jboss.netty.handler.codec.frame.TooLongFrameException;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.util.CharsetUtil;

public class HttpChunkAggregator extends SimpleChannelUpstreamHandler implements LifeCycleAwareChannelHandler {
    private static final ChannelBuffer CONTINUE = ChannelBuffers.copiedBuffer((CharSequence) "HTTP/1.1 100 Continue\r\n\r\n", CharsetUtil.US_ASCII);
    public static final int DEFAULT_MAX_COMPOSITEBUFFER_COMPONENTS = 1024;
    private ChannelHandlerContext ctx;
    private HttpMessage currentMessage;
    private final int maxContentLength;
    private int maxCumulationBufferComponents = 1024;

    public HttpChunkAggregator(int maxContentLength2) {
        if (maxContentLength2 > 0) {
            this.maxContentLength = maxContentLength2;
            return;
        }
        throw new IllegalArgumentException("maxContentLength must be a positive integer: " + maxContentLength2);
    }

    public final int getMaxCumulationBufferComponents() {
        return this.maxCumulationBufferComponents;
    }

    public final void setMaxCumulationBufferComponents(int maxCumulationBufferComponents2) {
        if (maxCumulationBufferComponents2 < 2) {
            throw new IllegalArgumentException("maxCumulationBufferComponents: " + maxCumulationBufferComponents2 + " (expected: >= 2)");
        } else if (this.ctx == null) {
            this.maxCumulationBufferComponents = maxCumulationBufferComponents2;
        } else {
            throw new IllegalStateException("decoder properties cannot be changed once the decoder is added to a pipeline.");
        }
    }

    public void messageReceived(ChannelHandlerContext ctx2, MessageEvent e) throws Exception {
        Object msg = e.getMessage();
        HttpMessage currentMessage2 = this.currentMessage;
        if (msg instanceof HttpMessage) {
            HttpMessage m = (HttpMessage) msg;
            if (HttpHeaders.is100ContinueExpected(m)) {
                Channels.write(ctx2, Channels.succeededFuture(ctx2.getChannel()), (Object) CONTINUE.duplicate());
            }
            if (m.isChunked()) {
                List<String> encodings = m.getHeaders("Transfer-Encoding");
                encodings.remove(HttpHeaders.Values.CHUNKED);
                if (encodings.isEmpty()) {
                    m.removeHeader("Transfer-Encoding");
                }
                m.setChunked(false);
                this.currentMessage = m;
                return;
            }
            this.currentMessage = null;
            ctx2.sendUpstream(e);
        } else if (!(msg instanceof HttpChunk)) {
            ctx2.sendUpstream(e);
        } else if (currentMessage2 != null) {
            HttpChunk chunk = (HttpChunk) msg;
            ChannelBuffer content = currentMessage2.getContent();
            if (content.readableBytes() <= this.maxContentLength - chunk.getContent().readableBytes()) {
                appendToCumulation(chunk.getContent());
                if (chunk.isLast()) {
                    this.currentMessage = null;
                    if (chunk instanceof HttpChunkTrailer) {
                        for (Map.Entry<String, String> header : ((HttpChunkTrailer) chunk).getHeaders()) {
                            currentMessage2.setHeader(header.getKey(), (Object) header.getValue());
                        }
                    }
                    currentMessage2.setHeader("Content-Length", (Object) String.valueOf(content.readableBytes()));
                    Channels.fireMessageReceived(ctx2, (Object) currentMessage2, e.getRemoteAddress());
                    return;
                }
                return;
            }
            throw new TooLongFrameException("HTTP content length exceeded " + this.maxContentLength + " bytes.");
        } else {
            throw new IllegalStateException("received " + HttpChunk.class.getSimpleName() + " without " + HttpMessage.class.getSimpleName());
        }
    }

    /* access modifiers changed from: protected */
    public void appendToCumulation(ChannelBuffer input) {
        ChannelBuffer cumulation = this.currentMessage.getContent();
        if (cumulation instanceof CompositeChannelBuffer) {
            CompositeChannelBuffer composite = (CompositeChannelBuffer) cumulation;
            if (composite.numComponents() >= this.maxCumulationBufferComponents) {
                this.currentMessage.setContent(ChannelBuffers.wrappedBuffer(composite.copy(), input));
                return;
            }
            List<ChannelBuffer> decomposed = composite.decompose(0, composite.readableBytes());
            ChannelBuffer[] buffers = (ChannelBuffer[]) decomposed.toArray(new ChannelBuffer[(decomposed.size() + 1)]);
            buffers[buffers.length - 1] = input;
            this.currentMessage.setContent(ChannelBuffers.wrappedBuffer(buffers));
            return;
        }
        this.currentMessage.setContent(ChannelBuffers.wrappedBuffer(cumulation, input));
    }

    public void beforeAdd(ChannelHandlerContext ctx2) throws Exception {
        this.ctx = ctx2;
    }

    public void afterAdd(ChannelHandlerContext ctx2) throws Exception {
    }

    public void beforeRemove(ChannelHandlerContext ctx2) throws Exception {
    }

    public void afterRemove(ChannelHandlerContext ctx2) throws Exception {
    }
}
