package org.jboss.netty.handler.codec.http;

import java.util.Queue;
import java.util.concurrent.atomic.AtomicLong;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.ChannelUpstreamHandler;
import org.jboss.netty.handler.codec.PrematureChannelClosureException;
import org.jboss.netty.handler.codec.http.HttpMessageDecoder;
import org.jboss.netty.util.internal.QueueFactory;

public class HttpClientCodec implements ChannelUpstreamHandler, ChannelDownstreamHandler {
    private final HttpResponseDecoder decoder;
    volatile boolean done;
    private final HttpRequestEncoder encoder;
    /* access modifiers changed from: private */
    public final boolean failOnMissingResponse;
    final Queue<HttpMethod> queue;
    /* access modifiers changed from: private */
    public final AtomicLong requestResponseCounter;

    public HttpClientCodec() {
        this(4096, 8192, 8192, false);
    }

    public HttpClientCodec(int maxInitialLineLength, int maxHeaderSize, int maxChunkSize) {
        this(maxInitialLineLength, maxHeaderSize, maxChunkSize, false);
    }

    public HttpClientCodec(int maxInitialLineLength, int maxHeaderSize, int maxChunkSize, boolean failOnMissingResponse2) {
        this.queue = QueueFactory.createQueue(HttpMethod.class);
        this.encoder = new Encoder();
        this.requestResponseCounter = new AtomicLong(0);
        this.decoder = new Decoder(maxInitialLineLength, maxHeaderSize, maxChunkSize);
        this.failOnMissingResponse = failOnMissingResponse2;
    }

    public void handleUpstream(ChannelHandlerContext ctx, ChannelEvent e) throws Exception {
        this.decoder.handleUpstream(ctx, e);
    }

    public void handleDownstream(ChannelHandlerContext ctx, ChannelEvent e) throws Exception {
        this.encoder.handleDownstream(ctx, e);
    }

    private final class Encoder extends HttpRequestEncoder {
        Encoder() {
        }

        /* access modifiers changed from: protected */
        public Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
            if ((msg instanceof HttpRequest) && !HttpClientCodec.this.done) {
                HttpClientCodec.this.queue.offer(((HttpRequest) msg).getMethod());
            }
            Object obj = super.encode(ctx, channel, msg);
            if (HttpClientCodec.this.failOnMissingResponse) {
                if ((msg instanceof HttpRequest) && !((HttpRequest) msg).isChunked()) {
                    HttpClientCodec.this.requestResponseCounter.incrementAndGet();
                } else if ((msg instanceof HttpChunk) && ((HttpChunk) msg).isLast()) {
                    HttpClientCodec.this.requestResponseCounter.incrementAndGet();
                }
            }
            return obj;
        }
    }

    private final class Decoder extends HttpResponseDecoder {
        Decoder(int maxInitialLineLength, int maxHeaderSize, int maxChunkSize) {
            super(maxInitialLineLength, maxHeaderSize, maxChunkSize);
        }

        /* access modifiers changed from: protected */
        public Object decode(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer, HttpMessageDecoder.State state) throws Exception {
            if (HttpClientCodec.this.done) {
                return buffer.readBytes(actualReadableBytes());
            }
            Object msg = super.decode(ctx, channel, buffer, state);
            if (HttpClientCodec.this.failOnMissingResponse) {
                decrement(msg);
            }
            return msg;
        }

        private void decrement(Object msg) {
            if (msg != null) {
                if ((msg instanceof HttpMessage) && !((HttpMessage) msg).isChunked()) {
                    HttpClientCodec.this.requestResponseCounter.decrementAndGet();
                } else if ((msg instanceof HttpChunk) && ((HttpChunk) msg).isLast()) {
                    HttpClientCodec.this.requestResponseCounter.decrementAndGet();
                } else if (msg instanceof Object[]) {
                    HttpClientCodec.this.requestResponseCounter.decrementAndGet();
                }
            }
        }

        /* access modifiers changed from: protected */
        public boolean isContentAlwaysEmpty(HttpMessage msg) {
            int statusCode = ((HttpResponse) msg).getStatus().getCode();
            if (statusCode == 100) {
                return true;
            }
            HttpMethod method = HttpClientCodec.this.queue.poll();
            char firstChar = method.getName().charAt(0);
            if (firstChar != 'C') {
                if (firstChar == 'H' && HttpMethod.HEAD.equals(method)) {
                    return true;
                }
            } else if (statusCode == 200 && HttpMethod.CONNECT.equals(method)) {
                HttpClientCodec.this.done = true;
                HttpClientCodec.this.queue.clear();
                return true;
            }
            return super.isContentAlwaysEmpty(msg);
        }

        public void channelClosed(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
            super.channelClosed(ctx, e);
            if (HttpClientCodec.this.failOnMissingResponse) {
                long missingResponses = HttpClientCodec.this.requestResponseCounter.get();
                if (missingResponses > 0) {
                    throw new PrematureChannelClosureException("Channel closed but still missing " + missingResponses + " response(s)");
                }
            }
        }
    }
}
