package org.jboss.netty.handler.codec.spdy;

import java.util.List;
import java.util.Map;
import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.handler.codec.http.HttpChunk;
import org.jboss.netty.handler.codec.http.HttpChunkTrailer;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMessage;
import org.jboss.netty.handler.codec.http.HttpRequest;
import org.jboss.netty.handler.codec.http.HttpResponse;
import org.jboss.netty.handler.codec.spdy.SpdyHttpHeaders;

public class SpdyHttpEncoder implements ChannelDownstreamHandler {
    private volatile int currentStreamID;
    private final int spdyVersion;

    @Deprecated
    public SpdyHttpEncoder() {
        this(2);
    }

    public SpdyHttpEncoder(int version) {
        if (version < 2 || version > 3) {
            throw new IllegalArgumentException("unsupported version: " + version);
        }
        this.spdyVersion = version;
    }

    public void handleDownstream(ChannelHandlerContext ctx, ChannelEvent evt) throws Exception {
        if (!(evt instanceof MessageEvent)) {
            ctx.sendDownstream(evt);
            return;
        }
        MessageEvent e = (MessageEvent) evt;
        Object msg = e.getMessage();
        if (msg instanceof HttpRequest) {
            HttpRequest httpRequest = (HttpRequest) msg;
            SpdySynStreamFrame spdySynStreamFrame = createSynStreamFrame(httpRequest);
            Channels.write(ctx, getContentFuture(ctx, e, spdySynStreamFrame.getStreamId(), httpRequest), spdySynStreamFrame, e.getRemoteAddress());
        } else if (msg instanceof HttpResponse) {
            HttpResponse httpResponse = (HttpResponse) msg;
            if (httpResponse.containsHeader(SpdyHttpHeaders.Names.ASSOCIATED_TO_STREAM_ID)) {
                SpdySynStreamFrame spdySynStreamFrame2 = createSynStreamFrame(httpResponse);
                Channels.write(ctx, getContentFuture(ctx, e, spdySynStreamFrame2.getStreamId(), httpResponse), spdySynStreamFrame2, e.getRemoteAddress());
                return;
            }
            SpdySynReplyFrame spdySynReplyFrame = createSynReplyFrame(httpResponse);
            Channels.write(ctx, getContentFuture(ctx, e, spdySynReplyFrame.getStreamId(), httpResponse), spdySynReplyFrame, e.getRemoteAddress());
        } else if (msg instanceof HttpChunk) {
            HttpChunk chunk = (HttpChunk) msg;
            SpdyDataFrame spdyDataFrame = new DefaultSpdyDataFrame(this.currentStreamID);
            spdyDataFrame.setData(chunk.getContent());
            spdyDataFrame.setLast(chunk.isLast());
            if (chunk instanceof HttpChunkTrailer) {
                List<Map.Entry<String, String>> trailers = ((HttpChunkTrailer) chunk).getHeaders();
                if (trailers.isEmpty()) {
                    Channels.write(ctx, e.getFuture(), spdyDataFrame, e.getRemoteAddress());
                    return;
                }
                SpdyHeadersFrame spdyHeadersFrame = new DefaultSpdyHeadersFrame(this.currentStreamID);
                for (Map.Entry<String, String> entry : trailers) {
                    spdyHeadersFrame.addHeader(entry.getKey(), entry.getValue());
                }
                ChannelFuture future = Channels.future(e.getChannel());
                future.addListener(new SpdyFrameWriter(ctx, e, spdyDataFrame));
                Channels.write(ctx, future, spdyHeadersFrame, e.getRemoteAddress());
                return;
            }
            Channels.write(ctx, e.getFuture(), spdyDataFrame, e.getRemoteAddress());
        } else {
            ctx.sendDownstream(evt);
        }
    }

    private ChannelFuture getContentFuture(ChannelHandlerContext ctx, MessageEvent e, int streamID, HttpMessage httpMessage) {
        if (httpMessage.getContent().readableBytes() == 0) {
            return e.getFuture();
        }
        SpdyDataFrame spdyDataFrame = new DefaultSpdyDataFrame(streamID);
        spdyDataFrame.setData(httpMessage.getContent());
        spdyDataFrame.setLast(true);
        ChannelFuture future = Channels.future(e.getChannel());
        future.addListener(new SpdyFrameWriter(ctx, e, spdyDataFrame));
        return future;
    }

    private class SpdyFrameWriter implements ChannelFutureListener {
        private final ChannelHandlerContext ctx;
        private final MessageEvent e;
        private final Object spdyFrame;

        SpdyFrameWriter(ChannelHandlerContext ctx2, MessageEvent e2, Object spdyFrame2) {
            this.ctx = ctx2;
            this.e = e2;
            this.spdyFrame = spdyFrame2;
        }

        public void operationComplete(ChannelFuture future) throws Exception {
            if (future.isSuccess()) {
                Channels.write(this.ctx, this.e.getFuture(), this.spdyFrame, this.e.getRemoteAddress());
            } else if (future.isCancelled()) {
                this.e.getFuture().cancel();
            } else {
                this.e.getFuture().setFailure(future.getCause());
            }
        }
    }

    private SpdySynStreamFrame createSynStreamFrame(HttpMessage httpMessage) throws Exception {
        boolean chunked = httpMessage.isChunked();
        int streamID = SpdyHttpHeaders.getStreamId(httpMessage);
        int associatedToStreamID = SpdyHttpHeaders.getAssociatedToStreamId(httpMessage);
        byte priority = SpdyHttpHeaders.getPriority(httpMessage);
        String URL = SpdyHttpHeaders.getUrl(httpMessage);
        String scheme = SpdyHttpHeaders.getScheme(httpMessage);
        SpdyHttpHeaders.removeStreamId(httpMessage);
        SpdyHttpHeaders.removeAssociatedToStreamId(httpMessage);
        SpdyHttpHeaders.removePriority(httpMessage);
        SpdyHttpHeaders.removeUrl(httpMessage);
        SpdyHttpHeaders.removeScheme(httpMessage);
        httpMessage.removeHeader("Connection");
        httpMessage.removeHeader("Keep-Alive");
        httpMessage.removeHeader("Proxy-Connection");
        httpMessage.removeHeader("Transfer-Encoding");
        SpdySynStreamFrame spdySynStreamFrame = new DefaultSpdySynStreamFrame(streamID, associatedToStreamID, priority);
        if (httpMessage instanceof HttpRequest) {
            HttpRequest httpRequest = (HttpRequest) httpMessage;
            SpdyHeaders.setMethod(this.spdyVersion, spdySynStreamFrame, httpRequest.getMethod());
            SpdyHeaders.setUrl(this.spdyVersion, spdySynStreamFrame, httpRequest.getUri());
            SpdyHeaders.setVersion(this.spdyVersion, spdySynStreamFrame, httpMessage.getProtocolVersion());
        }
        if (httpMessage instanceof HttpResponse) {
            SpdyHeaders.setStatus(this.spdyVersion, spdySynStreamFrame, ((HttpResponse) httpMessage).getStatus());
            SpdyHeaders.setUrl(this.spdyVersion, spdySynStreamFrame, URL);
            spdySynStreamFrame.setUnidirectional(true);
        }
        if (this.spdyVersion >= 3) {
            String host = HttpHeaders.getHost(httpMessage);
            httpMessage.removeHeader("Host");
            SpdyHeaders.setHost(spdySynStreamFrame, host);
        }
        if (scheme == null) {
            scheme = "https";
        }
        SpdyHeaders.setScheme(this.spdyVersion, spdySynStreamFrame, scheme);
        for (Map.Entry<String, String> entry : httpMessage.getHeaders()) {
            spdySynStreamFrame.addHeader(entry.getKey(), entry.getValue());
        }
        boolean z = false;
        if (chunked) {
            this.currentStreamID = streamID;
            spdySynStreamFrame.setLast(false);
        } else {
            if (httpMessage.getContent().readableBytes() == 0) {
                z = true;
            }
            spdySynStreamFrame.setLast(z);
        }
        return spdySynStreamFrame;
    }

    private SpdySynReplyFrame createSynReplyFrame(HttpResponse httpResponse) throws Exception {
        boolean chunked = httpResponse.isChunked();
        int streamID = SpdyHttpHeaders.getStreamId(httpResponse);
        SpdyHttpHeaders.removeStreamId(httpResponse);
        httpResponse.removeHeader("Connection");
        httpResponse.removeHeader("Keep-Alive");
        httpResponse.removeHeader("Proxy-Connection");
        httpResponse.removeHeader("Transfer-Encoding");
        SpdySynReplyFrame spdySynReplyFrame = new DefaultSpdySynReplyFrame(streamID);
        SpdyHeaders.setStatus(this.spdyVersion, spdySynReplyFrame, httpResponse.getStatus());
        SpdyHeaders.setVersion(this.spdyVersion, spdySynReplyFrame, httpResponse.getProtocolVersion());
        for (Map.Entry<String, String> entry : httpResponse.getHeaders()) {
            spdySynReplyFrame.addHeader(entry.getKey(), entry.getValue());
        }
        boolean z = false;
        if (chunked) {
            this.currentStreamID = streamID;
            spdySynReplyFrame.setLast(false);
        } else {
            if (httpResponse.getContent().readableBytes() == 0) {
                z = true;
            }
            spdySynReplyFrame.setLast(z);
        }
        return spdySynReplyFrame;
    }
}
