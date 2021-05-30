package org.jboss.netty.handler.codec.spdy;

import java.util.HashMap;
import java.util.Map;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.handler.codec.frame.TooLongFrameException;
import org.jboss.netty.handler.codec.http.DefaultHttpRequest;
import org.jboss.netty.handler.codec.http.DefaultHttpResponse;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMessage;
import org.jboss.netty.handler.codec.http.HttpMethod;
import org.jboss.netty.handler.codec.http.HttpRequest;
import org.jboss.netty.handler.codec.http.HttpResponse;
import org.jboss.netty.handler.codec.http.HttpResponseStatus;
import org.jboss.netty.handler.codec.http.HttpVersion;
import org.jboss.netty.handler.codec.oneone.OneToOneDecoder;

public class SpdyHttpDecoder extends OneToOneDecoder {
    private final int maxContentLength;
    private final Map<Integer, HttpMessage> messageMap;
    private final int spdyVersion;

    @Deprecated
    public SpdyHttpDecoder(int maxContentLength2) {
        this(2, maxContentLength2);
    }

    public SpdyHttpDecoder(int version, int maxContentLength2) {
        this.messageMap = new HashMap();
        if (version < 2 || version > 3) {
            throw new IllegalArgumentException("unsupported version: " + version);
        } else if (maxContentLength2 > 0) {
            this.spdyVersion = version;
            this.maxContentLength = maxContentLength2;
        } else {
            throw new IllegalArgumentException("maxContentLength must be a positive integer: " + maxContentLength2);
        }
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        if (msg instanceof SpdySynStreamFrame) {
            SpdySynStreamFrame spdySynStreamFrame = (SpdySynStreamFrame) msg;
            int streamID = spdySynStreamFrame.getStreamId();
            if (SpdyCodecUtil.isServerId(streamID)) {
                int associatedToStreamID = spdySynStreamFrame.getAssociatedToStreamId();
                if (associatedToStreamID == 0) {
                    Channels.write(ctx, Channels.future(channel), (Object) new DefaultSpdyRstStreamFrame(streamID, SpdyStreamStatus.INVALID_STREAM));
                }
                String URL = SpdyHeaders.getUrl(this.spdyVersion, spdySynStreamFrame);
                if (URL == null) {
                    Channels.write(ctx, Channels.future(channel), (Object) new DefaultSpdyRstStreamFrame(streamID, SpdyStreamStatus.PROTOCOL_ERROR));
                }
                try {
                    HttpResponse httpResponse = createHttpResponse(this.spdyVersion, spdySynStreamFrame);
                    SpdyHttpHeaders.setStreamId(httpResponse, streamID);
                    SpdyHttpHeaders.setAssociatedToStreamId(httpResponse, associatedToStreamID);
                    SpdyHttpHeaders.setPriority(httpResponse, spdySynStreamFrame.getPriority());
                    SpdyHttpHeaders.setUrl(httpResponse, URL);
                    if (spdySynStreamFrame.isLast()) {
                        HttpHeaders.setContentLength(httpResponse, 0);
                        return httpResponse;
                    }
                    this.messageMap.put(new Integer(streamID), httpResponse);
                } catch (Exception e) {
                    Channels.write(ctx, Channels.future(channel), (Object) new DefaultSpdyRstStreamFrame(streamID, SpdyStreamStatus.PROTOCOL_ERROR));
                }
            } else {
                try {
                    HttpRequest httpRequest = createHttpRequest(this.spdyVersion, spdySynStreamFrame);
                    SpdyHttpHeaders.setStreamId(httpRequest, streamID);
                    if (spdySynStreamFrame.isLast()) {
                        return httpRequest;
                    }
                    this.messageMap.put(new Integer(streamID), httpRequest);
                } catch (Exception e2) {
                    SpdySynReplyFrame spdySynReplyFrame = new DefaultSpdySynReplyFrame(streamID);
                    spdySynReplyFrame.setLast(true);
                    SpdyHeaders.setStatus(this.spdyVersion, spdySynReplyFrame, HttpResponseStatus.BAD_REQUEST);
                    SpdyHeaders.setVersion(this.spdyVersion, spdySynReplyFrame, HttpVersion.HTTP_1_0);
                    Channels.write(ctx, Channels.future(channel), (Object) spdySynReplyFrame);
                }
            }
        } else if (msg instanceof SpdySynReplyFrame) {
            SpdySynReplyFrame spdySynReplyFrame2 = (SpdySynReplyFrame) msg;
            int streamID2 = spdySynReplyFrame2.getStreamId();
            try {
                HttpResponse httpResponse2 = createHttpResponse(this.spdyVersion, spdySynReplyFrame2);
                SpdyHttpHeaders.setStreamId(httpResponse2, streamID2);
                if (spdySynReplyFrame2.isLast()) {
                    HttpHeaders.setContentLength(httpResponse2, 0);
                    return httpResponse2;
                }
                this.messageMap.put(new Integer(streamID2), httpResponse2);
            } catch (Exception e3) {
                Channels.write(ctx, Channels.future(channel), (Object) new DefaultSpdyRstStreamFrame(streamID2, SpdyStreamStatus.PROTOCOL_ERROR));
            }
        } else if (msg instanceof SpdyHeadersFrame) {
            SpdyHeadersFrame spdyHeadersFrame = (SpdyHeadersFrame) msg;
            HttpMessage httpMessage = this.messageMap.get(new Integer(spdyHeadersFrame.getStreamId()));
            if (httpMessage == null) {
                return null;
            }
            for (Map.Entry<String, String> e4 : spdyHeadersFrame.getHeaders()) {
                httpMessage.addHeader(e4.getKey(), e4.getValue());
            }
        } else if (msg instanceof SpdyDataFrame) {
            SpdyDataFrame spdyDataFrame = (SpdyDataFrame) msg;
            Integer streamID3 = new Integer(spdyDataFrame.getStreamId());
            HttpMessage httpMessage2 = this.messageMap.get(streamID3);
            if (httpMessage2 == null) {
                return null;
            }
            ChannelBuffer content = httpMessage2.getContent();
            if (content.readableBytes() <= this.maxContentLength - spdyDataFrame.getData().readableBytes()) {
                if (content == ChannelBuffers.EMPTY_BUFFER) {
                    content = ChannelBuffers.dynamicBuffer(channel.getConfig().getBufferFactory());
                    content.writeBytes(spdyDataFrame.getData());
                    httpMessage2.setContent(content);
                } else {
                    content.writeBytes(spdyDataFrame.getData());
                }
                if (spdyDataFrame.isLast()) {
                    HttpHeaders.setContentLength(httpMessage2, (long) content.readableBytes());
                    this.messageMap.remove(streamID3);
                    return httpMessage2;
                }
            } else {
                this.messageMap.remove(streamID3);
                throw new TooLongFrameException("HTTP content length exceeded " + this.maxContentLength + " bytes.");
            }
        } else if (msg instanceof SpdyRstStreamFrame) {
            this.messageMap.remove(new Integer(((SpdyRstStreamFrame) msg).getStreamId()));
        }
        return null;
    }

    private static HttpRequest createHttpRequest(int spdyVersion2, SpdyHeaderBlock requestFrame) throws Exception {
        HttpMethod method = SpdyHeaders.getMethod(spdyVersion2, requestFrame);
        String url = SpdyHeaders.getUrl(spdyVersion2, requestFrame);
        HttpVersion httpVersion = SpdyHeaders.getVersion(spdyVersion2, requestFrame);
        SpdyHeaders.removeMethod(spdyVersion2, requestFrame);
        SpdyHeaders.removeUrl(spdyVersion2, requestFrame);
        SpdyHeaders.removeVersion(spdyVersion2, requestFrame);
        HttpRequest httpRequest = new DefaultHttpRequest(httpVersion, method, url);
        SpdyHeaders.removeScheme(spdyVersion2, requestFrame);
        if (spdyVersion2 >= 3) {
            String host = SpdyHeaders.getHost(requestFrame);
            SpdyHeaders.removeHost(requestFrame);
            HttpHeaders.setHost(httpRequest, host);
        }
        for (Map.Entry<String, String> e : requestFrame.getHeaders()) {
            httpRequest.addHeader(e.getKey(), e.getValue());
        }
        HttpHeaders.setKeepAlive(httpRequest, true);
        httpRequest.removeHeader("Transfer-Encoding");
        return httpRequest;
    }

    private static HttpResponse createHttpResponse(int spdyVersion2, SpdyHeaderBlock responseFrame) throws Exception {
        HttpResponseStatus status = SpdyHeaders.getStatus(spdyVersion2, responseFrame);
        HttpVersion version = SpdyHeaders.getVersion(spdyVersion2, responseFrame);
        SpdyHeaders.removeStatus(spdyVersion2, responseFrame);
        SpdyHeaders.removeVersion(spdyVersion2, responseFrame);
        HttpResponse httpResponse = new DefaultHttpResponse(version, status);
        for (Map.Entry<String, String> e : responseFrame.getHeaders()) {
            httpResponse.addHeader(e.getKey(), e.getValue());
        }
        HttpHeaders.setKeepAlive(httpResponse, true);
        httpResponse.removeHeader("Transfer-Encoding");
        httpResponse.removeHeader("Trailer");
        return httpResponse;
    }
}
