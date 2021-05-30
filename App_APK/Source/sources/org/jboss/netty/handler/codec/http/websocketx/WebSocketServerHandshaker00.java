package org.jboss.netty.handler.codec.http.websocketx;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.handler.codec.http.DefaultHttpResponse;
import org.jboss.netty.handler.codec.http.HttpChunkAggregator;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpRequest;
import org.jboss.netty.handler.codec.http.HttpRequestDecoder;
import org.jboss.netty.handler.codec.http.HttpResponse;
import org.jboss.netty.handler.codec.http.HttpResponseEncoder;
import org.jboss.netty.handler.codec.http.HttpResponseStatus;
import org.jboss.netty.handler.codec.http.HttpVersion;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;

public class WebSocketServerHandshaker00 extends WebSocketServerHandshaker {
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) WebSocketServerHandshaker00.class);

    public WebSocketServerHandshaker00(String webSocketURL, String subprotocols) {
        this(webSocketURL, subprotocols, Long.MAX_VALUE);
    }

    public WebSocketServerHandshaker00(String webSocketURL, String subprotocols, long maxFramePayloadLength) {
        super(WebSocketVersion.V00, webSocketURL, subprotocols, maxFramePayloadLength);
    }

    public ChannelFuture handshake(Channel channel, HttpRequest req) {
        boolean z = false;
        if (logger.isDebugEnabled()) {
            logger.debug(String.format("Channel %s WS Version 00 server handshake", new Object[]{channel.getId()}));
        }
        if (!"Upgrade".equalsIgnoreCase(req.getHeader("Connection")) || !HttpHeaders.Values.WEBSOCKET.equalsIgnoreCase(req.getHeader("Upgrade"))) {
            throw new WebSocketHandshakeException("not a WebSocket handshake request: missing upgrade");
        }
        if (req.containsHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY1) && req.containsHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY2)) {
            z = true;
        }
        boolean isHixie76 = z;
        HttpResponse res = new DefaultHttpResponse(HttpVersion.HTTP_1_1, new HttpResponseStatus(101, isHixie76 ? "WebSocket Protocol Handshake" : "Web Socket Protocol Handshake"));
        res.addHeader("Upgrade", HttpHeaders.Values.WEBSOCKET);
        res.addHeader("Connection", "Upgrade");
        if (isHixie76) {
            res.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_ORIGIN, req.getHeader("Origin"));
            res.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_LOCATION, getWebSocketUrl());
            String subprotocols = req.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL);
            if (subprotocols != null) {
                String selectedSubprotocol = selectSubprotocol(subprotocols);
                if (selectedSubprotocol != null) {
                    res.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL, selectedSubprotocol);
                    setSelectedSubprotocol(selectedSubprotocol);
                } else {
                    throw new WebSocketHandshakeException("Requested subprotocol(s) not supported: " + subprotocols);
                }
            }
            String key1 = req.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY1);
            String key2 = req.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY2);
            long c = req.getContent().readLong();
            ChannelBuffer input = ChannelBuffers.buffer(16);
            input.writeInt((int) (Long.parseLong(key1.replaceAll("[^0-9]", "")) / ((long) key1.replaceAll("[^ ]", "").length())));
            input.writeInt((int) (Long.parseLong(key2.replaceAll("[^0-9]", "")) / ((long) key2.replaceAll("[^ ]", "").length())));
            input.writeLong(c);
            res.setContent(ChannelBuffers.wrappedBuffer(WebSocketUtil.md5(input.array())));
        } else {
            res.addHeader(HttpHeaders.Names.WEBSOCKET_ORIGIN, req.getHeader("Origin"));
            res.addHeader(HttpHeaders.Names.WEBSOCKET_LOCATION, getWebSocketUrl());
            String protocol = req.getHeader(HttpHeaders.Names.WEBSOCKET_PROTOCOL);
            if (protocol != null) {
                res.addHeader(HttpHeaders.Names.WEBSOCKET_PROTOCOL, selectSubprotocol(protocol));
            }
        }
        ChannelPipeline p = channel.getPipeline();
        if (p.get(HttpChunkAggregator.class) != null) {
            p.remove(HttpChunkAggregator.class);
        }
        p.replace(HttpRequestDecoder.class, "wsdecoder", (ChannelHandler) new WebSocket00FrameDecoder(getMaxFramePayloadLength()));
        ChannelFuture future = channel.write(res);
        p.replace(HttpResponseEncoder.class, "wsencoder", (ChannelHandler) new WebSocket00FrameEncoder());
        return future;
    }

    public ChannelFuture close(Channel channel, CloseWebSocketFrame frame) {
        return channel.write(frame);
    }
}
