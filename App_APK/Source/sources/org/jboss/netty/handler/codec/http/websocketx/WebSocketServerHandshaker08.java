package org.jboss.netty.handler.codec.http.websocketx;

import java.io.UnsupportedEncodingException;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.Channels;
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
import org.jboss.netty.util.CharsetUtil;

public class WebSocketServerHandshaker08 extends WebSocketServerHandshaker {
    public static final String WEBSOCKET_08_ACCEPT_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) WebSocketServerHandshaker08.class);
    private final boolean allowExtensions;

    public WebSocketServerHandshaker08(String webSocketURL, String subprotocols, boolean allowExtensions2) {
        this(webSocketURL, subprotocols, allowExtensions2, Long.MAX_VALUE);
    }

    public WebSocketServerHandshaker08(String webSocketURL, String subprotocols, boolean allowExtensions2, long maxFramePayloadLength) {
        super(WebSocketVersion.V08, webSocketURL, subprotocols, maxFramePayloadLength);
        this.allowExtensions = allowExtensions2;
    }

    public ChannelFuture handshake(Channel channel, HttpRequest req) {
        Channel channel2 = channel;
        HttpRequest httpRequest = req;
        if (logger.isDebugEnabled()) {
            logger.debug(String.format("Channel %s WS Version 8 server handshake", new Object[]{channel.getId()}));
        }
        HttpResponse res = new DefaultHttpResponse(HttpVersion.HTTP_1_1, HttpResponseStatus.SWITCHING_PROTOCOLS);
        String key = httpRequest.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY);
        if (key != null) {
            try {
                byte[] sha1 = WebSocketUtil.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").getBytes(CharsetUtil.US_ASCII.name()));
                String accept = WebSocketUtil.base64(sha1);
                if (logger.isDebugEnabled()) {
                    logger.debug(String.format("WS Version 8 Server Handshake key: %s. Response: %s.", new Object[]{key, accept}));
                }
                res.setStatus(HttpResponseStatus.SWITCHING_PROTOCOLS);
                res.addHeader("Upgrade", HttpHeaders.Values.WEBSOCKET.toLowerCase());
                res.addHeader("Connection", "Upgrade");
                res.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_ACCEPT, accept);
                String subprotocols = httpRequest.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL);
                if (subprotocols != null) {
                    String selectedSubprotocol = selectSubprotocol(subprotocols);
                    if (selectedSubprotocol != null) {
                        res.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL, selectedSubprotocol);
                        setSelectedSubprotocol(selectedSubprotocol);
                    } else {
                        throw new WebSocketHandshakeException("Requested subprotocol(s) not supported: " + subprotocols);
                    }
                }
                ChannelFuture future = channel2.write(res);
                ChannelPipeline p = channel.getPipeline();
                if (p.get(HttpChunkAggregator.class) != null) {
                    p.remove(HttpChunkAggregator.class);
                }
                byte[] bArr = sha1;
                p.replace(HttpRequestDecoder.class, "wsdecoder", (ChannelHandler) new WebSocket08FrameDecoder(true, this.allowExtensions, getMaxFramePayloadLength()));
                p.replace(HttpResponseEncoder.class, "wsencoder", (ChannelHandler) new WebSocket08FrameEncoder(false));
                return future;
            } catch (UnsupportedEncodingException e) {
                return Channels.failedFuture(channel2, e);
            }
        } else {
            throw new WebSocketHandshakeException("not a WebSocket request: missing key");
        }
    }

    public ChannelFuture close(Channel channel, CloseWebSocketFrame frame) {
        ChannelFuture f = channel.write(frame);
        f.addListener(ChannelFutureListener.CLOSE);
        return f;
    }
}
