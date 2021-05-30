package org.jboss.netty.handler.codec.http.websocketx;

import java.net.URI;
import java.util.Map;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.handler.codec.http.DefaultHttpRequest;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMethod;
import org.jboss.netty.handler.codec.http.HttpRequest;
import org.jboss.netty.handler.codec.http.HttpRequestEncoder;
import org.jboss.netty.handler.codec.http.HttpResponse;
import org.jboss.netty.handler.codec.http.HttpResponseDecoder;
import org.jboss.netty.handler.codec.http.HttpResponseStatus;
import org.jboss.netty.handler.codec.http.HttpVersion;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.CharsetUtil;

public class WebSocketClientHandshaker08 extends WebSocketClientHandshaker {
    public static final String MAGIC_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) WebSocketClientHandshaker08.class);
    private final boolean allowExtensions;
    private String expectedChallengeResponseString;

    public WebSocketClientHandshaker08(URI webSocketURL, WebSocketVersion version, String subprotocol, boolean allowExtensions2, Map<String, String> customHeaders) {
        this(webSocketURL, version, subprotocol, allowExtensions2, customHeaders, Long.MAX_VALUE);
    }

    public WebSocketClientHandshaker08(URI webSocketURL, WebSocketVersion version, String subprotocol, boolean allowExtensions2, Map<String, String> customHeaders, long maxFramePayloadLength) {
        super(webSocketURL, version, subprotocol, customHeaders, maxFramePayloadLength);
        this.allowExtensions = allowExtensions2;
    }

    public ChannelFuture handshake(Channel channel) throws Exception {
        URI wsURL = getWebSocketUrl();
        String path = wsURL.getPath();
        if (wsURL.getQuery() != null && wsURL.getQuery().length() > 0) {
            path = wsURL.getPath() + "?" + wsURL.getQuery();
        }
        String key = WebSocketUtil.base64(WebSocketUtil.randomBytes(16));
        this.expectedChallengeResponseString = WebSocketUtil.base64(WebSocketUtil.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").getBytes(CharsetUtil.US_ASCII.name())));
        if (logger.isDebugEnabled()) {
            logger.debug(String.format("WS Version 08 Client Handshake key: %s. Expected response: %s.", new Object[]{key, this.expectedChallengeResponseString}));
        }
        HttpRequest request = new DefaultHttpRequest(HttpVersion.HTTP_1_1, HttpMethod.GET, path);
        request.addHeader("Upgrade", HttpHeaders.Values.WEBSOCKET.toLowerCase());
        request.addHeader("Connection", "Upgrade");
        request.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY, key);
        request.addHeader("Host", wsURL.getHost());
        int wsPort = wsURL.getPort();
        String originValue = "http://" + wsURL.getHost();
        if (!(wsPort == 80 || wsPort == 443)) {
            originValue = originValue + ":" + wsPort;
        }
        request.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_ORIGIN, originValue);
        String expectedSubprotocol = getExpectedSubprotocol();
        if (expectedSubprotocol != null && !expectedSubprotocol.equals("")) {
            request.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL, expectedSubprotocol);
        }
        request.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_VERSION, "8");
        if (this.customHeaders != null) {
            for (String header : this.customHeaders.keySet()) {
                request.addHeader(header, this.customHeaders.get(header));
            }
        }
        ChannelFuture future = channel.write(request);
        URI uri = wsURL;
        channel.getPipeline().replace(HttpRequestEncoder.class, "ws-encoder", (ChannelHandler) new WebSocket08FrameEncoder(true));
        return future;
    }

    public void finishHandshake(Channel channel, HttpResponse response) {
        if (response.getStatus().equals(HttpResponseStatus.SWITCHING_PROTOCOLS)) {
            String upgrade = response.getHeader("Upgrade");
            if (upgrade == null || !upgrade.toLowerCase().equals(HttpHeaders.Values.WEBSOCKET.toLowerCase())) {
                throw new WebSocketHandshakeException("Invalid handshake response upgrade: " + response.getHeader("Upgrade"));
            }
            String connection = response.getHeader("Connection");
            if (connection == null || !connection.toLowerCase().equals("Upgrade".toLowerCase())) {
                throw new WebSocketHandshakeException("Invalid handshake response connection: " + response.getHeader("Connection"));
            }
            String accept = response.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_ACCEPT);
            if (accept == null || !accept.equals(this.expectedChallengeResponseString)) {
                throw new WebSocketHandshakeException(String.format("Invalid challenge. Actual: %s. Expected: %s", new Object[]{accept, this.expectedChallengeResponseString}));
            }
            setActualSubprotocol(response.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL));
            setHandshakeComplete();
            ((HttpResponseDecoder) channel.getPipeline().get(HttpResponseDecoder.class)).replace("ws-decoder", new WebSocket08FrameDecoder(false, this.allowExtensions, getMaxFramePayloadLength()));
            return;
        }
        throw new WebSocketHandshakeException("Invalid handshake response status: " + response.getStatus());
    }
}
