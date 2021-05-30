package org.jboss.netty.handler.codec.http.websocketx;

import java.net.URI;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Map;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.handler.codec.http.DefaultHttpRequest;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMethod;
import org.jboss.netty.handler.codec.http.HttpRequestEncoder;
import org.jboss.netty.handler.codec.http.HttpResponse;
import org.jboss.netty.handler.codec.http.HttpResponseDecoder;
import org.jboss.netty.handler.codec.http.HttpResponseStatus;
import org.jboss.netty.handler.codec.http.HttpVersion;

public class WebSocketClientHandshaker00 extends WebSocketClientHandshaker {
    private byte[] expectedChallengeResponseBytes;

    public WebSocketClientHandshaker00(URI webSocketURL, WebSocketVersion version, String subprotocol, Map<String, String> customHeaders) {
        this(webSocketURL, version, subprotocol, customHeaders, Long.MAX_VALUE);
    }

    public WebSocketClientHandshaker00(URI webSocketURL, WebSocketVersion version, String subprotocol, Map<String, String> customHeaders, long maxFramePayloadLength) {
        super(webSocketURL, version, subprotocol, customHeaders, maxFramePayloadLength);
    }

    public ChannelFuture handshake(Channel channel) {
        String path;
        int spaces1 = WebSocketUtil.randomNumber(1, 12);
        int spaces2 = WebSocketUtil.randomNumber(1, 12);
        int max1 = Integer.MAX_VALUE / spaces1;
        int max2 = Integer.MAX_VALUE / spaces2;
        int number1 = WebSocketUtil.randomNumber(0, max1);
        int number2 = WebSocketUtil.randomNumber(0, max2);
        String key1 = Integer.toString(number1 * spaces1);
        String key2 = Integer.toString(number2 * spaces2);
        String key12 = insertRandomCharacters(key1);
        String key22 = insertRandomCharacters(key2);
        String key13 = insertSpaces(key12, spaces1);
        String key23 = insertSpaces(key22, spaces2);
        byte[] key3 = WebSocketUtil.randomBytes(8);
        ByteBuffer buffer = ByteBuffer.allocate(4);
        buffer.putInt(number1);
        byte[] number1Array = buffer.array();
        ByteBuffer buffer2 = ByteBuffer.allocate(4);
        buffer2.putInt(number2);
        byte[] number2Array = buffer2.array();
        byte[] challenge = new byte[16];
        int i = spaces2;
        int i2 = max2;
        System.arraycopy(number1Array, 0, challenge, 0, 4);
        System.arraycopy(number2Array, 0, challenge, 4, 4);
        System.arraycopy(key3, 0, challenge, 8, 8);
        this.expectedChallengeResponseBytes = WebSocketUtil.md5(challenge);
        URI wsURL = getWebSocketUrl();
        String path2 = wsURL.getPath();
        if (wsURL.getQuery() == null || wsURL.getQuery().length() <= 0) {
            int i3 = spaces1;
            path = path2;
        } else {
            String str = path2;
            StringBuilder sb = new StringBuilder();
            int i4 = spaces1;
            sb.append(wsURL.getPath());
            sb.append("?");
            sb.append(wsURL.getQuery());
            path = sb.toString();
        }
        int i5 = max1;
        byte[] bArr = number2Array;
        DefaultHttpRequest defaultHttpRequest = new DefaultHttpRequest(HttpVersion.HTTP_1_1, HttpMethod.GET, path);
        defaultHttpRequest.addHeader("Upgrade", HttpHeaders.Values.WEBSOCKET);
        defaultHttpRequest.addHeader("Connection", "Upgrade");
        defaultHttpRequest.addHeader("Host", wsURL.getHost());
        int wsPort = wsURL.getPort();
        StringBuilder sb2 = new StringBuilder();
        String str2 = path;
        sb2.append("http://");
        sb2.append(wsURL.getHost());
        String originValue = sb2.toString();
        if (wsPort == 80 || wsPort == 443) {
        } else {
            StringBuilder sb3 = new StringBuilder();
            sb3.append(originValue);
            URI uri = wsURL;
            sb3.append(":");
            sb3.append(wsPort);
            originValue = sb3.toString();
        }
        defaultHttpRequest.addHeader("Origin", originValue);
        defaultHttpRequest.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY1, key13);
        defaultHttpRequest.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_KEY2, key23);
        String expectedSubprotocol = getExpectedSubprotocol();
        if (expectedSubprotocol != null && !expectedSubprotocol.equals("")) {
            defaultHttpRequest.addHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL, expectedSubprotocol);
        }
        if (this.customHeaders != null) {
            for (String header : this.customHeaders.keySet()) {
                String expectedSubprotocol2 = expectedSubprotocol;
                defaultHttpRequest.addHeader(header, this.customHeaders.get(header));
                expectedSubprotocol = expectedSubprotocol2;
                originValue = originValue;
            }
        }
        String str3 = originValue;
        defaultHttpRequest.setContent(ChannelBuffers.copiedBuffer(key3));
        ChannelFuture future = channel.write(defaultHttpRequest);
        DefaultHttpRequest defaultHttpRequest2 = defaultHttpRequest;
        channel.getPipeline().replace(HttpRequestEncoder.class, "ws-encoder", (ChannelHandler) new WebSocket00FrameEncoder());
        return future;
    }

    public void finishHandshake(Channel channel, HttpResponse response) throws WebSocketHandshakeException {
        if (response.getStatus().equals(new HttpResponseStatus(101, "WebSocket Protocol Handshake"))) {
            String upgrade = response.getHeader("Upgrade");
            if (upgrade == null || !upgrade.equals(HttpHeaders.Values.WEBSOCKET)) {
                throw new WebSocketHandshakeException("Invalid handshake response upgrade: " + response.getHeader("Upgrade"));
            }
            String connection = response.getHeader("Connection");
            if (connection == null || !connection.equals("Upgrade")) {
                throw new WebSocketHandshakeException("Invalid handshake response connection: " + response.getHeader("Connection"));
            } else if (Arrays.equals(response.getContent().array(), this.expectedChallengeResponseBytes)) {
                setActualSubprotocol(response.getHeader(HttpHeaders.Names.SEC_WEBSOCKET_PROTOCOL));
                setHandshakeComplete();
                ((HttpResponseDecoder) channel.getPipeline().get(HttpResponseDecoder.class)).replace("ws-decoder", new WebSocket00FrameDecoder(getMaxFramePayloadLength()));
            } else {
                throw new WebSocketHandshakeException("Invalid challenge");
            }
        } else {
            throw new WebSocketHandshakeException("Invalid handshake response status: " + response.getStatus());
        }
    }

    private static String insertRandomCharacters(String key) {
        int count = WebSocketUtil.randomNumber(1, 12);
        char[] randomChars = new char[count];
        int randCount = 0;
        while (randCount < count) {
            int rand = (int) ((Math.random() * 126.0d) + 33.0d);
            if ((33 < rand && rand < 47) || (58 < rand && rand < 126)) {
                randomChars[randCount] = (char) rand;
                randCount++;
            }
        }
        String key2 = key;
        for (int i = 0; i < count; i++) {
            int split = WebSocketUtil.randomNumber(0, key2.length());
            key2 = key2.substring(0, split) + randomChars[i] + key2.substring(split);
        }
        return key2;
    }

    private static String insertSpaces(String key, int spaces) {
        String key2 = key;
        for (int i = 0; i < spaces; i++) {
            int split = WebSocketUtil.randomNumber(1, key2.length() - 1);
            key2 = key2.substring(0, split) + " " + key2.substring(split);
        }
        return key2;
    }
}
