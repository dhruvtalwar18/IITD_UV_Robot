package org.jboss.netty.handler.codec.http.websocketx;

import java.net.URI;
import java.util.Map;

public class WebSocketClientHandshakerFactory {
    public WebSocketClientHandshaker newHandshaker(URI webSocketURL, WebSocketVersion version, String subprotocol, boolean allowExtensions, Map<String, String> customHeaders) throws WebSocketHandshakeException {
        return newHandshaker(webSocketURL, version, subprotocol, allowExtensions, customHeaders, Long.MAX_VALUE);
    }

    public WebSocketClientHandshaker newHandshaker(URI webSocketURL, WebSocketVersion version, String subprotocol, boolean allowExtensions, Map<String, String> customHeaders, long maxFramePayloadLength) throws WebSocketHandshakeException {
        WebSocketVersion webSocketVersion = version;
        if (webSocketVersion == WebSocketVersion.V13) {
            return new WebSocketClientHandshaker13(webSocketURL, version, subprotocol, allowExtensions, customHeaders, maxFramePayloadLength);
        }
        if (webSocketVersion == WebSocketVersion.V08) {
            return new WebSocketClientHandshaker08(webSocketURL, version, subprotocol, allowExtensions, customHeaders, maxFramePayloadLength);
        }
        if (webSocketVersion == WebSocketVersion.V00) {
            return new WebSocketClientHandshaker00(webSocketURL, version, subprotocol, customHeaders, maxFramePayloadLength);
        }
        throw new WebSocketHandshakeException("Protocol version " + version.toString() + " not supported.");
    }
}
