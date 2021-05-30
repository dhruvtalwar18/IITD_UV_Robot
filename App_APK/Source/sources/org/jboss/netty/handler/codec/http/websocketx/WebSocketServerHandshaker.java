package org.jboss.netty.handler.codec.http.websocketx;

import java.util.LinkedHashSet;
import java.util.Set;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.handler.codec.http.HttpRequest;

public abstract class WebSocketServerHandshaker {
    public static final ChannelFutureListener HANDSHAKE_LISTENER = new ChannelFutureListener() {
        public void operationComplete(ChannelFuture future) throws Exception {
            if (!future.isSuccess()) {
                Channels.fireExceptionCaught(future.getChannel(), future.getCause());
            }
        }
    };
    private final long maxFramePayloadLength;
    private String selectedSubprotocol;
    private final String[] subprotocols;
    private final WebSocketVersion version;
    private final String webSocketUrl;

    public abstract ChannelFuture close(Channel channel, CloseWebSocketFrame closeWebSocketFrame);

    public abstract ChannelFuture handshake(Channel channel, HttpRequest httpRequest);

    protected WebSocketServerHandshaker(WebSocketVersion version2, String webSocketUrl2, String subprotocols2) {
        this(version2, webSocketUrl2, subprotocols2, Long.MAX_VALUE);
    }

    protected WebSocketServerHandshaker(WebSocketVersion version2, String webSocketUrl2, String subprotocols2, long maxFramePayloadLength2) {
        this.version = version2;
        this.webSocketUrl = webSocketUrl2;
        if (subprotocols2 != null) {
            String[] subprotocolArray = subprotocols2.split(",");
            for (int i = 0; i < subprotocolArray.length; i++) {
                subprotocolArray[i] = subprotocolArray[i].trim();
            }
            this.subprotocols = subprotocolArray;
        } else {
            this.subprotocols = new String[0];
        }
        this.maxFramePayloadLength = maxFramePayloadLength2;
    }

    public String getWebSocketUrl() {
        return this.webSocketUrl;
    }

    public Set<String> getSubprotocols() {
        Set<String> ret = new LinkedHashSet<>();
        for (String p : this.subprotocols) {
            ret.add(p);
        }
        return ret;
    }

    public WebSocketVersion getVersion() {
        return this.version;
    }

    public long getMaxFramePayloadLength() {
        return this.maxFramePayloadLength;
    }

    /* access modifiers changed from: protected */
    public String selectSubprotocol(String requestedSubprotocols) {
        if (requestedSubprotocols == null || this.subprotocols.length == 0) {
            return null;
        }
        for (String p : requestedSubprotocols.split(",")) {
            String requestedSubprotocol = p.trim();
            for (String supportedSubprotocol : this.subprotocols) {
                if (requestedSubprotocol.equals(supportedSubprotocol)) {
                    return requestedSubprotocol;
                }
            }
        }
        return null;
    }

    public String getSelectedSubprotocol() {
        return this.selectedSubprotocol;
    }

    /* access modifiers changed from: protected */
    public void setSelectedSubprotocol(String value) {
        this.selectedSubprotocol = value;
    }
}
