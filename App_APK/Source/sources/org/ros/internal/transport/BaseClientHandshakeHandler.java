package org.ros.internal.transport;

import java.util.concurrent.ExecutorService;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.MessageEvent;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.internal.transport.tcp.AbstractNamedChannelHandler;

public abstract class BaseClientHandshakeHandler extends AbstractNamedChannelHandler {
    /* access modifiers changed from: private */
    public final ClientHandshake clientHandshake;
    private final ListenerGroup<ClientHandshakeListener> clientHandshakeListeners;

    /* access modifiers changed from: protected */
    public abstract void onFailure(String str, ChannelHandlerContext channelHandlerContext, MessageEvent messageEvent);

    /* access modifiers changed from: protected */
    public abstract void onSuccess(ConnectionHeader connectionHeader, ChannelHandlerContext channelHandlerContext, MessageEvent messageEvent);

    public BaseClientHandshakeHandler(ClientHandshake clientHandshake2, ExecutorService executorService) {
        this.clientHandshake = clientHandshake2;
        this.clientHandshakeListeners = new ListenerGroup<>(executorService);
    }

    public void addListener(ClientHandshakeListener clientHandshakeListener) {
        this.clientHandshakeListeners.add(clientHandshakeListener);
    }

    public void channelConnected(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        super.channelConnected(ctx, e);
        e.getChannel().write(this.clientHandshake.getOutgoingConnectionHeader().encode());
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        ConnectionHeader connectionHeader = ConnectionHeader.decode((ChannelBuffer) e.getMessage());
        if (this.clientHandshake.handshake(connectionHeader)) {
            onSuccess(connectionHeader, ctx, e);
            signalOnSuccess(connectionHeader);
            return;
        }
        onFailure(this.clientHandshake.getErrorMessage(), ctx, e);
        signalOnFailure(this.clientHandshake.getErrorMessage());
    }

    private void signalOnSuccess(final ConnectionHeader incommingConnectionHeader) {
        this.clientHandshakeListeners.signal(new SignalRunnable<ClientHandshakeListener>() {
            public void run(ClientHandshakeListener listener) {
                listener.onSuccess(BaseClientHandshakeHandler.this.clientHandshake.getOutgoingConnectionHeader(), incommingConnectionHeader);
            }
        });
    }

    private void signalOnFailure(final String errorMessage) {
        this.clientHandshakeListeners.signal(new SignalRunnable<ClientHandshakeListener>() {
            public void run(ClientHandshakeListener listener) {
                listener.onFailure(BaseClientHandshakeHandler.this.clientHandshake.getOutgoingConnectionHeader(), errorMessage);
            }
        });
    }
}
