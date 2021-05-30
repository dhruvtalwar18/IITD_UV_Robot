package org.jboss.netty.channel.socket.nio;

import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.channels.SocketChannel;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;

public class NioSocketChannel extends AbstractNioChannel<SocketChannel> implements org.jboss.netty.channel.socket.SocketChannel {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private static final int ST_BOUND = 1;
    private static final int ST_CLOSED = -1;
    private static final int ST_CONNECTED = 2;
    private static final int ST_OPEN = 0;
    private final NioSocketChannelConfig config;
    volatile int state = 0;

    public /* bridge */ /* synthetic */ int getInterestOps() {
        return super.getInterestOps();
    }

    public /* bridge */ /* synthetic */ InetSocketAddress getLocalAddress() {
        return super.getLocalAddress();
    }

    public /* bridge */ /* synthetic */ InetSocketAddress getRemoteAddress() {
        return super.getRemoteAddress();
    }

    public NioSocketChannel(Channel parent, ChannelFactory factory, ChannelPipeline pipeline, ChannelSink sink, SocketChannel socket, NioWorker worker) {
        super(parent, factory, pipeline, sink, worker, socket);
        this.config = new DefaultNioSocketChannelConfig(socket.socket());
    }

    public NioWorker getWorker() {
        return (NioWorker) super.getWorker();
    }

    public NioSocketChannelConfig getConfig() {
        return this.config;
    }

    public boolean isOpen() {
        return this.state >= 0;
    }

    public boolean isBound() {
        return this.state >= 1;
    }

    public boolean isConnected() {
        return this.state == 2;
    }

    /* access modifiers changed from: package-private */
    public final void setBound() {
        this.state = 1;
    }

    /* access modifiers changed from: package-private */
    public final void setConnected() {
        if (this.state != -1) {
            this.state = 2;
        }
    }

    /* access modifiers changed from: protected */
    public boolean setClosed() {
        this.state = -1;
        return super.setClosed();
    }

    /* access modifiers changed from: package-private */
    public InetSocketAddress getLocalSocketAddress() throws Exception {
        return (InetSocketAddress) ((SocketChannel) this.channel).socket().getLocalSocketAddress();
    }

    /* access modifiers changed from: package-private */
    public InetSocketAddress getRemoteSocketAddress() throws Exception {
        return (InetSocketAddress) ((SocketChannel) this.channel).socket().getRemoteSocketAddress();
    }

    public ChannelFuture write(Object message, SocketAddress remoteAddress) {
        if (remoteAddress == null || remoteAddress.equals(getRemoteAddress())) {
            return super.write(message, (SocketAddress) null);
        }
        return getUnsupportedOperationFuture();
    }
}
