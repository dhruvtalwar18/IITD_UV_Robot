package org.jboss.netty.channel.socket.oio;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PushbackInputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;
import org.jboss.netty.channel.socket.DefaultSocketChannelConfig;
import org.jboss.netty.channel.socket.SocketChannel;
import org.jboss.netty.channel.socket.SocketChannelConfig;

abstract class OioSocketChannel extends AbstractOioChannel implements SocketChannel {
    private final SocketChannelConfig config;
    final Socket socket;

    /* access modifiers changed from: package-private */
    public abstract PushbackInputStream getInputStream();

    /* access modifiers changed from: package-private */
    public abstract OutputStream getOutputStream();

    OioSocketChannel(Channel parent, ChannelFactory factory, ChannelPipeline pipeline, ChannelSink sink, Socket socket2) {
        super(parent, factory, pipeline, sink);
        this.socket = socket2;
        this.config = new DefaultSocketChannelConfig(socket2);
    }

    public SocketChannelConfig getConfig() {
        return this.config;
    }

    /* access modifiers changed from: package-private */
    public boolean isSocketBound() {
        return this.socket.isBound();
    }

    /* access modifiers changed from: package-private */
    public boolean isSocketConnected() {
        return this.socket.isConnected();
    }

    /* access modifiers changed from: package-private */
    public InetSocketAddress getLocalSocketAddress() throws Exception {
        return (InetSocketAddress) this.socket.getLocalSocketAddress();
    }

    /* access modifiers changed from: package-private */
    public InetSocketAddress getRemoteSocketAddress() throws Exception {
        return (InetSocketAddress) this.socket.getRemoteSocketAddress();
    }

    /* access modifiers changed from: package-private */
    public void closeSocket() throws IOException {
        this.socket.close();
    }

    /* access modifiers changed from: package-private */
    public boolean isSocketClosed() {
        return this.socket.isClosed();
    }
}
