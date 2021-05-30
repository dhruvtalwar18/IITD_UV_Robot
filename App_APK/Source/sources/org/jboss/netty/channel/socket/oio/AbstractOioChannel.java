package org.jboss.netty.channel.socket.oio;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import org.jboss.netty.channel.AbstractChannel;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;
import org.jboss.netty.channel.socket.Worker;

abstract class AbstractOioChannel extends AbstractChannel {
    final Object interestOpsLock = new Object();
    private volatile InetSocketAddress localAddress;
    volatile InetSocketAddress remoteAddress;
    volatile Worker worker;
    volatile Thread workerThread;

    /* access modifiers changed from: package-private */
    public abstract void closeSocket() throws IOException;

    /* access modifiers changed from: package-private */
    public abstract InetSocketAddress getLocalSocketAddress() throws Exception;

    /* access modifiers changed from: package-private */
    public abstract InetSocketAddress getRemoteSocketAddress() throws Exception;

    /* access modifiers changed from: package-private */
    public abstract boolean isSocketBound();

    /* access modifiers changed from: package-private */
    public abstract boolean isSocketClosed();

    /* access modifiers changed from: package-private */
    public abstract boolean isSocketConnected();

    AbstractOioChannel(Channel parent, ChannelFactory factory, ChannelPipeline pipeline, ChannelSink sink) {
        super(parent, factory, pipeline, sink);
    }

    /* access modifiers changed from: protected */
    public boolean setClosed() {
        return super.setClosed();
    }

    /* access modifiers changed from: protected */
    public void setInterestOpsNow(int interestOps) {
        super.setInterestOpsNow(interestOps);
    }

    public ChannelFuture write(Object message, SocketAddress remoteAddress2) {
        if (remoteAddress2 == null || remoteAddress2.equals(getRemoteAddress())) {
            return super.write(message, (SocketAddress) null);
        }
        return super.write(message, remoteAddress2);
    }

    public boolean isBound() {
        return isOpen() && isSocketBound();
    }

    public boolean isConnected() {
        return isOpen() && isSocketConnected();
    }

    public InetSocketAddress getLocalAddress() {
        InetSocketAddress localAddress2 = this.localAddress;
        if (localAddress2 != null) {
            return localAddress2;
        }
        try {
            InetSocketAddress localSocketAddress = getLocalSocketAddress();
            InetSocketAddress localAddress3 = localSocketAddress;
            this.localAddress = localSocketAddress;
            return localAddress3;
        } catch (Throwable th) {
            return null;
        }
    }

    public InetSocketAddress getRemoteAddress() {
        InetSocketAddress remoteAddress2 = this.remoteAddress;
        if (remoteAddress2 != null) {
            return remoteAddress2;
        }
        try {
            InetSocketAddress remoteSocketAddress = getRemoteSocketAddress();
            InetSocketAddress remoteAddress3 = remoteSocketAddress;
            this.remoteAddress = remoteSocketAddress;
            return remoteAddress3;
        } catch (Throwable th) {
            return null;
        }
    }
}
