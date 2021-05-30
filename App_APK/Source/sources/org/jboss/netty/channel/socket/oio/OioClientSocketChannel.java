package org.jboss.netty.channel.socket.oio;

import java.io.OutputStream;
import java.io.PushbackInputStream;
import java.net.Socket;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;
import org.jboss.netty.channel.Channels;

class OioClientSocketChannel extends OioSocketChannel {
    volatile PushbackInputStream in;
    volatile OutputStream out;

    OioClientSocketChannel(ChannelFactory factory, ChannelPipeline pipeline, ChannelSink sink) {
        super((Channel) null, factory, pipeline, sink, new Socket());
        Channels.fireChannelOpen((Channel) this);
    }

    /* access modifiers changed from: package-private */
    public PushbackInputStream getInputStream() {
        return this.in;
    }

    /* access modifiers changed from: package-private */
    public OutputStream getOutputStream() {
        return this.out;
    }
}
