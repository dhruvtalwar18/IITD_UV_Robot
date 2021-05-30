package org.ros.internal.transport.tcp;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.net.SocketAddress;
import java.nio.ByteOrder;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.bootstrap.ClientBootstrap;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.buffer.HeapChannelBufferFactory;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.channel.socket.nio.NioClientSocketChannelFactory;
import org.ros.exception.RosRuntimeException;

public class TcpClient {
    private static final boolean DEBUG = false;
    private static final int DEFAULT_CONNECTION_TIMEOUT_DURATION = 5;
    private static final TimeUnit DEFAULT_CONNECTION_TIMEOUT_UNIT = TimeUnit.SECONDS;
    private static final boolean DEFAULT_KEEP_ALIVE = true;
    private static final Log log = LogFactory.getLog(TcpClient.class);
    private final ClientBootstrap bootstrap = new ClientBootstrap(this.channelFactory);
    private Channel channel;
    private final ChannelBufferFactory channelBufferFactory = new HeapChannelBufferFactory(ByteOrder.LITTLE_ENDIAN);
    private final ChannelFactory channelFactory;
    private final ChannelGroup channelGroup;
    /* access modifiers changed from: private */
    public final List<NamedChannelHandler> namedChannelHandlers;

    public TcpClient(ChannelGroup channelGroup2, Executor executor) {
        this.channelGroup = channelGroup2;
        this.channelFactory = new NioClientSocketChannelFactory(executor, executor);
        this.bootstrap.setOption("bufferFactory", this.channelBufferFactory);
        setConnectionTimeout(5, DEFAULT_CONNECTION_TIMEOUT_UNIT);
        setKeepAlive(true);
        this.namedChannelHandlers = Lists.newArrayList();
    }

    public void setConnectionTimeout(long duration, TimeUnit unit) {
        this.bootstrap.setOption("connectionTimeoutMillis", Long.valueOf(TimeUnit.MILLISECONDS.convert(duration, unit)));
    }

    public void setKeepAlive(boolean value) {
        this.bootstrap.setOption("keepAlive", Boolean.valueOf(value));
    }

    public void addNamedChannelHandler(NamedChannelHandler namedChannelHandler) {
        this.namedChannelHandlers.add(namedChannelHandler);
    }

    public void addAllNamedChannelHandlers(List<NamedChannelHandler> namedChannelHandlers2) {
        this.namedChannelHandlers.addAll(namedChannelHandlers2);
    }

    public void connect(String connectionName, SocketAddress socketAddress) {
        this.bootstrap.setPipelineFactory(new TcpClientPipelineFactory(this.channelGroup) {
            public ChannelPipeline getPipeline() {
                ChannelPipeline pipeline = super.getPipeline();
                for (NamedChannelHandler namedChannelHandler : TcpClient.this.namedChannelHandlers) {
                    pipeline.addLast(namedChannelHandler.getName(), namedChannelHandler);
                }
                return pipeline;
            }
        });
        ChannelFuture future = this.bootstrap.connect(socketAddress).awaitUninterruptibly();
        if (future.isSuccess()) {
            this.channel = future.getChannel();
            return;
        }
        throw new RosRuntimeException("Connection exception: " + socketAddress, future.getCause());
    }

    public Channel getChannel() {
        return this.channel;
    }

    public ChannelFuture write(ChannelBuffer buffer) {
        Preconditions.checkNotNull(this.channel);
        Preconditions.checkNotNull(buffer);
        return this.channel.write(buffer);
    }
}
