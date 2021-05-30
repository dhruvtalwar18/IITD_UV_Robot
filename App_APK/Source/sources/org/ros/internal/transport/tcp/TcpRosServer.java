package org.ros.internal.transport.tcp;

import com.google.common.base.Preconditions;
import java.net.InetSocketAddress;
import java.nio.ByteOrder;
import java.util.concurrent.Callable;
import java.util.concurrent.Executor;
import java.util.concurrent.ScheduledExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.bootstrap.ServerBootstrap;
import org.jboss.netty.buffer.HeapChannelBufferFactory;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.channel.group.DefaultChannelGroup;
import org.jboss.netty.channel.socket.nio.NioServerSocketChannelFactory;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.service.ServiceManager;
import org.ros.internal.node.topic.TopicParticipantManager;

public class TcpRosServer {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(TcpRosServer.class);
    private final AdvertiseAddress advertiseAddress;
    private final BindAddress bindAddress;
    private ServerBootstrap bootstrap;
    private ChannelFactory channelFactory;
    private final ScheduledExecutorService executorService;
    private ChannelGroup incomingChannelGroup;
    /* access modifiers changed from: private */
    public Channel outgoingChannel;
    private final ServiceManager serviceManager;
    private final TopicParticipantManager topicParticipantManager;

    public TcpRosServer(BindAddress bindAddress2, AdvertiseAddress advertiseAddress2, TopicParticipantManager topicParticipantManager2, ServiceManager serviceManager2, ScheduledExecutorService executorService2) {
        this.bindAddress = bindAddress2;
        this.advertiseAddress = advertiseAddress2;
        this.topicParticipantManager = topicParticipantManager2;
        this.serviceManager = serviceManager2;
        this.executorService = executorService2;
    }

    public void start() {
        Preconditions.checkState(this.outgoingChannel == null);
        this.channelFactory = new NioServerSocketChannelFactory((Executor) this.executorService, (Executor) this.executorService);
        this.bootstrap = new ServerBootstrap(this.channelFactory);
        this.bootstrap.setOption("child.bufferFactory", new HeapChannelBufferFactory(ByteOrder.LITTLE_ENDIAN));
        this.bootstrap.setOption("child.keepAlive", true);
        this.incomingChannelGroup = new DefaultChannelGroup();
        this.bootstrap.setPipelineFactory(new TcpServerPipelineFactory(this.incomingChannelGroup, this.topicParticipantManager, this.serviceManager));
        this.outgoingChannel = this.bootstrap.bind(this.bindAddress.toInetSocketAddress());
        this.advertiseAddress.setPortCallable(new Callable<Integer>() {
            public Integer call() throws Exception {
                return Integer.valueOf(((InetSocketAddress) TcpRosServer.this.outgoingChannel.getLocalAddress()).getPort());
            }
        });
    }

    public void shutdown() {
        if (this.outgoingChannel != null) {
            this.outgoingChannel.close().awaitUninterruptibly();
        }
        this.incomingChannelGroup.close().awaitUninterruptibly();
        this.outgoingChannel = null;
    }

    public InetSocketAddress getAddress() {
        return this.advertiseAddress.toInetSocketAddress();
    }

    public AdvertiseAddress getAdvertiseAddress() {
        return this.advertiseAddress;
    }
}
