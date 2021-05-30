package org.ros.internal.transport.tcp;

import com.google.common.collect.Lists;
import java.net.SocketAddress;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Executor;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.channel.group.DefaultChannelGroup;

public class TcpClientManager {
    private final ChannelGroup channelGroup = new DefaultChannelGroup();
    private final Executor executor;
    private final List<NamedChannelHandler> namedChannelHandlers = Lists.newArrayList();
    private final Collection<TcpClient> tcpClients = Lists.newArrayList();

    public TcpClientManager(Executor executor2) {
        this.executor = executor2;
    }

    public void addNamedChannelHandler(NamedChannelHandler namedChannelHandler) {
        this.namedChannelHandlers.add(namedChannelHandler);
    }

    public void addAllNamedChannelHandlers(List<NamedChannelHandler> namedChannelHandlers2) {
        this.namedChannelHandlers.addAll(namedChannelHandlers2);
    }

    public TcpClient connect(String connectionName, SocketAddress socketAddress) {
        TcpClient tcpClient = new TcpClient(this.channelGroup, this.executor);
        tcpClient.addAllNamedChannelHandlers(this.namedChannelHandlers);
        tcpClient.connect(connectionName, socketAddress);
        this.tcpClients.add(tcpClient);
        return tcpClient;
    }

    public void shutdown() {
        this.channelGroup.close().awaitUninterruptibly();
        this.tcpClients.clear();
    }
}
