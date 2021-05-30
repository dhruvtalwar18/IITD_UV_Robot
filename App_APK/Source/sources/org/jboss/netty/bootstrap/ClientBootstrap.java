package org.jboss.netty.bootstrap;

import java.net.SocketAddress;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelPipelineException;

public class ClientBootstrap extends Bootstrap {
    public ClientBootstrap() {
    }

    public ClientBootstrap(ChannelFactory channelFactory) {
        super(channelFactory);
    }

    public ChannelFuture connect() {
        SocketAddress remoteAddress = (SocketAddress) getOption("remoteAddress");
        if (remoteAddress != null) {
            return connect(remoteAddress);
        }
        throw new IllegalStateException("remoteAddress option is not set.");
    }

    public ChannelFuture connect(SocketAddress remoteAddress) {
        if (remoteAddress != null) {
            return connect(remoteAddress, (SocketAddress) getOption("localAddress"));
        }
        throw new NullPointerException("remoteAddress");
    }

    public ChannelFuture connect(SocketAddress remoteAddress, SocketAddress localAddress) {
        if (remoteAddress != null) {
            try {
                Channel ch = getFactory().newChannel(getPipelineFactory().getPipeline());
                try {
                    ch.getConfig().setOptions(getOptions());
                    if (1 == 0) {
                        ch.close();
                    }
                    if (localAddress != null) {
                        ch.bind(localAddress);
                    }
                    return ch.connect(remoteAddress);
                } catch (Throwable th) {
                    if (0 == 0) {
                        ch.close();
                    }
                    throw th;
                }
            } catch (Exception e) {
                throw new ChannelPipelineException("Failed to initialize a pipeline.", e);
            }
        } else {
            throw new NullPointerException("remoteAddress");
        }
    }

    public ChannelFuture bind(SocketAddress localAddress) {
        if (localAddress != null) {
            try {
                Channel ch = getFactory().newChannel(getPipelineFactory().getPipeline());
                try {
                    ch.getConfig().setOptions(getOptions());
                    if (1 == 0) {
                        ch.close();
                    }
                    return ch.bind(localAddress);
                } catch (Throwable th) {
                    if (0 == 0) {
                        ch.close();
                    }
                    throw th;
                }
            } catch (Exception e) {
                throw new ChannelPipelineException("Failed to initialize a pipeline.", e);
            }
        } else {
            throw new NullPointerException("localAddress");
        }
    }
}
