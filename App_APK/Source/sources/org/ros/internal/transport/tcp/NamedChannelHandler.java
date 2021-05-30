package org.ros.internal.transport.tcp;

import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelUpstreamHandler;

public interface NamedChannelHandler extends ChannelUpstreamHandler, ChannelDownstreamHandler {
    String getName();
}
