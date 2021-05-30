package org.ros.internal.transport.tcp;

import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelPipelineFactory;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.group.ChannelGroup;
import org.ros.internal.transport.ConnectionTrackingHandler;

public class ConnectionTrackingChannelPipelineFactory implements ChannelPipelineFactory {
    public static final String CONNECTION_TRACKING_HANDLER = "ConnectionTrackingHandler";
    private final ConnectionTrackingHandler connectionTrackingHandler;

    public ConnectionTrackingChannelPipelineFactory(ChannelGroup channelGroup) {
        this.connectionTrackingHandler = new ConnectionTrackingHandler(channelGroup);
    }

    public ChannelPipeline getPipeline() {
        ChannelPipeline pipeline = Channels.pipeline();
        pipeline.addLast(CONNECTION_TRACKING_HANDLER, this.connectionTrackingHandler);
        return pipeline;
    }
}
