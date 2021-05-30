package org.ros.internal.transport.tcp;

import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.handler.codec.frame.LengthFieldBasedFrameDecoder;
import org.jboss.netty.handler.codec.frame.LengthFieldPrepender;

public class TcpClientPipelineFactory extends ConnectionTrackingChannelPipelineFactory {
    public static final String LENGTH_FIELD_BASED_FRAME_DECODER = "LengthFieldBasedFrameDecoder";
    public static final String LENGTH_FIELD_PREPENDER = "LengthFieldPrepender";

    public TcpClientPipelineFactory(ChannelGroup channelGroup) {
        super(channelGroup);
    }

    public ChannelPipeline getPipeline() {
        ChannelPipeline pipeline = super.getPipeline();
        pipeline.addLast("LengthFieldPrepender", new LengthFieldPrepender(4));
        pipeline.addLast("LengthFieldBasedFrameDecoder", new LengthFieldBasedFrameDecoder(Integer.MAX_VALUE, 0, 4, 0, 4));
        return pipeline;
    }
}
