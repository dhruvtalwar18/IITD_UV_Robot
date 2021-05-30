package org.ros.internal.transport.tcp;

import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.handler.codec.frame.LengthFieldBasedFrameDecoder;
import org.jboss.netty.handler.codec.frame.LengthFieldPrepender;
import org.ros.internal.node.service.ServiceManager;
import org.ros.internal.node.topic.TopicParticipantManager;

public class TcpServerPipelineFactory extends ConnectionTrackingChannelPipelineFactory {
    public static final String HANDSHAKE_HANDLER = "HandshakeHandler";
    public static final String LENGTH_FIELD_BASED_FRAME_DECODER = "LengthFieldBasedFrameDecoder";
    public static final String LENGTH_FIELD_PREPENDER = "LengthFieldPrepender";
    private final ServiceManager serviceManager;
    private final TopicParticipantManager topicParticipantManager;

    public TcpServerPipelineFactory(ChannelGroup channelGroup, TopicParticipantManager topicParticipantManager2, ServiceManager serviceManager2) {
        super(channelGroup);
        this.topicParticipantManager = topicParticipantManager2;
        this.serviceManager = serviceManager2;
    }

    public ChannelPipeline getPipeline() {
        ChannelPipeline pipeline = super.getPipeline();
        pipeline.addLast("LengthFieldPrepender", new LengthFieldPrepender(4));
        pipeline.addLast("LengthFieldBasedFrameDecoder", new LengthFieldBasedFrameDecoder(Integer.MAX_VALUE, 0, 4, 0, 4));
        pipeline.addLast(HANDSHAKE_HANDLER, new TcpServerHandshakeHandler(this.topicParticipantManager, this.serviceManager));
        return pipeline;
    }
}
