package org.ros.internal.transport.tcp;

import com.google.common.base.Preconditions;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelHandler;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.service.DefaultServiceServer;
import org.ros.internal.node.service.ServiceManager;
import org.ros.internal.node.service.ServiceResponseEncoder;
import org.ros.internal.node.topic.DefaultPublisher;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.internal.node.topic.TopicIdentifier;
import org.ros.internal.node.topic.TopicParticipantManager;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;

public class TcpServerHandshakeHandler extends SimpleChannelHandler {
    private final ServiceManager serviceManager;
    private final TopicParticipantManager topicParticipantManager;

    public TcpServerHandshakeHandler(TopicParticipantManager topicParticipantManager2, ServiceManager serviceManager2) {
        this.topicParticipantManager = topicParticipantManager2;
        this.serviceManager = serviceManager2;
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        ChannelPipeline pipeline = e.getChannel().getPipeline();
        ConnectionHeader incomingHeader = ConnectionHeader.decode((ChannelBuffer) e.getMessage());
        if (incomingHeader.hasField("service")) {
            handleServiceHandshake(e, pipeline, incomingHeader);
        } else {
            handleSubscriberHandshake(ctx, e, pipeline, incomingHeader);
        }
    }

    private void handleServiceHandshake(MessageEvent e, ChannelPipeline pipeline, ConnectionHeader incomingHeader) {
        GraphName serviceName = GraphName.of(incomingHeader.getField("service"));
        Preconditions.checkState(this.serviceManager.hasServer(serviceName));
        DefaultServiceServer<?, ?> serviceServer = this.serviceManager.getServer(serviceName);
        e.getChannel().write(serviceServer.finishHandshake(incomingHeader));
        String probe = incomingHeader.getField(ConnectionHeaderFields.PROBE);
        if (probe == null || !probe.equals("1")) {
            pipeline.replace("LengthFieldPrepender", "ServiceResponseEncoder", (ChannelHandler) new ServiceResponseEncoder());
            pipeline.replace((ChannelHandler) this, "ServiceRequestHandler", serviceServer.newRequestHandler());
            return;
        }
        e.getChannel().close();
    }

    private void handleSubscriberHandshake(ChannelHandlerContext ctx, MessageEvent e, ChannelPipeline pipeline, ConnectionHeader incomingConnectionHeader) throws InterruptedException, Exception {
        Preconditions.checkState(incomingConnectionHeader.hasField(ConnectionHeaderFields.TOPIC), "Handshake header missing field: topic");
        GraphName topicName = GraphName.of(incomingConnectionHeader.getField(ConnectionHeaderFields.TOPIC));
        boolean hasPublisher = this.topicParticipantManager.hasPublisher(topicName);
        Preconditions.checkState(hasPublisher, "No publisher for topic: " + topicName);
        DefaultPublisher<?> publisher = this.topicParticipantManager.getPublisher(topicName);
        ChannelBuffer outgoingBuffer = publisher.finishHandshake(incomingConnectionHeader);
        Channel channel = ctx.getChannel();
        ChannelFuture future = channel.write(outgoingBuffer).await();
        if (future.isSuccess()) {
            publisher.addSubscriber(new SubscriberIdentifier(NodeIdentifier.forName(incomingConnectionHeader.getField(ConnectionHeaderFields.CALLER_ID)), new TopicIdentifier(topicName)), channel);
            pipeline.replace((ChannelHandler) this, "DiscardHandler", (ChannelHandler) new SimpleChannelHandler());
            return;
        }
        throw new RosRuntimeException(future.getCause());
    }
}
