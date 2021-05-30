package org.jboss.netty.channel.local;

import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;

public class DefaultLocalClientChannelFactory implements LocalClientChannelFactory {
    private final ChannelSink sink = new LocalClientChannelSink();

    public LocalChannel newChannel(ChannelPipeline pipeline) {
        return new DefaultLocalChannel((LocalServerChannel) null, this, pipeline, this.sink, (DefaultLocalChannel) null);
    }

    public void releaseExternalResources() {
    }
}
