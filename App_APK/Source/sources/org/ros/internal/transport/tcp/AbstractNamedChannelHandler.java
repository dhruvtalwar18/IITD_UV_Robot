package org.ros.internal.transport.tcp;

import org.jboss.netty.channel.SimpleChannelHandler;

public abstract class AbstractNamedChannelHandler extends SimpleChannelHandler implements NamedChannelHandler {
    public String toString() {
        return String.format("NamedChannelHandler<%s, %s>", new Object[]{getName(), super.toString()});
    }
}
