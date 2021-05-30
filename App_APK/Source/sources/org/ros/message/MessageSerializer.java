package org.ros.message;

import org.jboss.netty.buffer.ChannelBuffer;

public interface MessageSerializer<T> {
    void serialize(T t, ChannelBuffer channelBuffer);
}
