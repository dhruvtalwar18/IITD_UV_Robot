package org.ros.message;

import org.jboss.netty.buffer.ChannelBuffer;

public interface MessageDeserializer<T> {
    T deserialize(ChannelBuffer channelBuffer);
}
