package org.ros.internal.message;

import java.nio.ByteOrder;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

public class MessageBuffers {
    static final int ESTIMATED_LENGTH = 256;

    private MessageBuffers() {
    }

    public static ChannelBuffer dynamicBuffer() {
        return ChannelBuffers.dynamicBuffer(ByteOrder.LITTLE_ENDIAN, 256);
    }
}
