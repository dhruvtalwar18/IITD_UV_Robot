package org.jboss.netty.buffer;

import java.nio.ByteOrder;

public abstract class AbstractChannelBufferFactory implements ChannelBufferFactory {
    private final ByteOrder defaultOrder;

    protected AbstractChannelBufferFactory() {
        this(ByteOrder.BIG_ENDIAN);
    }

    protected AbstractChannelBufferFactory(ByteOrder defaultOrder2) {
        if (defaultOrder2 != null) {
            this.defaultOrder = defaultOrder2;
            return;
        }
        throw new NullPointerException("defaultOrder");
    }

    public ChannelBuffer getBuffer(int capacity) {
        return getBuffer(getDefaultOrder(), capacity);
    }

    public ChannelBuffer getBuffer(byte[] array, int offset, int length) {
        return getBuffer(getDefaultOrder(), array, offset, length);
    }

    public ByteOrder getDefaultOrder() {
        return this.defaultOrder;
    }
}
