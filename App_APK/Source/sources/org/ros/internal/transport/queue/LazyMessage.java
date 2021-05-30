package org.ros.internal.transport.queue;

import com.google.common.annotations.VisibleForTesting;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageDeserializer;

public class LazyMessage<T> {
    private final ChannelBuffer buffer;
    private final MessageDeserializer<T> deserializer;
    private T message;
    private final Object mutex;

    public LazyMessage(ChannelBuffer buffer2, MessageDeserializer<T> deserializer2) {
        this.buffer = buffer2;
        this.deserializer = deserializer2;
        this.mutex = new Object();
    }

    @VisibleForTesting
    LazyMessage(T message2) {
        this((ChannelBuffer) null, (MessageDeserializer) null);
        this.message = message2;
    }

    public T get() {
        synchronized (this.mutex) {
            if (this.message != null) {
                T t = this.message;
                return t;
            }
            this.message = this.deserializer.deserialize(this.buffer);
            return this.message;
        }
    }
}
