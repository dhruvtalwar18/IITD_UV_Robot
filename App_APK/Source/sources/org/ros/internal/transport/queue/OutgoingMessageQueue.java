package org.ros.internal.transport.queue;

import com.google.common.annotations.VisibleForTesting;
import java.util.concurrent.ExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.channel.group.ChannelGroupFuture;
import org.jboss.netty.channel.group.ChannelGroupFutureListener;
import org.jboss.netty.channel.group.DefaultChannelGroup;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.internal.message.MessageBufferPool;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.MessageSerializer;

public class OutgoingMessageQueue<T> {
    private static final boolean DEBUG = false;
    private static final int DEQUE_CAPACITY = 16;
    private static final Log log = LogFactory.getLog(OutgoingMessageQueue.class);
    /* access modifiers changed from: private */
    public final ChannelGroup channelGroup = new DefaultChannelGroup();
    /* access modifiers changed from: private */
    public final CircularBlockingDeque<T> deque = new CircularBlockingDeque<>(16);
    private boolean latchMode = false;
    private final ChannelBuffer latchedBuffer = MessageBuffers.dynamicBuffer();
    private T latchedMessage;
    /* access modifiers changed from: private */
    public final MessageBufferPool messageBufferPool = new MessageBufferPool();
    private final Object mutex = new Object();
    /* access modifiers changed from: private */
    public final MessageSerializer<T> serializer;
    private final OutgoingMessageQueue<T>.Writer writer = new Writer();

    private final class Writer extends CancellableLoop {
        private Writer() {
        }

        public void loop() throws InterruptedException {
            T message = OutgoingMessageQueue.this.deque.takeFirst();
            final ChannelBuffer buffer = OutgoingMessageQueue.this.messageBufferPool.acquire();
            OutgoingMessageQueue.this.serializer.serialize(message, buffer);
            OutgoingMessageQueue.this.channelGroup.write(buffer).addListener(new ChannelGroupFutureListener() {
                public void operationComplete(ChannelGroupFuture future) throws Exception {
                    OutgoingMessageQueue.this.messageBufferPool.release(buffer);
                }
            });
        }
    }

    public OutgoingMessageQueue(MessageSerializer<T> serializer2, ExecutorService executorService) {
        this.serializer = serializer2;
        executorService.execute(this.writer);
    }

    public void setLatchMode(boolean enabled) {
        this.latchMode = enabled;
    }

    public boolean getLatchMode() {
        return this.latchMode;
    }

    public void add(T message) {
        this.deque.addLast(message);
        setLatchedMessage(message);
    }

    private void setLatchedMessage(T message) {
        synchronized (this.mutex) {
            this.latchedMessage = message;
        }
    }

    public void shutdown() {
        this.writer.cancel();
        this.channelGroup.close().awaitUninterruptibly();
    }

    public void addChannel(Channel channel) {
        if (!this.writer.isRunning()) {
            log.warn("Failed to add channel. Cannot add channels after shutdown.");
            return;
        }
        if (this.latchMode && this.latchedMessage != null) {
            writeLatchedMessage(channel);
        }
        this.channelGroup.add(channel);
    }

    private void writeLatchedMessage(Channel channel) {
        synchronized (this.mutex) {
            this.latchedBuffer.clear();
            this.serializer.serialize(this.latchedMessage, this.latchedBuffer);
            channel.write(this.latchedBuffer);
        }
    }

    public int getNumberOfChannels() {
        return this.channelGroup.size();
    }

    @VisibleForTesting
    public ChannelGroup getChannelGroup() {
        return this.channelGroup;
    }
}
