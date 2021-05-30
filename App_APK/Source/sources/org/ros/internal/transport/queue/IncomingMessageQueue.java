package org.ros.internal.transport.queue;

import java.util.concurrent.ExecutorService;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.internal.transport.tcp.NamedChannelHandler;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageListener;

public class IncomingMessageQueue<T> {
    private static final int DEQUE_CAPACITY = 16;
    private final MessageDispatcher<T> messageDispatcher;
    private final MessageReceiver<T> messageReceiver;

    public IncomingMessageQueue(MessageDeserializer<T> deserializer, ExecutorService executorService) {
        CircularBlockingDeque<LazyMessage<T>> lazyMessages = new CircularBlockingDeque<>(16);
        this.messageReceiver = new MessageReceiver<>(lazyMessages, deserializer);
        this.messageDispatcher = new MessageDispatcher<>(lazyMessages, executorService);
        executorService.execute(this.messageDispatcher);
    }

    public void setLatchMode(boolean enabled) {
        this.messageDispatcher.setLatchMode(enabled);
    }

    public boolean getLatchMode() {
        return this.messageDispatcher.getLatchMode();
    }

    public void addListener(MessageListener<T> messageListener, int queueCapacity) {
        this.messageDispatcher.addListener(messageListener, queueCapacity);
    }

    public boolean removeListener(MessageListener<T> messageListener) {
        return this.messageDispatcher.removeListener(messageListener);
    }

    public void removeAllListeners() {
        this.messageDispatcher.removeAllListeners();
    }

    public void shutdown() {
        this.messageDispatcher.cancel();
    }

    public NamedChannelHandler getMessageReceiver() {
        return this.messageReceiver;
    }
}
