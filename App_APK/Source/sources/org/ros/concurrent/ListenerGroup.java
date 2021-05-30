package org.ros.concurrent;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.Collection;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public class ListenerGroup<T> {
    private static final int DEFAULT_QUEUE_CAPACITY = 128;
    private final Collection<EventDispatcher<T>> eventDispatchers = Lists.newCopyOnWriteArrayList();
    private final ExecutorService executorService;

    public ListenerGroup(ExecutorService executorService2) {
        this.executorService = executorService2;
    }

    public EventDispatcher<T> add(T listener, int queueCapacity) {
        EventDispatcher<T> eventDispatcher = new EventDispatcher<>(listener, queueCapacity);
        this.eventDispatchers.add(eventDispatcher);
        this.executorService.execute(eventDispatcher);
        return eventDispatcher;
    }

    public EventDispatcher<T> add(T listener) {
        return add(listener, 128);
    }

    public Collection<EventDispatcher<T>> addAll(Collection<T> listeners, int limit) {
        Collection<EventDispatcher<T>> eventDispatchers2 = Lists.newArrayList();
        for (T listener : listeners) {
            eventDispatchers2.add(add(listener, limit));
        }
        return eventDispatchers2;
    }

    public Collection<EventDispatcher<T>> addAll(Collection<T> listeners) {
        return addAll(listeners, 128);
    }

    public boolean remove(T listener) {
        Preconditions.checkNotNull(listener);
        for (EventDispatcher<T> eventDispatcher : this.eventDispatchers) {
            if (listener.equals(eventDispatcher.getListener())) {
                eventDispatcher.cancel();
                this.eventDispatchers.remove(eventDispatcher);
                return true;
            }
        }
        return false;
    }

    public int size() {
        return this.eventDispatchers.size();
    }

    public void signal(SignalRunnable<T> signalRunnable) {
        for (EventDispatcher<T> eventDispatcher : this.eventDispatchers) {
            eventDispatcher.signal(signalRunnable);
        }
    }

    public boolean signal(final SignalRunnable<T> signalRunnable, long timeout, TimeUnit unit) throws InterruptedException {
        Collection<EventDispatcher<T>> copy = Lists.newArrayList(this.eventDispatchers);
        final CountDownLatch latch = new CountDownLatch(copy.size());
        for (EventDispatcher<T> eventDispatcher : copy) {
            eventDispatcher.signal(new SignalRunnable<T>() {
                public void run(T listener) {
                    signalRunnable.run(listener);
                    latch.countDown();
                }
            });
        }
        return latch.await(timeout, unit);
    }

    public void shutdown() {
        for (EventDispatcher<T> eventDispatcher : this.eventDispatchers) {
            eventDispatcher.cancel();
        }
        this.eventDispatchers.clear();
    }
}
