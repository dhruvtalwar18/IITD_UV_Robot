package com.google.common.eventbus;

import com.google.common.annotations.Beta;
import com.google.common.eventbus.EventBus;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executor;

@Beta
public class AsyncEventBus extends EventBus {
    private final ConcurrentLinkedQueue<EventBus.EventWithHandler> eventsToDispatch = new ConcurrentLinkedQueue<>();
    private final Executor executor;

    public AsyncEventBus(String identifier, Executor executor2) {
        super(identifier);
        this.executor = executor2;
    }

    public AsyncEventBus(Executor executor2) {
        this.executor = executor2;
    }

    /* access modifiers changed from: protected */
    public void enqueueEvent(Object event, EventHandler handler) {
        this.eventsToDispatch.offer(new EventBus.EventWithHandler(event, handler));
    }

    /* access modifiers changed from: protected */
    public void dispatchQueuedEvents() {
        while (true) {
            EventBus.EventWithHandler eventWithHandler = this.eventsToDispatch.poll();
            if (eventWithHandler != null) {
                dispatch(eventWithHandler.event, eventWithHandler.handler);
            } else {
                return;
            }
        }
    }

    /* access modifiers changed from: protected */
    public void dispatch(final Object event, final EventHandler handler) {
        this.executor.execute(new Runnable() {
            public void run() {
                AsyncEventBus.super.dispatch(event, handler);
            }
        });
    }
}
