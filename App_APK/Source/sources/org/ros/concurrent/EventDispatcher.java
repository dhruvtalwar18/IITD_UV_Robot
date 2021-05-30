package org.ros.concurrent;

public class EventDispatcher<T> extends CancellableLoop {
    private final CircularBlockingDeque<SignalRunnable<T>> events;
    private final T listener;

    public EventDispatcher(T listener2, int queueCapacity) {
        this.listener = listener2;
        this.events = new CircularBlockingDeque<>(queueCapacity);
    }

    public void signal(SignalRunnable<T> signalRunnable) {
        this.events.addLast(signalRunnable);
    }

    public void loop() throws InterruptedException {
        this.events.takeFirst().run(this.listener);
    }

    public T getListener() {
        return this.listener;
    }
}
