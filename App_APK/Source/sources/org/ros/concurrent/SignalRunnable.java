package org.ros.concurrent;

public interface SignalRunnable<T> {
    void run(T t);
}
