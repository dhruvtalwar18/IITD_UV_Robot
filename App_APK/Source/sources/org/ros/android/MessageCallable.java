package org.ros.android;

public interface MessageCallable<T, S> {
    T call(S s);
}
