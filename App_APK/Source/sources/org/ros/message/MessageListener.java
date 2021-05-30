package org.ros.message;

public interface MessageListener<T> {
    void onNewMessage(T t);
}
