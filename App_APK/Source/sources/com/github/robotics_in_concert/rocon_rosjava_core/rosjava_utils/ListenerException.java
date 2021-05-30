package com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils;

public class ListenerException extends Exception {
    public ListenerException(Throwable throwable) {
        super(throwable);
    }

    public ListenerException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public ListenerException(String message) {
        super(message);
    }
}
