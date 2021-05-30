package com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils;

public class BlockingServiceClientException extends Exception {
    public BlockingServiceClientException(Throwable throwable) {
        super(throwable);
    }

    public BlockingServiceClientException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public BlockingServiceClientException(String message) {
        super(message);
    }
}
