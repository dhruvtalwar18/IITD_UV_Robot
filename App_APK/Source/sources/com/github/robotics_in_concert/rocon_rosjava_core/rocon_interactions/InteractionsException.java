package com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions;

public class InteractionsException extends Exception {
    public InteractionsException(Throwable throwable) {
        super(throwable);
    }

    public InteractionsException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public InteractionsException(String message) {
        super(message);
    }
}
