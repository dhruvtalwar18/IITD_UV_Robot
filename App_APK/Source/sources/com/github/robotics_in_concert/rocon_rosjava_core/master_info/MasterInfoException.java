package com.github.robotics_in_concert.rocon_rosjava_core.master_info;

public class MasterInfoException extends Exception {
    public MasterInfoException(Throwable throwable) {
        super(throwable);
    }

    public MasterInfoException(String message, Throwable throwable) {
        super(message, throwable);
    }

    public MasterInfoException(String message) {
        super(message);
    }
}
