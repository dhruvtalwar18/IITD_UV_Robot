package com.google.common.util.concurrent;

import com.google.common.annotations.Beta;

@Beta
public interface Service {

    @Beta
    public enum State {
        NEW,
        STARTING,
        RUNNING,
        STOPPING,
        TERMINATED,
        FAILED
    }

    boolean isRunning();

    ListenableFuture<State> start();

    State startAndWait();

    State state();

    ListenableFuture<State> stop();

    State stopAndWait();
}
