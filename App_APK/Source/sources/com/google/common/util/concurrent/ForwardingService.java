package com.google.common.util.concurrent;

import com.google.common.annotations.Beta;
import com.google.common.collect.ForwardingObject;
import com.google.common.util.concurrent.Service;

@Beta
public abstract class ForwardingService extends ForwardingObject implements Service {
    /* access modifiers changed from: protected */
    public abstract Service delegate();

    protected ForwardingService() {
    }

    public ListenableFuture<Service.State> start() {
        return delegate().start();
    }

    public Service.State state() {
        return delegate().state();
    }

    public ListenableFuture<Service.State> stop() {
        return delegate().stop();
    }

    public Service.State startAndWait() {
        return delegate().startAndWait();
    }

    public Service.State stopAndWait() {
        return delegate().stopAndWait();
    }

    public boolean isRunning() {
        return delegate().isRunning();
    }

    /* access modifiers changed from: protected */
    public Service.State standardStartAndWait() {
        return (Service.State) Futures.getUnchecked(start());
    }

    /* access modifiers changed from: protected */
    public Service.State standardStopAndWait() {
        return (Service.State) Futures.getUnchecked(stop());
    }
}
