package com.google.common.util.concurrent;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import com.google.common.util.concurrent.Service;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.ReentrantLock;

@Beta
public abstract class AbstractService implements Service {
    private final ReentrantLock lock = new ReentrantLock();
    private final Transition shutdown = new Transition();
    private boolean shutdownWhenStartupFinishes = false;
    private final Transition startup = new Transition();
    private Service.State state = Service.State.NEW;

    /* access modifiers changed from: protected */
    public abstract void doStart();

    /* access modifiers changed from: protected */
    public abstract void doStop();

    public final ListenableFuture<Service.State> start() {
        this.lock.lock();
        try {
            if (this.state == Service.State.NEW) {
                this.state = Service.State.STARTING;
                doStart();
            }
        } catch (Throwable th) {
            this.lock.unlock();
            throw th;
        }
        this.lock.unlock();
        return this.startup;
    }

    public final ListenableFuture<Service.State> stop() {
        this.lock.lock();
        try {
            if (this.state == Service.State.NEW) {
                this.state = Service.State.TERMINATED;
                this.startup.set(Service.State.TERMINATED);
                this.shutdown.set(Service.State.TERMINATED);
            } else if (this.state == Service.State.STARTING) {
                this.shutdownWhenStartupFinishes = true;
                this.startup.set(Service.State.STOPPING);
            } else if (this.state == Service.State.RUNNING) {
                this.state = Service.State.STOPPING;
                doStop();
            }
        } catch (Throwable th) {
            this.lock.unlock();
            throw th;
        }
        this.lock.unlock();
        return this.shutdown;
    }

    public Service.State startAndWait() {
        return (Service.State) Futures.getUnchecked(start());
    }

    public Service.State stopAndWait() {
        return (Service.State) Futures.getUnchecked(stop());
    }

    /* access modifiers changed from: protected */
    public final void notifyStarted() {
        this.lock.lock();
        try {
            if (this.state == Service.State.STARTING) {
                this.state = Service.State.RUNNING;
                if (this.shutdownWhenStartupFinishes) {
                    stop();
                } else {
                    this.startup.set(Service.State.RUNNING);
                }
                return;
            }
            IllegalStateException failure = new IllegalStateException("Cannot notifyStarted() when the service is " + this.state);
            notifyFailed(failure);
            throw failure;
        } finally {
            this.lock.unlock();
        }
    }

    /* access modifiers changed from: protected */
    public final void notifyStopped() {
        this.lock.lock();
        try {
            if (this.state != Service.State.STOPPING) {
                if (this.state != Service.State.RUNNING) {
                    IllegalStateException failure = new IllegalStateException("Cannot notifyStopped() when the service is " + this.state);
                    notifyFailed(failure);
                    throw failure;
                }
            }
            this.state = Service.State.TERMINATED;
            this.shutdown.set(Service.State.TERMINATED);
        } finally {
            this.lock.unlock();
        }
    }

    /* access modifiers changed from: protected */
    public final void notifyFailed(Throwable cause) {
        Preconditions.checkNotNull(cause);
        this.lock.lock();
        try {
            if (this.state == Service.State.STARTING) {
                this.startup.setException(cause);
                this.shutdown.setException(new Exception("Service failed to start.", cause));
            } else if (this.state == Service.State.STOPPING) {
                this.shutdown.setException(cause);
            } else if (this.state == Service.State.RUNNING) {
                this.shutdown.setException(new Exception("Service failed while running", cause));
            } else if (this.state == Service.State.NEW || this.state == Service.State.TERMINATED) {
                throw new IllegalStateException("Failed while in state:" + this.state, cause);
            }
            this.state = Service.State.FAILED;
        } finally {
            this.lock.unlock();
        }
    }

    public final boolean isRunning() {
        return state() == Service.State.RUNNING;
    }

    /* Debug info: failed to restart local var, previous not found, register: 2 */
    public final Service.State state() {
        Service.State state2;
        this.lock.lock();
        try {
            if (!this.shutdownWhenStartupFinishes || this.state != Service.State.STARTING) {
                state2 = this.state;
            } else {
                state2 = Service.State.STOPPING;
            }
            return state2;
        } finally {
            this.lock.unlock();
        }
    }

    public String toString() {
        return getClass().getSimpleName() + " [" + state() + "]";
    }

    private class Transition extends AbstractFuture<Service.State> {
        private Transition() {
        }

        public Service.State get(long timeout, TimeUnit unit) throws InterruptedException, TimeoutException, ExecutionException {
            try {
                return (Service.State) super.get(timeout, unit);
            } catch (TimeoutException e) {
                throw new TimeoutException(AbstractService.this.toString());
            }
        }
    }
}
