package com.google.common.util.concurrent;

import com.google.common.base.Preconditions;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.AbstractQueuedSynchronizer;
import javax.annotation.Nullable;

public abstract class AbstractFuture<V> implements ListenableFuture<V> {
    private final ExecutionList executionList = new ExecutionList();
    private final Sync<V> sync = new Sync<>();

    public V get(long timeout, TimeUnit unit) throws InterruptedException, TimeoutException, ExecutionException {
        return this.sync.get(unit.toNanos(timeout));
    }

    public V get() throws InterruptedException, ExecutionException {
        return this.sync.get();
    }

    public boolean isDone() {
        return this.sync.isDone();
    }

    public boolean isCancelled() {
        return this.sync.isCancelled();
    }

    public boolean cancel(boolean mayInterruptIfRunning) {
        if (!this.sync.cancel()) {
            return false;
        }
        this.executionList.execute();
        if (!mayInterruptIfRunning) {
            return true;
        }
        interruptTask();
        return true;
    }

    /* access modifiers changed from: protected */
    public void interruptTask() {
    }

    public void addListener(Runnable listener, Executor exec) {
        this.executionList.add(listener, exec);
    }

    /* access modifiers changed from: protected */
    public boolean set(@Nullable V value) {
        boolean result = this.sync.set(value);
        if (result) {
            this.executionList.execute();
        }
        return result;
    }

    /* access modifiers changed from: protected */
    public boolean setException(Throwable throwable) {
        boolean result = this.sync.setException((Throwable) Preconditions.checkNotNull(throwable));
        if (result) {
            this.executionList.execute();
        }
        if (!(throwable instanceof Error)) {
            return result;
        }
        throw ((Error) throwable);
    }

    static final class Sync<V> extends AbstractQueuedSynchronizer {
        static final int CANCELLED = 4;
        static final int COMPLETED = 2;
        static final int COMPLETING = 1;
        static final int RUNNING = 0;
        private static final long serialVersionUID = 0;
        private Throwable exception;
        private V value;

        Sync() {
        }

        /* access modifiers changed from: protected */
        public int tryAcquireShared(int ignored) {
            if (isDone()) {
                return 1;
            }
            return -1;
        }

        /* access modifiers changed from: protected */
        public boolean tryReleaseShared(int finalState) {
            setState(finalState);
            return true;
        }

        /* access modifiers changed from: package-private */
        public V get(long nanos) throws TimeoutException, CancellationException, ExecutionException, InterruptedException {
            if (tryAcquireSharedNanos(-1, nanos)) {
                return getValue();
            }
            throw new TimeoutException("Timeout waiting for task.");
        }

        /* access modifiers changed from: package-private */
        public V get() throws CancellationException, ExecutionException, InterruptedException {
            acquireSharedInterruptibly(-1);
            return getValue();
        }

        private V getValue() throws CancellationException, ExecutionException {
            int state = getState();
            if (state != 2) {
                if (state != 4) {
                    throw new IllegalStateException("Error, synchronizer in invalid state: " + state);
                }
                throw new CancellationException("Task was cancelled.");
            } else if (this.exception == null) {
                return this.value;
            } else {
                throw new ExecutionException(this.exception);
            }
        }

        /* access modifiers changed from: package-private */
        public boolean isDone() {
            return (getState() & 6) != 0;
        }

        /* access modifiers changed from: package-private */
        public boolean isCancelled() {
            return getState() == 4;
        }

        /* access modifiers changed from: package-private */
        public boolean set(@Nullable V v) {
            return complete(v, (Throwable) null, 2);
        }

        /* access modifiers changed from: package-private */
        public boolean setException(Throwable t) {
            return complete((Object) null, t, 2);
        }

        /* access modifiers changed from: package-private */
        public boolean cancel() {
            return complete((Object) null, (Throwable) null, 4);
        }

        private boolean complete(@Nullable V v, @Nullable Throwable t, int finalState) {
            boolean doCompletion = compareAndSetState(0, 1);
            if (doCompletion) {
                this.value = v;
                this.exception = t;
                releaseShared(finalState);
            } else if (getState() == 1) {
                acquireShared(-1);
            }
            return doCompletion;
        }
    }
}
