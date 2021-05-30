package com.google.common.util.concurrent;

import com.google.common.annotations.Beta;
import com.google.common.base.Throwables;
import com.google.common.util.concurrent.Service;
import java.util.concurrent.Executor;

@Beta
public abstract class AbstractIdleService implements Service {
    private final Service delegate = new AbstractService() {
        /* access modifiers changed from: protected */
        public final void doStart() {
            AbstractIdleService.this.executor(Service.State.STARTING).execute(new Runnable() {
                public void run() {
                    try {
                        AbstractIdleService.this.startUp();
                        AnonymousClass1.this.notifyStarted();
                    } catch (Throwable t) {
                        AnonymousClass1.this.notifyFailed(t);
                        throw Throwables.propagate(t);
                    }
                }
            });
        }

        /* access modifiers changed from: protected */
        public final void doStop() {
            AbstractIdleService.this.executor(Service.State.STOPPING).execute(new Runnable() {
                public void run() {
                    try {
                        AbstractIdleService.this.shutDown();
                        AnonymousClass1.this.notifyStopped();
                    } catch (Throwable t) {
                        AnonymousClass1.this.notifyFailed(t);
                        throw Throwables.propagate(t);
                    }
                }
            });
        }
    };

    /* access modifiers changed from: protected */
    public abstract void shutDown() throws Exception;

    /* access modifiers changed from: protected */
    public abstract void startUp() throws Exception;

    /* access modifiers changed from: protected */
    public Executor executor(final Service.State state) {
        return new Executor() {
            public void execute(Runnable command) {
                new Thread(command, AbstractIdleService.this.getServiceName() + " " + state).start();
            }
        };
    }

    public String toString() {
        return getServiceName() + " [" + state() + "]";
    }

    public final ListenableFuture<Service.State> start() {
        return this.delegate.start();
    }

    public final Service.State startAndWait() {
        return this.delegate.startAndWait();
    }

    public final boolean isRunning() {
        return this.delegate.isRunning();
    }

    public final Service.State state() {
        return this.delegate.state();
    }

    public final ListenableFuture<Service.State> stop() {
        return this.delegate.stop();
    }

    public final Service.State stopAndWait() {
        return this.delegate.stopAndWait();
    }

    /* access modifiers changed from: private */
    public String getServiceName() {
        return getClass().getSimpleName();
    }
}
