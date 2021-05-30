package org.jboss.netty.util.internal;

import java.util.concurrent.Executor;

public final class DeadLockProofWorker {
    public static final ThreadLocal<Executor> PARENT = new ThreadLocal<>();

    public static void start(final Executor parent, final Runnable runnable) {
        if (parent == null) {
            throw new NullPointerException("parent");
        } else if (runnable != null) {
            parent.execute(new Runnable() {
                public void run() {
                    DeadLockProofWorker.PARENT.set(parent);
                    try {
                        runnable.run();
                    } finally {
                        DeadLockProofWorker.PARENT.remove();
                    }
                }
            });
        } else {
            throw new NullPointerException("runnable");
        }
    }

    private DeadLockProofWorker() {
    }
}
