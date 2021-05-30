package org.jboss.netty.util.internal;

import java.util.concurrent.Executor;

public class UnterminatableExecutor implements Executor {
    private final Executor executor;

    public UnterminatableExecutor(Executor executor2) {
        if (executor2 != null) {
            this.executor = executor2;
            return;
        }
        throw new NullPointerException("executor");
    }

    public void execute(Runnable command) {
        this.executor.execute(command);
    }
}
