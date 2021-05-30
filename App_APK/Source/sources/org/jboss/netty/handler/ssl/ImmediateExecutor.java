package org.jboss.netty.handler.ssl;

import java.util.concurrent.Executor;

final class ImmediateExecutor implements Executor {
    static final ImmediateExecutor INSTANCE = new ImmediateExecutor();

    ImmediateExecutor() {
    }

    public void execute(Runnable command) {
        command.run();
    }
}
