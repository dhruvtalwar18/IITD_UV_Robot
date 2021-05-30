package com.google.common.util.concurrent;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.logging.Level;
import java.util.logging.Logger;

public final class ExecutionList {
    /* access modifiers changed from: private */
    public static final Logger log = Logger.getLogger(ExecutionList.class.getName());
    private boolean executed = false;
    private final Queue<RunnableExecutorPair> runnables = Lists.newLinkedList();

    public void add(Runnable runnable, Executor executor) {
        Preconditions.checkNotNull(runnable, "Runnable was null.");
        Preconditions.checkNotNull(executor, "Executor was null.");
        boolean executeImmediate = false;
        synchronized (this.runnables) {
            if (!this.executed) {
                this.runnables.add(new RunnableExecutorPair(runnable, executor));
            } else {
                executeImmediate = true;
            }
        }
        if (executeImmediate) {
            new RunnableExecutorPair(runnable, executor).execute();
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:10:0x0013, code lost:
        if (r2.runnables.isEmpty() != false) goto L_0x0021;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:11:0x0015, code lost:
        r2.runnables.poll().execute();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:12:0x0021, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void execute() {
        /*
            r2 = this;
            java.util.Queue<com.google.common.util.concurrent.ExecutionList$RunnableExecutorPair> r0 = r2.runnables
            monitor-enter(r0)
            boolean r1 = r2.executed     // Catch:{ all -> 0x0022 }
            if (r1 == 0) goto L_0x0009
            monitor-exit(r0)     // Catch:{ all -> 0x0022 }
            return
        L_0x0009:
            r1 = 1
            r2.executed = r1     // Catch:{ all -> 0x0022 }
            monitor-exit(r0)     // Catch:{ all -> 0x0022 }
        L_0x000d:
            java.util.Queue<com.google.common.util.concurrent.ExecutionList$RunnableExecutorPair> r0 = r2.runnables
            boolean r0 = r0.isEmpty()
            if (r0 != 0) goto L_0x0021
            java.util.Queue<com.google.common.util.concurrent.ExecutionList$RunnableExecutorPair> r0 = r2.runnables
            java.lang.Object r0 = r0.poll()
            com.google.common.util.concurrent.ExecutionList$RunnableExecutorPair r0 = (com.google.common.util.concurrent.ExecutionList.RunnableExecutorPair) r0
            r0.execute()
            goto L_0x000d
        L_0x0021:
            return
        L_0x0022:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x0022 }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.util.concurrent.ExecutionList.execute():void");
    }

    private static class RunnableExecutorPair {
        final Executor executor;
        final Runnable runnable;

        RunnableExecutorPair(Runnable runnable2, Executor executor2) {
            this.runnable = runnable2;
            this.executor = executor2;
        }

        /* access modifiers changed from: package-private */
        public void execute() {
            try {
                this.executor.execute(this.runnable);
            } catch (RuntimeException e) {
                Logger access$000 = ExecutionList.log;
                Level level = Level.SEVERE;
                access$000.log(level, "RuntimeException while executing runnable " + this.runnable + " with executor " + this.executor, e);
            }
        }
    }
}
