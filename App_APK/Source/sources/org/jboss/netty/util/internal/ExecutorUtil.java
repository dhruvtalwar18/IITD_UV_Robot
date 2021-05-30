package org.jboss.netty.util.internal;

import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public final class ExecutorUtil {
    public static boolean isShutdown(Executor executor) {
        if (!(executor instanceof ExecutorService) || !((ExecutorService) executor).isShutdown()) {
            return false;
        }
        return true;
    }

    public static void terminate(Executor... executors) {
        terminate(DeadLockProofWorker.PARENT, executors);
    }

    public static void terminate(ThreadLocal<Executor> deadLockChecker, Executor... executors) {
        if (executors != null) {
            Executor[] executorsCopy = new Executor[executors.length];
            int i = 0;
            while (i < executors.length) {
                if (executors[i] != null) {
                    executorsCopy[i] = executors[i];
                    i++;
                } else {
                    throw new NullPointerException("executors[" + i + "]");
                }
            }
            Executor currentParent = deadLockChecker.get();
            if (currentParent != null) {
                Executor[] arr$ = executorsCopy;
                int len$ = arr$.length;
                int i$ = 0;
                while (i$ < len$) {
                    if (arr$[i$] != currentParent) {
                        i$++;
                    } else {
                        throw new IllegalStateException("An Executor cannot be shut down from the thread acquired from itself.  Please make sure you are not calling releaseExternalResources() from an I/O worker thread.");
                    }
                }
            }
            boolean interrupted = false;
            for (Executor e : executorsCopy) {
                if (e instanceof ExecutorService) {
                    ExecutorService es = (ExecutorService) e;
                    while (true) {
                        try {
                            es.shutdownNow();
                        } catch (SecurityException e2) {
                            try {
                                es.shutdown();
                            } catch (SecurityException e3) {
                            } catch (NullPointerException e4) {
                            }
                        } catch (NullPointerException e5) {
                        }
                        try {
                            if (es.awaitTermination(100, TimeUnit.MILLISECONDS)) {
                                break;
                            }
                        } catch (InterruptedException e6) {
                            interrupted = true;
                        }
                    }
                }
            }
            if (interrupted) {
                Thread.currentThread().interrupt();
                return;
            }
            return;
        }
        throw new NullPointerException("executors");
    }

    private ExecutorUtil() {
    }
}
