package org.ros.concurrent;

import com.google.common.collect.Maps;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletionService;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorCompletionService;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.exception.RosRuntimeException;

public class RetryingExecutorService {
    private static final boolean DEBUG = false;
    private static final long DEFAULT_RETRY_DELAY = 5;
    private static final TimeUnit DEFAULT_RETRY_TIME_UNIT = TimeUnit.SECONDS;
    private static final Log log = LogFactory.getLog(RetryingExecutorService.class);
    /* access modifiers changed from: private */
    public final Map<Future<Boolean>, Callable<Boolean>> callables = Maps.newConcurrentMap();
    /* access modifiers changed from: private */
    public final CompletionService<Boolean> completionService;
    /* access modifiers changed from: private */
    public final Map<Callable<Boolean>, CountDownLatch> latches = Maps.newConcurrentMap();
    /* access modifiers changed from: private */
    public final Object mutex;
    /* access modifiers changed from: private */
    public long retryDelay;
    private final RetryLoop retryLoop = new RetryLoop();
    /* access modifiers changed from: private */
    public TimeUnit retryTimeUnit;
    private boolean running;
    /* access modifiers changed from: private */
    public final ScheduledExecutorService scheduledExecutorService;

    private class RetryLoop extends CancellableLoop {
        private RetryLoop() {
        }

        public void loop() throws InterruptedException {
            Callable callable;
            CountDownLatch latch;
            Future take = RetryingExecutorService.this.completionService.take();
            synchronized (RetryingExecutorService.this.mutex) {
                callable = (Callable) RetryingExecutorService.this.callables.remove(take);
                latch = (CountDownLatch) RetryingExecutorService.this.latches.get(callable);
            }
            try {
                if (((Boolean) take.get()).booleanValue()) {
                    final Callable callable2 = callable;
                    RetryingExecutorService.this.scheduledExecutorService.schedule(new Runnable() {
                        public void run() {
                            RetryingExecutorService.this.submit(callable2);
                        }
                    }, RetryingExecutorService.this.retryDelay, RetryingExecutorService.this.retryTimeUnit);
                    return;
                }
                latch.countDown();
            } catch (ExecutionException e) {
                throw new RosRuntimeException(e.getCause());
            }
        }
    }

    public RetryingExecutorService(ScheduledExecutorService scheduledExecutorService2) {
        this.scheduledExecutorService = scheduledExecutorService2;
        this.completionService = new ExecutorCompletionService(scheduledExecutorService2);
        this.mutex = new Object();
        this.retryDelay = 5;
        this.retryTimeUnit = DEFAULT_RETRY_TIME_UNIT;
        this.running = true;
        scheduledExecutorService2.execute(this.retryLoop);
    }

    public void submit(Callable<Boolean> callable) {
        synchronized (this.mutex) {
            if (this.running) {
                Future<Boolean> future = this.completionService.submit(callable);
                this.latches.put(callable, new CountDownLatch(1));
                this.callables.put(future, callable);
            } else {
                throw new RejectedExecutionException();
            }
        }
    }

    public void setRetryDelay(long delay, TimeUnit unit) {
        this.retryDelay = delay;
        this.retryTimeUnit = unit;
    }

    public void shutdown(long timeout, TimeUnit unit) throws InterruptedException {
        this.running = false;
        for (CountDownLatch latch : this.latches.values()) {
            latch.await(timeout, unit);
        }
        this.retryLoop.cancel();
    }
}
