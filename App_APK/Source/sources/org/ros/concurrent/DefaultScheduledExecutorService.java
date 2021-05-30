package org.ros.concurrent;

import com.google.common.collect.Lists;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class DefaultScheduledExecutorService implements ScheduledExecutorService {
    private static final int CORE_POOL_SIZE = 11;
    private final ExecutorService executorService;
    private final ScheduledExecutorService scheduledExecutorService;

    public DefaultScheduledExecutorService() {
        this(Executors.newCachedThreadPool());
    }

    public DefaultScheduledExecutorService(ExecutorService executorService2) {
        this(executorService2, Executors.newScheduledThreadPool(11));
    }

    public DefaultScheduledExecutorService(ExecutorService executorService2, ScheduledExecutorService scheduledExecutorService2) {
        this.executorService = executorService2;
        this.scheduledExecutorService = scheduledExecutorService2;
    }

    public void shutdown() {
        this.executorService.shutdown();
        this.scheduledExecutorService.shutdown();
    }

    public List<Runnable> shutdownNow() {
        List<Runnable> combined = Lists.newArrayList();
        combined.addAll(this.executorService.shutdownNow());
        combined.addAll(this.scheduledExecutorService.shutdownNow());
        return combined;
    }

    public boolean isShutdown() {
        return this.executorService.isShutdown() && this.scheduledExecutorService.isShutdown();
    }

    public boolean isTerminated() {
        return this.executorService.isTerminated() && this.scheduledExecutorService.isTerminated();
    }

    public boolean awaitTermination(long timeout, TimeUnit unit) throws InterruptedException {
        return this.executorService.awaitTermination(timeout, unit) && this.scheduledExecutorService.awaitTermination(timeout, unit);
    }

    public <T> Future<T> submit(Callable<T> task) {
        return this.executorService.submit(task);
    }

    public <T> Future<T> submit(Runnable task, T result) {
        return this.executorService.submit(task, result);
    }

    public Future<?> submit(Runnable task) {
        return this.executorService.submit(task);
    }

    public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks) throws InterruptedException {
        return this.executorService.invokeAll(tasks);
    }

    public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit) throws InterruptedException {
        return this.executorService.invokeAll(tasks, timeout, unit);
    }

    public <T> T invokeAny(Collection<? extends Callable<T>> tasks) throws InterruptedException, ExecutionException {
        return this.executorService.invokeAny(tasks);
    }

    public <T> T invokeAny(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit) throws InterruptedException, ExecutionException, TimeoutException {
        return this.executorService.invokeAny(tasks, timeout, unit);
    }

    public void execute(Runnable command) {
        this.executorService.execute(command);
    }

    public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit) {
        return this.scheduledExecutorService.schedule(command, delay, unit);
    }

    public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit) {
        return this.scheduledExecutorService.schedule(callable, delay, unit);
    }

    public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period, TimeUnit unit) {
        return this.scheduledExecutorService.scheduleAtFixedRate(command, initialDelay, period, unit);
    }

    public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay, TimeUnit unit) {
        return this.scheduledExecutorService.scheduleWithFixedDelay(command, initialDelay, delay, unit);
    }
}
