package org.ros.concurrent;

import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class SharedScheduledExecutorService implements ScheduledExecutorService {
    private ScheduledExecutorService scheduledExecutorService;

    public SharedScheduledExecutorService(ScheduledExecutorService wrapped) {
        this.scheduledExecutorService = wrapped;
    }

    public boolean awaitTermination(long timeout, TimeUnit unit) throws InterruptedException {
        return this.scheduledExecutorService.awaitTermination(timeout, unit);
    }

    public void execute(Runnable command) {
        this.scheduledExecutorService.execute(command);
    }

    public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit) throws InterruptedException {
        return this.scheduledExecutorService.invokeAll(tasks, timeout, unit);
    }

    public <T> List<Future<T>> invokeAll(Collection<? extends Callable<T>> tasks) throws InterruptedException {
        return this.scheduledExecutorService.invokeAll(tasks);
    }

    public <T> T invokeAny(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit) throws InterruptedException, ExecutionException, TimeoutException {
        return this.scheduledExecutorService.invokeAny(tasks, timeout, unit);
    }

    public <T> T invokeAny(Collection<? extends Callable<T>> tasks) throws InterruptedException, ExecutionException {
        return this.scheduledExecutorService.invokeAny(tasks);
    }

    public boolean isShutdown() {
        return this.scheduledExecutorService.isShutdown();
    }

    public boolean isTerminated() {
        return this.scheduledExecutorService.isTerminated();
    }

    public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit) {
        return this.scheduledExecutorService.schedule(callable, delay, unit);
    }

    public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit) {
        return this.scheduledExecutorService.schedule(command, delay, unit);
    }

    public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period, TimeUnit unit) {
        return this.scheduledExecutorService.scheduleAtFixedRate(command, initialDelay, period, unit);
    }

    public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay, TimeUnit unit) {
        return this.scheduledExecutorService.scheduleWithFixedDelay(command, initialDelay, delay, unit);
    }

    public void shutdown() {
        throw new UnsupportedOperationException("Cannot shut down service");
    }

    public List<Runnable> shutdownNow() {
        throw new UnsupportedOperationException("Cannot shut down service");
    }

    public <T> Future<T> submit(Callable<T> task) {
        return this.scheduledExecutorService.submit(task);
    }

    public <T> Future<T> submit(Runnable task, T result) {
        return this.scheduledExecutorService.submit(task, result);
    }

    public Future<?> submit(Runnable task) {
        return this.scheduledExecutorService.submit(task);
    }
}
