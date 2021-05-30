package org.ros.node.service;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.ros.internal.node.CountDownRegistrantListener;

public class CountDownServiceServerListener<T, S> extends CountDownRegistrantListener<ServiceServer<T, S>> implements ServiceServerListener<T, S> {
    private final CountDownLatch shutdownLatch = new CountDownLatch(1);

    public static <T, S> CountDownServiceServerListener<T, S> newDefault() {
        return newFromCounts(1, 1, 1, 1);
    }

    public static <T, S> CountDownServiceServerListener<T, S> newFromCounts(int masterRegistrationSuccessCount, int masterRegistrationFailureCount, int masterUnregistrationSuccessCount, int masterUnregistrationFailureCount) {
        return new CountDownServiceServerListener<>(new CountDownLatch(masterRegistrationSuccessCount), new CountDownLatch(masterRegistrationFailureCount), new CountDownLatch(masterUnregistrationSuccessCount), new CountDownLatch(masterUnregistrationFailureCount));
    }

    private CountDownServiceServerListener(CountDownLatch masterRegistrationSuccessLatch, CountDownLatch masterRegistrationFailureLatch, CountDownLatch masterUnregistrationSuccessLatch, CountDownLatch masterUnregistrationFailureLatch) {
        super(masterRegistrationSuccessLatch, masterRegistrationFailureLatch, masterUnregistrationSuccessLatch, masterUnregistrationFailureLatch);
    }

    public void onShutdown(ServiceServer<T, S> serviceServer) {
        this.shutdownLatch.countDown();
    }

    public void awaitShutdown() throws InterruptedException {
        this.shutdownLatch.await();
    }

    public boolean awaitShutdown(long timeout, TimeUnit unit) throws InterruptedException {
        return this.shutdownLatch.await(timeout, unit);
    }
}
