package org.ros.internal.node;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class CountDownRegistrantListener<T> implements RegistrantListener<T> {
    private final CountDownLatch masterRegistrationFailureLatch;
    private final CountDownLatch masterRegistrationSuccessLatch;
    private final CountDownLatch masterUnregistrationFailureLatch;
    private final CountDownLatch masterUnregistrationSuccessLatch;

    public CountDownRegistrantListener() {
        this(1, 1, 1, 1);
    }

    public CountDownRegistrantListener(int masterRegistrationSuccessCount, int masterRegistrationFailureCount, int masterUnregistrationSuccessCount, int masterUnregistrationFailureCount) {
        this(new CountDownLatch(masterRegistrationSuccessCount), new CountDownLatch(masterRegistrationFailureCount), new CountDownLatch(masterUnregistrationSuccessCount), new CountDownLatch(masterUnregistrationFailureCount));
    }

    public CountDownRegistrantListener(CountDownLatch masterRegistrationSuccessLatch2, CountDownLatch masterRegistrationFailureLatch2, CountDownLatch masterUnregistrationSuccessLatch2, CountDownLatch masterUnregistrationFailureLatch2) {
        this.masterRegistrationSuccessLatch = masterRegistrationSuccessLatch2;
        this.masterRegistrationFailureLatch = masterRegistrationFailureLatch2;
        this.masterUnregistrationSuccessLatch = masterUnregistrationSuccessLatch2;
        this.masterUnregistrationFailureLatch = masterUnregistrationFailureLatch2;
    }

    public void onMasterRegistrationSuccess(T t) {
        this.masterRegistrationSuccessLatch.countDown();
    }

    public void onMasterRegistrationFailure(T t) {
        this.masterRegistrationFailureLatch.countDown();
    }

    public void onMasterUnregistrationSuccess(T t) {
        this.masterUnregistrationSuccessLatch.countDown();
    }

    public void onMasterUnregistrationFailure(T t) {
        this.masterUnregistrationFailureLatch.countDown();
    }

    public void awaitMasterRegistrationSuccess() throws InterruptedException {
        this.masterRegistrationSuccessLatch.await();
    }

    public boolean awaitMasterRegistrationSuccess(long timeout, TimeUnit unit) throws InterruptedException {
        return this.masterRegistrationSuccessLatch.await(timeout, unit);
    }

    public void awaitMasterUnregistrationSuccess() throws InterruptedException {
        this.masterUnregistrationSuccessLatch.await();
    }

    public boolean awaitMasterUnregistrationSuccess(long timeout, TimeUnit unit) throws InterruptedException {
        return this.masterUnregistrationSuccessLatch.await(timeout, unit);
    }

    public void awaitMasterRegistrationFailure() throws InterruptedException {
        this.masterRegistrationFailureLatch.await();
    }

    public boolean awaitMasterRegistrationFailure(long timeout, TimeUnit unit) throws InterruptedException {
        return this.masterRegistrationFailureLatch.await(timeout, unit);
    }

    public void awaitMasterUnregistrationFailure() throws InterruptedException {
        this.masterUnregistrationFailureLatch.await();
    }

    public boolean awaitMasterUnregistrationFailure(long timeout, TimeUnit unit) throws InterruptedException {
        return this.masterUnregistrationFailureLatch.await(timeout, unit);
    }
}
