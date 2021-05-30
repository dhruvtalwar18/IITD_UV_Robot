package org.ros.node.topic;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.ros.internal.node.CountDownRegistrantListener;
import org.ros.internal.node.topic.PublisherIdentifier;

public class CountDownSubscriberListener<T> extends CountDownRegistrantListener<Subscriber<T>> implements SubscriberListener<T> {
    private final CountDownLatch newPublisherLatch;
    private final CountDownLatch shutdownLatch = new CountDownLatch(1);

    public static <T> CountDownSubscriberListener<T> newDefault() {
        return newFromCounts(1, 1, 1, 1, 1);
    }

    public static <T> CountDownSubscriberListener<T> newFromCounts(int masterRegistrationSuccessCount, int masterRegistrationFailureCount, int masterUnregistrationSuccessCount, int masterUnregistrationFailureCount, int newSubscriberCount) {
        return new CountDownSubscriberListener(new CountDownLatch(masterRegistrationSuccessCount), new CountDownLatch(masterRegistrationFailureCount), new CountDownLatch(masterUnregistrationSuccessCount), new CountDownLatch(masterUnregistrationFailureCount), new CountDownLatch(newSubscriberCount));
    }

    private CountDownSubscriberListener(CountDownLatch masterRegistrationSuccessLatch, CountDownLatch masterRegistrationFailureLatch, CountDownLatch masterUnregistrationSuccessLatch, CountDownLatch masterUnregistrationFailureLatch, CountDownLatch newPublisherLatch2) {
        super(masterRegistrationSuccessLatch, masterRegistrationFailureLatch, masterUnregistrationSuccessLatch, masterUnregistrationFailureLatch);
        this.newPublisherLatch = newPublisherLatch2;
    }

    public void onNewPublisher(Subscriber<T> subscriber, PublisherIdentifier publisherIdentifier) {
        this.newPublisherLatch.countDown();
    }

    public void onShutdown(Subscriber<T> subscriber) {
        this.shutdownLatch.countDown();
    }

    public void awaitNewPublisher() throws InterruptedException {
        this.newPublisherLatch.await();
    }

    public boolean awaitNewPublisher(long timeout, TimeUnit unit) throws InterruptedException {
        return this.newPublisherLatch.await(timeout, unit);
    }

    public void awaitShutdown() throws InterruptedException {
        this.shutdownLatch.await();
    }

    public boolean awaitShutdown(long timeout, TimeUnit unit) throws InterruptedException {
        return this.shutdownLatch.await(timeout, unit);
    }
}
