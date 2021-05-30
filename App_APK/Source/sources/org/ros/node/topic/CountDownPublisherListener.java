package org.ros.node.topic;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.ros.internal.node.CountDownRegistrantListener;
import org.ros.internal.node.topic.SubscriberIdentifier;

public class CountDownPublisherListener<T> extends CountDownRegistrantListener<Publisher<T>> implements PublisherListener<T> {
    private final CountDownLatch newSubscriberLatch;
    private final CountDownLatch shutdownLatch = new CountDownLatch(1);

    public static <T> CountDownPublisherListener<T> newDefault() {
        return newFromCounts(1, 1, 1, 1, 1);
    }

    public static <T> CountDownPublisherListener<T> newFromCounts(int masterRegistrationSuccessCount, int masterRegistrationFailureCount, int masterUnregistrationSuccessCount, int masterUnregistrationFailureCount, int newSubscriberCount) {
        return new CountDownPublisherListener(new CountDownLatch(masterRegistrationSuccessCount), new CountDownLatch(masterRegistrationFailureCount), new CountDownLatch(masterUnregistrationSuccessCount), new CountDownLatch(masterUnregistrationFailureCount), new CountDownLatch(newSubscriberCount));
    }

    private CountDownPublisherListener(CountDownLatch masterRegistrationSuccessLatch, CountDownLatch masterRegistrationFailureLatch, CountDownLatch masterUnregistrationSuccessLatch, CountDownLatch masterUnregistrationFailureLatch, CountDownLatch newSubscriberLatch2) {
        super(masterRegistrationSuccessLatch, masterRegistrationFailureLatch, masterUnregistrationSuccessLatch, masterUnregistrationFailureLatch);
        this.newSubscriberLatch = newSubscriberLatch2;
    }

    public void onNewSubscriber(Publisher<T> publisher, SubscriberIdentifier subscriberIdentifier) {
        this.newSubscriberLatch.countDown();
    }

    public void onShutdown(Publisher<T> publisher) {
        this.shutdownLatch.countDown();
    }

    public void awaitNewSubscriber() throws InterruptedException {
        this.newSubscriberLatch.await();
    }

    public boolean awaitNewSubscriber(long timeout, TimeUnit unit) throws InterruptedException {
        return this.newSubscriberLatch.await(timeout, unit);
    }

    public void awaitShutdown() throws InterruptedException {
        this.shutdownLatch.await();
    }

    public boolean awaitShutdown(long timeout, TimeUnit unit) throws InterruptedException {
        return this.shutdownLatch.await(timeout, unit);
    }
}
