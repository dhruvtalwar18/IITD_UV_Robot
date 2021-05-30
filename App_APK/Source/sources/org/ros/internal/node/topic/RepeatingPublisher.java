package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import java.util.concurrent.ScheduledExecutorService;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.topic.Publisher;

public class RepeatingPublisher<MessageType> {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(RepeatingPublisher.class);
    private final ScheduledExecutorService executorService;
    /* access modifiers changed from: private */
    public final int frequency;
    /* access modifiers changed from: private */
    public final MessageType message;
    /* access modifiers changed from: private */
    public final Publisher<MessageType> publisher;
    private final RepeatingPublisher<MessageType>.RepeatingPublisherLoop runnable = new RepeatingPublisherLoop();

    private final class RepeatingPublisherLoop extends CancellableLoop {
        private RepeatingPublisherLoop() {
        }

        public void loop() throws InterruptedException {
            RepeatingPublisher.this.publisher.publish(RepeatingPublisher.this.message);
            double access$200 = (double) RepeatingPublisher.this.frequency;
            Double.isNaN(access$200);
            Thread.sleep((long) (1000.0d / access$200));
        }
    }

    public RepeatingPublisher(Publisher<MessageType> publisher2, MessageType message2, int frequency2, ScheduledExecutorService executorService2) {
        this.publisher = publisher2;
        this.message = message2;
        this.frequency = frequency2;
        this.executorService = executorService2;
    }

    public void start() {
        Preconditions.checkState(!this.runnable.isRunning());
        this.executorService.execute(this.runnable);
    }

    public void cancel() {
        Preconditions.checkState(this.runnable.isRunning());
        this.runnable.cancel();
    }
}
