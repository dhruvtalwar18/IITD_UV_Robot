package org.ros.time;

import org.ros.Topics;
import org.ros.internal.node.DefaultNode;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.node.topic.Subscriber;
import rosgraph_msgs.Clock;

public class ClockTopicTimeProvider implements TimeProvider {
    /* access modifiers changed from: private */
    public Clock clock;
    /* access modifiers changed from: private */
    public Object mutex = new Object();
    private final Subscriber<Clock> subscriber;

    public ClockTopicTimeProvider(DefaultNode defaultNode) {
        this.subscriber = defaultNode.newSubscriber(Topics.CLOCK, Clock._TYPE);
        this.subscriber.addMessageListener(new MessageListener<Clock>() {
            public void onNewMessage(Clock message) {
                synchronized (ClockTopicTimeProvider.this.mutex) {
                    Clock unused = ClockTopicTimeProvider.this.clock = message;
                }
            }
        });
    }

    public Subscriber<Clock> getSubscriber() {
        return this.subscriber;
    }

    public Time getCurrentTime() {
        Time time;
        if (this.clock == null) {
            return new Time();
        }
        synchronized (this.mutex) {
            time = new Time(this.clock.getClock());
        }
        return time;
    }
}
