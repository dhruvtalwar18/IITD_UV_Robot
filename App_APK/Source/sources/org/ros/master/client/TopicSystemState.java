package org.ros.master.client;

import java.util.Set;

public class TopicSystemState {
    private final Set<String> publishers;
    private final Set<String> subscribers;
    private final String topicName;

    public TopicSystemState(String topicName2, Set<String> publishers2, Set<String> subscribers2) {
        this.topicName = topicName2;
        this.publishers = publishers2;
        this.subscribers = subscribers2;
    }

    public String getTopicName() {
        return this.topicName;
    }

    public Set<String> getPublishers() {
        return this.publishers;
    }

    public Set<String> getSubscribers() {
        return this.subscribers;
    }
}
