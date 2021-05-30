package org.ros.master.client;

import java.util.Collection;

public class SystemState {
    private final Collection<TopicSystemState> topics;

    public SystemState(Collection<TopicSystemState> topics2) {
        this.topics = topics2;
    }

    public Collection<TopicSystemState> getTopics() {
        return this.topics;
    }
}
