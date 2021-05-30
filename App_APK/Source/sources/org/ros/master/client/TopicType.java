package org.ros.master.client;

public class TopicType {
    private final String messageType;
    private final String name;

    public TopicType(String name2, String messageType2) {
        this.name = name2;
        this.messageType = messageType2;
    }

    public String getName() {
        return this.name;
    }

    public String getMessageType() {
        return this.messageType;
    }
}
