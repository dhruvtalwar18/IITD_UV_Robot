package org.ros.internal.node.service;

import org.jboss.netty.buffer.ChannelBuffer;

class ServiceServerResponse {
    private int errorCode;
    private ChannelBuffer message;
    private int messageLength;

    ServiceServerResponse() {
    }

    public void setErrorCode(int errorCode2) {
        this.errorCode = errorCode2;
    }

    public int getErrorCode() {
        return this.errorCode;
    }

    public void setMessage(ChannelBuffer buffer) {
        this.message = buffer;
    }

    public ChannelBuffer getMessage() {
        return this.message;
    }

    public void setMessageLength(int messageLength2) {
        this.messageLength = messageLength2;
    }

    public int getMessageLength() {
        return this.messageLength;
    }
}
