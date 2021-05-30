package org.ros.internal.node.service;

import org.ros.internal.message.RawMessage;

public class Service {
    private final Request request;
    private final Response response;

    public interface Request extends RawMessage {
    }

    public interface Response extends RawMessage {
    }

    public Service(Request request2, Response response2) {
        this.request = request2;
        this.response = response2;
    }

    public <T extends Request> T getRequest() {
        return this.request;
    }

    public <T extends Response> T getResponse() {
        return this.response;
    }
}
