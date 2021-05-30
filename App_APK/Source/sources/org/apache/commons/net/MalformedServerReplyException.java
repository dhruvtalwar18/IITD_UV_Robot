package org.apache.commons.net;

import java.io.IOException;

public class MalformedServerReplyException extends IOException {
    public MalformedServerReplyException() {
    }

    public MalformedServerReplyException(String message) {
        super(message);
    }
}
