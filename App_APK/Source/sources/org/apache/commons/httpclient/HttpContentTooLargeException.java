package org.apache.commons.httpclient;

public class HttpContentTooLargeException extends HttpException {
    private int maxlen;

    public HttpContentTooLargeException(String message, int maxlen2) {
        super(message);
        this.maxlen = maxlen2;
    }

    public int getMaxLength() {
        return this.maxlen;
    }
}
