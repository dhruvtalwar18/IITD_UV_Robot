package org.apache.commons.httpclient;

public class InvalidRedirectLocationException extends RedirectException {
    private final String location;

    public InvalidRedirectLocationException(String message, String location2) {
        super(message);
        this.location = location2;
    }

    public InvalidRedirectLocationException(String message, String location2, Throwable cause) {
        super(message, cause);
        this.location = location2;
    }

    public String getLocation() {
        return this.location;
    }
}
