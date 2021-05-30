package org.apache.commons.net.telnet;

public class InvalidTelnetOptionException extends Exception {
    private String msg;
    private int optionCode = -1;

    public InvalidTelnetOptionException(String message, int optcode) {
        this.optionCode = optcode;
        this.msg = message;
    }

    public String getMessage() {
        return this.msg + ": " + this.optionCode;
    }
}
