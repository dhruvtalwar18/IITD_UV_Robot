package org.apache.commons.httpclient;

public class URIException extends HttpException {
    public static final int ESCAPING = 3;
    public static final int PARSING = 1;
    public static final int PUNYCODE = 4;
    public static final int UNKNOWN = 0;
    public static final int UNSUPPORTED_ENCODING = 2;
    protected String reason;
    protected int reasonCode;

    public URIException() {
    }

    public URIException(int reasonCode2) {
        this.reasonCode = reasonCode2;
    }

    public URIException(int reasonCode2, String reason2) {
        super(reason2);
        this.reason = reason2;
        this.reasonCode = reasonCode2;
    }

    public URIException(String reason2) {
        super(reason2);
        this.reason = reason2;
        this.reasonCode = 0;
    }

    public int getReasonCode() {
        return this.reasonCode;
    }

    public void setReasonCode(int reasonCode2) {
        this.reasonCode = reasonCode2;
    }

    public String getReason() {
        return this.reason;
    }

    public void setReason(String reason2) {
        this.reason = reason2;
    }
}
