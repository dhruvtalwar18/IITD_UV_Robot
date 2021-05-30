package org.apache.commons.httpclient;

import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;

public class HttpException extends IOException {
    static /* synthetic */ Class class$java$lang$Throwable;
    private final Throwable cause;
    private String reason;
    private int reasonCode;

    public HttpException() {
        this.reasonCode = 200;
        this.cause = null;
    }

    public HttpException(String message) {
        super(message);
        this.reasonCode = 200;
        this.cause = null;
    }

    public HttpException(String message, Throwable cause2) {
        super(message);
        Class cls;
        Class cls2;
        this.reasonCode = 200;
        this.cause = cause2;
        try {
            Class[] paramsClasses = new Class[1];
            if (class$java$lang$Throwable == null) {
                cls = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls;
            } else {
                cls = class$java$lang$Throwable;
            }
            paramsClasses[0] = cls;
            if (class$java$lang$Throwable == null) {
                cls2 = class$("java.lang.Throwable");
                class$java$lang$Throwable = cls2;
            } else {
                cls2 = class$java$lang$Throwable;
            }
            cls2.getMethod("initCause", paramsClasses).invoke(this, new Object[]{cause2});
        } catch (Exception e) {
        }
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public Throwable getCause() {
        return this.cause;
    }

    public void printStackTrace() {
        printStackTrace(System.err);
    }

    public void printStackTrace(PrintStream s) {
        try {
            getClass().getMethod("getStackTrace", new Class[0]);
            super.printStackTrace(s);
        } catch (Exception e) {
            super.printStackTrace(s);
            if (this.cause != null) {
                s.print("Caused by: ");
                this.cause.printStackTrace(s);
            }
        }
    }

    public void printStackTrace(PrintWriter s) {
        try {
            getClass().getMethod("getStackTrace", new Class[0]);
            super.printStackTrace(s);
        } catch (Exception e) {
            super.printStackTrace(s);
            if (this.cause != null) {
                s.print("Caused by: ");
                this.cause.printStackTrace(s);
            }
        }
    }

    public void setReason(String reason2) {
        this.reason = reason2;
    }

    public String getReason() {
        return this.reason;
    }

    public void setReasonCode(int code) {
        this.reasonCode = code;
    }

    public int getReasonCode() {
        return this.reasonCode;
    }
}
