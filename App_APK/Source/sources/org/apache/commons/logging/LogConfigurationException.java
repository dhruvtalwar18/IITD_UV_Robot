package org.apache.commons.logging;

public class LogConfigurationException extends RuntimeException {
    protected Throwable cause;

    public LogConfigurationException() {
        this.cause = null;
    }

    public LogConfigurationException(String message) {
        super(message);
        this.cause = null;
    }

    /* JADX INFO: this call moved to the top of the method (can break code semantics) */
    public LogConfigurationException(Throwable cause2) {
        this(cause2 == null ? null : cause2.toString(), cause2);
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public LogConfigurationException(java.lang.String r3, java.lang.Throwable r4) {
        /*
            r2 = this;
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            r0.append(r3)
            java.lang.String r1 = " (Caused by "
            r0.append(r1)
            r0.append(r4)
            java.lang.String r1 = ")"
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            r0 = 0
            r2.cause = r0
            r2.cause = r4
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.logging.LogConfigurationException.<init>(java.lang.String, java.lang.Throwable):void");
    }

    public Throwable getCause() {
        return this.cause;
    }
}
