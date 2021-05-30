package org.apache.commons.lang;

import java.util.Arrays;

public class IncompleteArgumentException extends IllegalArgumentException {
    private static final long serialVersionUID = 4954193403612068178L;

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public IncompleteArgumentException(java.lang.String r3) {
        /*
            r2 = this;
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            r0.append(r3)
            java.lang.String r1 = " is incomplete."
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.IncompleteArgumentException.<init>(java.lang.String):void");
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public IncompleteArgumentException(java.lang.String r3, java.lang.String[] r4) {
        /*
            r2 = this;
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            r0.append(r3)
            java.lang.String r1 = " is missing the following items: "
            r0.append(r1)
            java.lang.String r1 = safeArrayToString(r4)
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.IncompleteArgumentException.<init>(java.lang.String, java.lang.String[]):void");
    }

    private static final String safeArrayToString(Object[] array) {
        if (array == null) {
            return null;
        }
        return Arrays.asList(array).toString();
    }
}
