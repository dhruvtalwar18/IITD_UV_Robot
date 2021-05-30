package org.apache.commons.lang;

public class NullArgumentException extends IllegalArgumentException {
    private static final long serialVersionUID = 1174360235354917591L;

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public NullArgumentException(java.lang.String r3) {
        /*
            r2 = this;
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            if (r3 != 0) goto L_0x000a
            java.lang.String r1 = "Argument"
            goto L_0x000b
        L_0x000a:
            r1 = r3
        L_0x000b:
            r0.append(r1)
            java.lang.String r1 = " must not be null."
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.NullArgumentException.<init>(java.lang.String):void");
    }
}
