package org.apache.commons.lang;

public class IllegalClassException extends IllegalArgumentException {
    private static final long serialVersionUID = 8063272569377254819L;

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public IllegalClassException(java.lang.Class r3, java.lang.Object r4) {
        /*
            r2 = this;
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            java.lang.String r1 = "Expected: "
            r0.append(r1)
            java.lang.String r1 = safeGetClassName(r3)
            r0.append(r1)
            java.lang.String r1 = ", actual: "
            r0.append(r1)
            if (r4 != 0) goto L_0x001b
            java.lang.String r1 = "null"
            goto L_0x0023
        L_0x001b:
            java.lang.Class r1 = r4.getClass()
            java.lang.String r1 = r1.getName()
        L_0x0023:
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.IllegalClassException.<init>(java.lang.Class, java.lang.Object):void");
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public IllegalClassException(java.lang.Class r3, java.lang.Class r4) {
        /*
            r2 = this;
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            java.lang.String r1 = "Expected: "
            r0.append(r1)
            java.lang.String r1 = safeGetClassName(r3)
            r0.append(r1)
            java.lang.String r1 = ", actual: "
            r0.append(r1)
            java.lang.String r1 = safeGetClassName(r4)
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.IllegalClassException.<init>(java.lang.Class, java.lang.Class):void");
    }

    public IllegalClassException(String message) {
        super(message);
    }

    private static final String safeGetClassName(Class cls) {
        if (cls == null) {
            return null;
        }
        return cls.getName();
    }
}
