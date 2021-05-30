package org.apache.commons.httpclient.protocol;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;

public final class ReflectionSocketFactory {
    private static Constructor INETSOCKETADDRESS_CONSTRUCTOR = null;
    private static boolean REFLECTION_FAILED = false;
    private static Method SOCKETBIND_METHOD = null;
    private static Method SOCKETCONNECT_METHOD = null;
    private static Class SOCKETTIMEOUTEXCEPTION_CLASS = null;
    static /* synthetic */ Class class$java$net$InetAddress;
    static /* synthetic */ Class class$java$net$Socket;

    private ReflectionSocketFactory() {
    }

    /* JADX WARNING: Removed duplicated region for block: B:48:0x0106  */
    /* JADX WARNING: Removed duplicated region for block: B:56:0x011d  */
    /* JADX WARNING: Removed duplicated region for block: B:61:0x0126  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.net.Socket createSocket(java.lang.String r16, java.lang.String r17, int r18, java.net.InetAddress r19, int r20, int r21) throws java.io.IOException, java.net.UnknownHostException, org.apache.commons.httpclient.ConnectTimeoutException {
        /*
            r1 = r21
            boolean r0 = REFLECTION_FAILED
            r2 = 0
            if (r0 == 0) goto L_0x0008
            return r2
        L_0x0008:
            r3 = 1
            java.lang.Class r0 = java.lang.Class.forName(r16)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.String r4 = "getDefault"
            r5 = 0
            java.lang.Class[] r6 = new java.lang.Class[r5]     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.reflect.Method r4 = r0.getMethod(r4, r6)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Object[] r6 = new java.lang.Object[r5]     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Object r6 = r4.invoke(r2, r6)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.String r7 = "createSocket"
            java.lang.Class[] r8 = new java.lang.Class[r5]     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.reflect.Method r7 = r0.getMethod(r7, r8)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            r4 = r7
            java.lang.Object[] r7 = new java.lang.Object[r5]     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Object r7 = r4.invoke(r6, r7)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.net.Socket r7 = (java.net.Socket) r7     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.reflect.Constructor r8 = INETSOCKETADDRESS_CONSTRUCTOR     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            r9 = 2
            if (r8 != 0) goto L_0x0055
            java.lang.String r8 = "java.net.InetSocketAddress"
            java.lang.Class r8 = java.lang.Class.forName(r8)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Class[] r10 = new java.lang.Class[r9]     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Class r11 = class$java$net$InetAddress     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            if (r11 != 0) goto L_0x0047
            java.lang.String r11 = "java.net.InetAddress"
            java.lang.Class r11 = class$(r11)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            class$java$net$InetAddress = r11     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            goto L_0x0049
        L_0x0047:
            java.lang.Class r11 = class$java$net$InetAddress     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
        L_0x0049:
            r10[r5] = r11     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Class r11 = java.lang.Integer.TYPE     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            r10[r3] = r11     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.reflect.Constructor r10 = r8.getConstructor(r10)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            INETSOCKETADDRESS_CONSTRUCTOR = r10     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
        L_0x0055:
            java.lang.reflect.Constructor r8 = INETSOCKETADDRESS_CONSTRUCTOR     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Object[] r10 = new java.lang.Object[r9]     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.net.InetAddress r11 = java.net.InetAddress.getByName(r17)     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            r10[r5] = r11     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            java.lang.Integer r11 = new java.lang.Integer     // Catch:{ InvocationTargetException -> 0x00f8, Exception -> 0x00ef }
            r12 = r18
            r11.<init>(r12)     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            r10[r3] = r11     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            java.lang.Object r8 = r8.newInstance(r10)     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            java.lang.reflect.Constructor r10 = INETSOCKETADDRESS_CONSTRUCTOR     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            java.lang.Object[] r11 = new java.lang.Object[r9]     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            r11[r5] = r19     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            java.lang.Integer r13 = new java.lang.Integer     // Catch:{ InvocationTargetException -> 0x00ed, Exception -> 0x00eb }
            r14 = r20
            r13.<init>(r14)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r11[r3] = r13     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.Object r10 = r10.newInstance(r11)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.reflect.Method r11 = SOCKETCONNECT_METHOD     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            if (r11 != 0) goto L_0x00a8
            java.lang.Class r11 = class$java$net$Socket     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            if (r11 != 0) goto L_0x0090
            java.lang.String r11 = "java.net.Socket"
            java.lang.Class r11 = class$(r11)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            class$java$net$Socket = r11     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            goto L_0x0092
        L_0x0090:
            java.lang.Class r11 = class$java$net$Socket     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
        L_0x0092:
            java.lang.String r13 = "connect"
            java.lang.Class[] r2 = new java.lang.Class[r9]     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.String r15 = "java.net.SocketAddress"
            java.lang.Class r15 = java.lang.Class.forName(r15)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r2[r5] = r15     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.Class r15 = java.lang.Integer.TYPE     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r2[r3] = r15     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.reflect.Method r2 = r11.getMethod(r13, r2)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            SOCKETCONNECT_METHOD = r2     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
        L_0x00a8:
            java.lang.reflect.Method r2 = SOCKETBIND_METHOD     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            if (r2 != 0) goto L_0x00cd
            java.lang.Class r2 = class$java$net$Socket     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            if (r2 != 0) goto L_0x00b9
            java.lang.String r2 = "java.net.Socket"
            java.lang.Class r2 = class$(r2)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            class$java$net$Socket = r2     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            goto L_0x00bb
        L_0x00b9:
            java.lang.Class r2 = class$java$net$Socket     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
        L_0x00bb:
            java.lang.String r11 = "bind"
            java.lang.Class[] r13 = new java.lang.Class[r3]     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.String r15 = "java.net.SocketAddress"
            java.lang.Class r15 = java.lang.Class.forName(r15)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r13[r5] = r15     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.reflect.Method r2 = r2.getMethod(r11, r13)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            SOCKETBIND_METHOD = r2     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
        L_0x00cd:
            java.lang.reflect.Method r2 = SOCKETBIND_METHOD     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.Object[] r11 = new java.lang.Object[r3]     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r11[r5] = r10     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r2.invoke(r7, r11)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.reflect.Method r2 = SOCKETCONNECT_METHOD     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.Object[] r9 = new java.lang.Object[r9]     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r9[r5] = r8     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            java.lang.Integer r5 = new java.lang.Integer     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r5.<init>(r1)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r9[r3] = r5     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            r2.invoke(r7, r9)     // Catch:{ InvocationTargetException -> 0x00e9, Exception -> 0x00e7 }
            return r7
        L_0x00e7:
            r0 = move-exception
            goto L_0x00f4
        L_0x00e9:
            r0 = move-exception
            goto L_0x00fd
        L_0x00eb:
            r0 = move-exception
            goto L_0x00f2
        L_0x00ed:
            r0 = move-exception
            goto L_0x00fb
        L_0x00ef:
            r0 = move-exception
            r12 = r18
        L_0x00f2:
            r14 = r20
        L_0x00f4:
            REFLECTION_FAILED = r3
            r2 = 0
            return r2
        L_0x00f8:
            r0 = move-exception
            r12 = r18
        L_0x00fb:
            r14 = r20
        L_0x00fd:
            r2 = r0
            java.lang.Throwable r4 = r2.getTargetException()
            java.lang.Class r0 = SOCKETTIMEOUTEXCEPTION_CLASS
            if (r0 != 0) goto L_0x0114
            java.lang.String r0 = "java.net.SocketTimeoutException"
            java.lang.Class r0 = java.lang.Class.forName(r0)     // Catch:{ ClassNotFoundException -> 0x010f }
            SOCKETTIMEOUTEXCEPTION_CLASS = r0     // Catch:{ ClassNotFoundException -> 0x010f }
            goto L_0x0114
        L_0x010f:
            r0 = move-exception
            REFLECTION_FAILED = r3
            r3 = 0
            return r3
        L_0x0114:
            r3 = 0
            java.lang.Class r0 = SOCKETTIMEOUTEXCEPTION_CLASS
            boolean r0 = r0.isInstance(r4)
            if (r0 != 0) goto L_0x0126
            boolean r0 = r4 instanceof java.io.IOException
            if (r0 != 0) goto L_0x0122
            return r3
        L_0x0122:
            r0 = r4
            java.io.IOException r0 = (java.io.IOException) r0
            throw r0
        L_0x0126:
            org.apache.commons.httpclient.ConnectTimeoutException r0 = new org.apache.commons.httpclient.ConnectTimeoutException
            java.lang.StringBuffer r3 = new java.lang.StringBuffer
            r3.<init>()
            java.lang.String r5 = "The host did not accept the connection within timeout of "
            r3.append(r5)
            r3.append(r1)
            java.lang.String r5 = " ms"
            r3.append(r5)
            java.lang.String r3 = r3.toString()
            r0.<init>(r3, r4)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.protocol.ReflectionSocketFactory.createSocket(java.lang.String, java.lang.String, int, java.net.InetAddress, int, int):java.net.Socket");
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }
}
