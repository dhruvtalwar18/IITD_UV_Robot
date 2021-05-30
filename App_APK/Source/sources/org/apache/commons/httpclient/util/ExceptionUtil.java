package org.apache.commons.httpclient.util;

import java.io.InterruptedIOException;
import java.lang.reflect.Method;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class ExceptionUtil {
    private static final Method INIT_CAUSE_METHOD = getInitCauseMethod();
    private static final Log LOG;
    private static final Class SOCKET_TIMEOUT_CLASS = SocketTimeoutExceptionClass();
    static /* synthetic */ Class class$java$lang$Throwable;
    static /* synthetic */ Class class$org$apache$commons$httpclient$util$ExceptionUtil;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$util$ExceptionUtil == null) {
            cls = class$("org.apache.commons.httpclient.util.ExceptionUtil");
            class$org$apache$commons$httpclient$util$ExceptionUtil = cls;
        } else {
            cls = class$org$apache$commons$httpclient$util$ExceptionUtil;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    private static Method getInitCauseMethod() {
        Class cls;
        Class cls2;
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
            return cls2.getMethod("initCause", paramsClasses);
        } catch (NoSuchMethodException e) {
            return null;
        }
    }

    private static Class SocketTimeoutExceptionClass() {
        try {
            return Class.forName("java.net.SocketTimeoutException");
        } catch (ClassNotFoundException e) {
            return null;
        }
    }

    public static void initCause(Throwable throwable, Throwable cause) {
        if (INIT_CAUSE_METHOD != null) {
            try {
                INIT_CAUSE_METHOD.invoke(throwable, new Object[]{cause});
            } catch (Exception e) {
                LOG.warn("Exception invoking Throwable.initCause", e);
            }
        }
    }

    public static boolean isSocketTimeoutException(InterruptedIOException e) {
        if (SOCKET_TIMEOUT_CLASS != null) {
            return SOCKET_TIMEOUT_CLASS.isInstance(e);
        }
        return true;
    }
}
