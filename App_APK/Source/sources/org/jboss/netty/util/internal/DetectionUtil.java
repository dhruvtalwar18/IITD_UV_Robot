package org.jboss.netty.util.internal;

import java.security.AccessController;
import java.security.PrivilegedActionException;
import java.security.PrivilegedExceptionAction;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.zip.Deflater;

public final class DetectionUtil {
    private static final boolean HAS_UNSAFE = hasUnsafe(AtomicInteger.class.getClassLoader());
    private static final boolean IS_WINDOWS = (System.getProperty("os.name").toLowerCase().indexOf("win") >= 0);
    private static final int JAVA_VERSION = javaVersion0();

    public static boolean isWindows() {
        return IS_WINDOWS;
    }

    public static boolean hasUnsafe() {
        return HAS_UNSAFE;
    }

    public static int javaVersion() {
        return JAVA_VERSION;
    }

    private static boolean hasUnsafe(ClassLoader loader) {
        if (!Boolean.valueOf(SystemPropertyUtil.get("org.jboss.netty.tryUnsafe", "true")).booleanValue()) {
            return false;
        }
        try {
            return hasUnsafeField(Class.forName("sun.misc.Unsafe", true, loader));
        } catch (Exception e) {
            return false;
        }
    }

    private static boolean hasUnsafeField(final Class<?> unsafeClass) throws PrivilegedActionException {
        return ((Boolean) AccessController.doPrivileged(new PrivilegedExceptionAction<Boolean>() {
            public Boolean run() throws Exception {
                unsafeClass.getDeclaredField("theUnsafe");
                return true;
            }
        })).booleanValue();
    }

    private static int javaVersion0() {
        try {
            Class.forName("android.app.Application");
            return 6;
        } catch (ClassNotFoundException e) {
            try {
                Deflater.class.getDeclaredField("SYNC_FLUSH");
                return 7;
            } catch (Exception e2) {
                try {
                    Double.class.getDeclaredField("MIN_NORMAL");
                    return 6;
                } catch (Exception e3) {
                    return 5;
                }
            }
        }
    }

    private DetectionUtil() {
    }
}
