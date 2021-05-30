package org.jboss.netty.util;

public final class ExternalResourceUtil {
    public static void release(ExternalResourceReleasable... releasables) {
        ExternalResourceReleasable[] releasablesCopy = new ExternalResourceReleasable[releasables.length];
        int i = 0;
        while (i < releasables.length) {
            if (releasables[i] != null) {
                releasablesCopy[i] = releasables[i];
                i++;
            } else {
                throw new NullPointerException("releasables[" + i + "]");
            }
        }
        for (ExternalResourceReleasable e : releasablesCopy) {
            e.releaseExternalResources();
        }
    }

    private ExternalResourceUtil() {
    }
}
