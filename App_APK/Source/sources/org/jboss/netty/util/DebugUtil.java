package org.jboss.netty.util;

import org.jboss.netty.util.internal.SystemPropertyUtil;

public final class DebugUtil {
    public static boolean isDebugEnabled() {
        String value;
        try {
            value = SystemPropertyUtil.get("org.jboss.netty.debug");
        } catch (Exception e) {
            value = null;
        }
        if (value == null) {
            return false;
        }
        String value2 = value.trim().toUpperCase();
        if (value2.startsWith("N") || value2.startsWith("F") || value2.equals("0")) {
            return false;
        }
        return true;
    }

    private DebugUtil() {
    }
}
