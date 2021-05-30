package org.ros.internal.system;

import java.lang.management.ManagementFactory;

public class Process {
    private Process() {
    }

    public static int getPid() {
        try {
            String mxName = ManagementFactory.getRuntimeMXBean().getName();
            int idx = mxName.indexOf(64);
            if (idx > 0) {
                try {
                    return Integer.parseInt(mxName.substring(0, idx));
                } catch (NumberFormatException e) {
                    return 0;
                }
            }
            throw new UnsupportedOperationException();
        } catch (NoClassDefFoundError e2) {
            try {
                return ((Integer) Class.forName("android.os.Process").getMethod("myPid", new Class[0]).invoke((Object) null, new Object[0])).intValue();
            } catch (Exception e3) {
            }
        }
    }
}
