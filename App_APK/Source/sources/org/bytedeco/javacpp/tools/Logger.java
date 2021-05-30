package org.bytedeco.javacpp.tools;

import java.io.PrintStream;
import org.apache.commons.httpclient.HttpState;

public class Logger {
    static boolean debug;

    public static Logger create(Class cls) {
        String s = System.getProperty("org.bytedeco.javacpp.logger", "").toLowerCase();
        if (s.equals("slf4j") || s.equals("slf4jlogger")) {
            return new Slf4jLogger(cls);
        }
        return new Logger();
    }

    static {
        boolean z = false;
        debug = false;
        String s = System.getProperty("org.bytedeco.javacpp.logger.debug", HttpState.PREEMPTIVE_DEFAULT).toLowerCase();
        if (s.equals("true") || s.equals("t") || s.equals("")) {
            z = true;
        }
        debug = z;
    }

    public boolean isDebugEnabled() {
        return debug;
    }

    public boolean isInfoEnabled() {
        return true;
    }

    public boolean isWarnEnabled() {
        return true;
    }

    public boolean isErrorEnabled() {
        return true;
    }

    public void debug(String s) {
        System.out.println(s);
    }

    public void info(String s) {
        System.out.println(s);
    }

    public void warn(String s) {
        PrintStream printStream = System.err;
        printStream.println("Warning: " + s);
    }

    public void error(String s) {
        PrintStream printStream = System.err;
        printStream.println("Error: " + s);
    }
}
