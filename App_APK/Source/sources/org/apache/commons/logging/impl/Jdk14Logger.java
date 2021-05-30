package org.apache.commons.logging.impl;

import android.support.v4.os.EnvironmentCompat;
import java.io.Serializable;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.logging.Log;

public class Jdk14Logger implements Log, Serializable {
    protected static final Level dummyLevel = Level.FINE;
    protected transient Logger logger = null;
    protected String name = null;

    public Jdk14Logger(String name2) {
        this.name = name2;
        this.logger = getLogger();
    }

    private void log(Level level, String msg, Throwable ex) {
        Logger logger2 = getLogger();
        if (logger2.isLoggable(level)) {
            StackTraceElement[] locations = new Throwable().getStackTrace();
            String cname = EnvironmentCompat.MEDIA_UNKNOWN;
            String method = EnvironmentCompat.MEDIA_UNKNOWN;
            if (locations != null && locations.length > 2) {
                StackTraceElement caller = locations[2];
                cname = caller.getClassName();
                method = caller.getMethodName();
            }
            String cname2 = cname;
            String method2 = method;
            if (ex == null) {
                logger2.logp(level, cname2, method2, msg);
            } else {
                logger2.logp(level, cname2, method2, msg, ex);
            }
        }
    }

    public void debug(Object message) {
        log(Level.FINE, String.valueOf(message), (Throwable) null);
    }

    public void debug(Object message, Throwable exception) {
        log(Level.FINE, String.valueOf(message), exception);
    }

    public void error(Object message) {
        log(Level.SEVERE, String.valueOf(message), (Throwable) null);
    }

    public void error(Object message, Throwable exception) {
        log(Level.SEVERE, String.valueOf(message), exception);
    }

    public void fatal(Object message) {
        log(Level.SEVERE, String.valueOf(message), (Throwable) null);
    }

    public void fatal(Object message, Throwable exception) {
        log(Level.SEVERE, String.valueOf(message), exception);
    }

    public Logger getLogger() {
        if (this.logger == null) {
            this.logger = Logger.getLogger(this.name);
        }
        return this.logger;
    }

    public void info(Object message) {
        log(Level.INFO, String.valueOf(message), (Throwable) null);
    }

    public void info(Object message, Throwable exception) {
        log(Level.INFO, String.valueOf(message), exception);
    }

    public boolean isDebugEnabled() {
        return getLogger().isLoggable(Level.FINE);
    }

    public boolean isErrorEnabled() {
        return getLogger().isLoggable(Level.SEVERE);
    }

    public boolean isFatalEnabled() {
        return getLogger().isLoggable(Level.SEVERE);
    }

    public boolean isInfoEnabled() {
        return getLogger().isLoggable(Level.INFO);
    }

    public boolean isTraceEnabled() {
        return getLogger().isLoggable(Level.FINEST);
    }

    public boolean isWarnEnabled() {
        return getLogger().isLoggable(Level.WARNING);
    }

    public void trace(Object message) {
        log(Level.FINEST, String.valueOf(message), (Throwable) null);
    }

    public void trace(Object message, Throwable exception) {
        log(Level.FINEST, String.valueOf(message), exception);
    }

    public void warn(Object message) {
        log(Level.WARNING, String.valueOf(message), (Throwable) null);
    }

    public void warn(Object message, Throwable exception) {
        log(Level.WARNING, String.valueOf(message), exception);
    }
}
