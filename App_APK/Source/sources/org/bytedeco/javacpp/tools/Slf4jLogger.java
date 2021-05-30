package org.bytedeco.javacpp.tools;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Slf4jLogger extends Logger {
    final Logger logger;

    public Slf4jLogger(Class cls) {
        this.logger = LoggerFactory.getLogger(cls);
    }

    public boolean isDebugEnabled() {
        return this.logger.isDebugEnabled();
    }

    public boolean isInfoEnabled() {
        return this.logger.isInfoEnabled();
    }

    public boolean isWarnEnabled() {
        return this.logger.isWarnEnabled();
    }

    public boolean isErrorEnabled() {
        return this.logger.isErrorEnabled();
    }

    public void debug(String s) {
        this.logger.debug(s);
    }

    public void info(String s) {
        this.logger.info(s);
    }

    public void warn(String s) {
        this.logger.warn(s);
    }

    public void error(String s) {
        this.logger.error(s);
    }
}
