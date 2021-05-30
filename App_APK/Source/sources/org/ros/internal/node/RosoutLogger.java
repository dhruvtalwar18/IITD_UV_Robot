package org.ros.internal.node;

import java.io.PrintWriter;
import java.io.StringWriter;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.Topics;
import org.ros.node.topic.Publisher;

class RosoutLogger implements Log {
    private final DefaultNode defaultNode;
    private final Log log;
    private final Publisher<rosgraph_msgs.Log> publisher;

    public RosoutLogger(DefaultNode defaultNode2) {
        this.defaultNode = defaultNode2;
        this.publisher = defaultNode2.newPublisher(Topics.ROSOUT, rosgraph_msgs.Log._TYPE);
        this.log = LogFactory.getLog(defaultNode2.getName().toString());
    }

    public Publisher<rosgraph_msgs.Log> getPublisher() {
        return this.publisher;
    }

    private void publish(byte level, Object message, Throwable throwable) {
        StringWriter stringWriter = new StringWriter();
        throwable.printStackTrace(new PrintWriter(stringWriter));
        publish(level, message.toString() + 10 + stringWriter.toString());
    }

    private void publish(byte level, Object message) {
        rosgraph_msgs.Log logMessage = this.publisher.newMessage();
        logMessage.getHeader().setStamp(this.defaultNode.getCurrentTime());
        logMessage.setLevel(level);
        logMessage.setName(this.defaultNode.getName().toString());
        logMessage.setMsg(message.toString());
        this.publisher.publish(logMessage);
    }

    public boolean isDebugEnabled() {
        return this.log.isDebugEnabled();
    }

    public boolean isErrorEnabled() {
        return this.log.isErrorEnabled();
    }

    public boolean isFatalEnabled() {
        return this.log.isFatalEnabled();
    }

    public boolean isInfoEnabled() {
        return this.log.isInfoEnabled();
    }

    public boolean isTraceEnabled() {
        return this.log.isTraceEnabled();
    }

    public boolean isWarnEnabled() {
        return this.log.isWarnEnabled();
    }

    public void trace(Object message) {
        this.log.trace(message);
        if (this.log.isTraceEnabled() && this.publisher != null) {
            publish((byte) 1, message);
        }
    }

    public void trace(Object message, Throwable t) {
        this.log.trace(message, t);
        if (this.log.isTraceEnabled() && this.publisher != null) {
            publish((byte) 1, message, t);
        }
    }

    public void debug(Object message) {
        this.log.debug(message);
        if (this.log.isDebugEnabled() && this.publisher != null) {
            publish((byte) 1, message);
        }
    }

    public void debug(Object message, Throwable t) {
        this.log.debug(message, t);
        if (this.log.isDebugEnabled() && this.publisher != null) {
            publish((byte) 1, message, t);
        }
    }

    public void info(Object message) {
        this.log.info(message);
        if (this.log.isInfoEnabled() && this.publisher != null) {
            publish((byte) 2, message);
        }
    }

    public void info(Object message, Throwable t) {
        this.log.info(message, t);
        if (this.log.isInfoEnabled() && this.publisher != null) {
            publish((byte) 2, message, t);
        }
    }

    public void warn(Object message) {
        this.log.warn(message);
        if (this.log.isWarnEnabled() && this.publisher != null) {
            publish((byte) 4, message);
        }
    }

    public void warn(Object message, Throwable t) {
        this.log.warn(message, t);
        if (this.log.isWarnEnabled() && this.publisher != null) {
            publish((byte) 4, message, t);
        }
    }

    public void error(Object message) {
        this.log.error(message);
        if (this.log.isErrorEnabled() && this.publisher != null) {
            publish((byte) 8, message);
        }
    }

    public void error(Object message, Throwable t) {
        this.log.error(message, t);
        if (this.log.isErrorEnabled() && this.publisher != null) {
            publish((byte) 8, message, t);
        }
    }

    public void fatal(Object message) {
        this.log.fatal(message);
        if (this.log.isFatalEnabled() && this.publisher != null) {
            publish((byte) 16, message);
        }
    }

    public void fatal(Object message, Throwable t) {
        this.log.fatal(message, t);
        if (this.log.isFatalEnabled() && this.publisher != null) {
            publish((byte) 16, message, t);
        }
    }
}
