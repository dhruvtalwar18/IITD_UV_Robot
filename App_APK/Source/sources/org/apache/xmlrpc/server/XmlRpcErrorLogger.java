package org.apache.xmlrpc.server;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class XmlRpcErrorLogger {
    private static final Log log = LogFactory.getLog(XmlRpcErrorLogger.class);

    public void log(String pMessage, Throwable pThrowable) {
        log.error(pMessage, pThrowable);
    }

    public void log(String pMessage) {
        log.error(pMessage);
    }
}
