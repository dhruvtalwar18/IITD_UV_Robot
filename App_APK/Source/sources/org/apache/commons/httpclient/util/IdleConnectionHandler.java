package org.apache.commons.httpclient.util;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class IdleConnectionHandler {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$util$IdleConnectionHandler;
    private Map connectionToAdded = new HashMap();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$util$IdleConnectionHandler == null) {
            cls = class$("org.apache.commons.httpclient.util.IdleConnectionHandler");
            class$org$apache$commons$httpclient$util$IdleConnectionHandler = cls;
        } else {
            cls = class$org$apache$commons$httpclient$util$IdleConnectionHandler;
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

    public void add(HttpConnection connection) {
        Long timeAdded = new Long(System.currentTimeMillis());
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Adding connection at: ");
            stringBuffer.append(timeAdded);
            log.debug(stringBuffer.toString());
        }
        this.connectionToAdded.put(connection, timeAdded);
    }

    public void remove(HttpConnection connection) {
        this.connectionToAdded.remove(connection);
    }

    public void removeAll() {
        this.connectionToAdded.clear();
    }

    public void closeIdleConnections(long idleTime) {
        long idleTimeout = System.currentTimeMillis() - idleTime;
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Checking for connections, idleTimeout: ");
            stringBuffer.append(idleTimeout);
            log.debug(stringBuffer.toString());
        }
        Iterator connectionIter = this.connectionToAdded.keySet().iterator();
        while (connectionIter.hasNext()) {
            HttpConnection conn = (HttpConnection) connectionIter.next();
            Long connectionTime = (Long) this.connectionToAdded.get(conn);
            if (connectionTime.longValue() <= idleTimeout) {
                if (LOG.isDebugEnabled()) {
                    Log log2 = LOG;
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Closing connection, connection time: ");
                    stringBuffer2.append(connectionTime);
                    log2.debug(stringBuffer2.toString());
                }
                connectionIter.remove();
                conn.close();
            }
        }
    }
}
