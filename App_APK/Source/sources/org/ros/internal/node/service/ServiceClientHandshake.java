package org.ros.internal.node.service;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.BaseClientHandshake;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;

public class ServiceClientHandshake extends BaseClientHandshake {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(ServiceClientHandshake.class);

    public ServiceClientHandshake(ConnectionHeader outgoingConnectionHeader) {
        super(outgoingConnectionHeader);
    }

    public boolean handshake(ConnectionHeader incommingConnectionHeader) {
        setErrorMessage(incommingConnectionHeader.getField(ConnectionHeaderFields.ERROR));
        if (getErrorMessage() != null) {
            return false;
        }
        if (!incommingConnectionHeader.getField(ConnectionHeaderFields.TYPE).equals(incommingConnectionHeader.getField(ConnectionHeaderFields.TYPE))) {
            setErrorMessage("Message types don't match.");
        }
        if (!incommingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM).equals(incommingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM))) {
            setErrorMessage("Checksums don't match.");
        }
        if (getErrorMessage() == null) {
            return true;
        }
        return false;
    }
}
