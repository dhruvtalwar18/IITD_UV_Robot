package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.BaseClientHandshake;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;

public class SubscriberHandshake extends BaseClientHandshake {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(SubscriberHandshake.class);

    public SubscriberHandshake(ConnectionHeader outgoingConnectionHeader) {
        super(outgoingConnectionHeader);
        Preconditions.checkNotNull(outgoingConnectionHeader.getField(ConnectionHeaderFields.TYPE));
        Preconditions.checkNotNull(outgoingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM));
    }

    public boolean handshake(ConnectionHeader incommingConnectionHeader) {
        setErrorMessage(incommingConnectionHeader.getField(ConnectionHeaderFields.ERROR));
        String incomingType = incommingConnectionHeader.getField(ConnectionHeaderFields.TYPE);
        if (incomingType == null) {
            setErrorMessage("Incoming type cannot be null.");
        } else if (!incomingType.equals(this.outgoingConnectionHeader.getField(ConnectionHeaderFields.TYPE))) {
            setErrorMessage("Message types don't match.");
        }
        String incomingMd5Checksum = incommingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM);
        if (incomingMd5Checksum == null) {
            setErrorMessage("Incoming MD5 checksum cannot be null.");
        } else if (!incomingMd5Checksum.equals(this.outgoingConnectionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM))) {
            setErrorMessage("Checksums don't match.");
        }
        return getErrorMessage() == null;
    }
}
