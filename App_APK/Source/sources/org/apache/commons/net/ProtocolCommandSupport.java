package org.apache.commons.net;

import java.io.Serializable;
import java.util.Iterator;
import org.apache.commons.net.util.ListenerList;

public class ProtocolCommandSupport implements Serializable {
    private ListenerList __listeners = new ListenerList();
    private Object __source;

    public ProtocolCommandSupport(Object source) {
        this.__source = source;
    }

    public void fireCommandSent(String command, String message) {
        ProtocolCommandEvent event = new ProtocolCommandEvent(this.__source, command, message);
        Iterator i$ = this.__listeners.iterator();
        while (i$.hasNext()) {
            ((ProtocolCommandListener) i$.next()).protocolCommandSent(event);
        }
    }

    public void fireReplyReceived(int replyCode, String message) {
        ProtocolCommandEvent event = new ProtocolCommandEvent(this.__source, replyCode, message);
        Iterator i$ = this.__listeners.iterator();
        while (i$.hasNext()) {
            ((ProtocolCommandListener) i$.next()).protocolReplyReceived(event);
        }
    }

    public void addProtocolCommandListener(ProtocolCommandListener listener) {
        this.__listeners.addListener(listener);
    }

    public void removeProtocolCommandListener(ProtocolCommandListener listener) {
        this.__listeners.removeListener(listener);
    }

    public int getListenerCount() {
        return this.__listeners.getListenerCount();
    }
}
