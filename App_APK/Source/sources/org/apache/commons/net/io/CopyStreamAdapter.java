package org.apache.commons.net.io;

import java.util.Iterator;
import org.apache.commons.net.util.ListenerList;

public class CopyStreamAdapter implements CopyStreamListener {
    private ListenerList internalListeners = new ListenerList();

    public void bytesTransferred(CopyStreamEvent event) {
        bytesTransferred(event.getTotalBytesTransferred(), event.getBytesTransferred(), event.getStreamSize());
    }

    public void bytesTransferred(long totalBytesTransferred, int bytesTransferred, long streamSize) {
        CopyStreamEvent event = new CopyStreamEvent(this, totalBytesTransferred, bytesTransferred, streamSize);
        Iterator i$ = this.internalListeners.iterator();
        while (i$.hasNext()) {
            ((CopyStreamListener) i$.next()).bytesTransferred(event);
        }
    }

    public void addCopyStreamListener(CopyStreamListener listener) {
        this.internalListeners.addListener(listener);
    }

    public void removeCopyStreamListener(CopyStreamListener listener) {
        this.internalListeners.removeListener(listener);
    }
}
