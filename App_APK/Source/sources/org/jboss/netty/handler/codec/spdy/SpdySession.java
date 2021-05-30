package org.jboss.netty.handler.codec.spdy;

import java.util.Comparator;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import org.jboss.netty.channel.MessageEvent;

final class SpdySession {
    private static final SpdyProtocolException STREAM_CLOSED = new SpdyProtocolException("Stream closed");
    /* access modifiers changed from: private */
    public final Map<Integer, StreamState> activeStreams = new ConcurrentHashMap();

    SpdySession() {
    }

    /* access modifiers changed from: package-private */
    public int numActiveStreams() {
        return this.activeStreams.size();
    }

    /* access modifiers changed from: package-private */
    public boolean noActiveStreams() {
        return this.activeStreams.isEmpty();
    }

    /* access modifiers changed from: package-private */
    public boolean isActiveStream(int streamID) {
        return this.activeStreams.containsKey(new Integer(streamID));
    }

    /* access modifiers changed from: package-private */
    public Set<Integer> getActiveStreams() {
        TreeSet<Integer> StreamIDs = new TreeSet<>(new PriorityComparator());
        StreamIDs.addAll(this.activeStreams.keySet());
        return StreamIDs;
    }

    /* access modifiers changed from: package-private */
    public void acceptStream(int streamID, byte priority, boolean remoteSideClosed, boolean localSideClosed, int sendWindowSize, int receiveWindowSize) {
        if (!remoteSideClosed || !localSideClosed) {
            this.activeStreams.put(new Integer(streamID), new StreamState(priority, remoteSideClosed, localSideClosed, sendWindowSize, receiveWindowSize));
        }
    }

    /* access modifiers changed from: package-private */
    public void removeStream(int streamID) {
        Integer StreamID = new Integer(streamID);
        StreamState state = this.activeStreams.get(StreamID);
        this.activeStreams.remove(StreamID);
        if (state != null) {
            for (MessageEvent e = state.removePendingWrite(); e != null; e = state.removePendingWrite()) {
                e.getFuture().setFailure(STREAM_CLOSED);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public boolean isRemoteSideClosed(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        return state == null || state.isRemoteSideClosed();
    }

    /* access modifiers changed from: package-private */
    public void closeRemoteSide(int streamID) {
        Integer StreamID = new Integer(streamID);
        StreamState state = this.activeStreams.get(StreamID);
        if (state != null) {
            state.closeRemoteSide();
            if (state.isLocalSideClosed()) {
                this.activeStreams.remove(StreamID);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public boolean isLocalSideClosed(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        return state == null || state.isLocalSideClosed();
    }

    /* access modifiers changed from: package-private */
    public void closeLocalSide(int streamID) {
        Integer StreamID = new Integer(streamID);
        StreamState state = this.activeStreams.get(StreamID);
        if (state != null) {
            state.closeLocalSide();
            if (state.isRemoteSideClosed()) {
                this.activeStreams.remove(StreamID);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public boolean hasReceivedReply(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        return state != null && state.hasReceivedReply();
    }

    /* access modifiers changed from: package-private */
    public void receivedReply(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (state != null) {
            state.receivedReply();
        }
    }

    /* access modifiers changed from: package-private */
    public int getSendWindowSize(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (state != null) {
            return state.getSendWindowSize();
        }
        return -1;
    }

    /* access modifiers changed from: package-private */
    public int updateSendWindowSize(int streamID, int deltaWindowSize) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (state != null) {
            return state.updateSendWindowSize(deltaWindowSize);
        }
        return -1;
    }

    /* access modifiers changed from: package-private */
    public int updateReceiveWindowSize(int streamID, int deltaWindowSize) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (deltaWindowSize > 0) {
            state.setReceiveWindowSizeLowerBound(0);
        }
        if (state != null) {
            return state.updateReceiveWindowSize(deltaWindowSize);
        }
        return -1;
    }

    /* access modifiers changed from: package-private */
    public int getReceiveWindowSizeLowerBound(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (state != null) {
            return state.getReceiveWindowSizeLowerBound();
        }
        return 0;
    }

    /* access modifiers changed from: package-private */
    public void updateAllReceiveWindowSizes(int deltaWindowSize) {
        for (StreamState state : this.activeStreams.values()) {
            state.updateReceiveWindowSize(deltaWindowSize);
            if (deltaWindowSize < 0) {
                state.setReceiveWindowSizeLowerBound(deltaWindowSize);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public boolean putPendingWrite(int streamID, MessageEvent evt) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        return state != null && state.putPendingWrite(evt);
    }

    /* access modifiers changed from: package-private */
    public MessageEvent getPendingWrite(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (state != null) {
            return state.getPendingWrite();
        }
        return null;
    }

    /* access modifiers changed from: package-private */
    public MessageEvent removePendingWrite(int streamID) {
        StreamState state = this.activeStreams.get(new Integer(streamID));
        if (state != null) {
            return state.removePendingWrite();
        }
        return null;
    }

    private static final class StreamState {
        private volatile boolean localSideClosed;
        private final ConcurrentLinkedQueue<MessageEvent> pendingWriteQueue = new ConcurrentLinkedQueue<>();
        private final byte priority;
        private final AtomicInteger receiveWindowSize;
        private volatile int receiveWindowSizeLowerBound;
        private boolean receivedReply;
        private volatile boolean remoteSideClosed;
        private final AtomicInteger sendWindowSize;

        StreamState(byte priority2, boolean remoteSideClosed2, boolean localSideClosed2, int sendWindowSize2, int receiveWindowSize2) {
            this.priority = priority2;
            this.remoteSideClosed = remoteSideClosed2;
            this.localSideClosed = localSideClosed2;
            this.sendWindowSize = new AtomicInteger(sendWindowSize2);
            this.receiveWindowSize = new AtomicInteger(receiveWindowSize2);
        }

        /* access modifiers changed from: package-private */
        public byte getPriority() {
            return this.priority;
        }

        /* access modifiers changed from: package-private */
        public boolean isRemoteSideClosed() {
            return this.remoteSideClosed;
        }

        /* access modifiers changed from: package-private */
        public void closeRemoteSide() {
            this.remoteSideClosed = true;
        }

        /* access modifiers changed from: package-private */
        public boolean isLocalSideClosed() {
            return this.localSideClosed;
        }

        /* access modifiers changed from: package-private */
        public void closeLocalSide() {
            this.localSideClosed = true;
        }

        /* access modifiers changed from: package-private */
        public boolean hasReceivedReply() {
            return this.receivedReply;
        }

        /* access modifiers changed from: package-private */
        public void receivedReply() {
            this.receivedReply = true;
        }

        /* access modifiers changed from: package-private */
        public int getSendWindowSize() {
            return this.sendWindowSize.get();
        }

        /* access modifiers changed from: package-private */
        public int updateSendWindowSize(int deltaWindowSize) {
            return this.sendWindowSize.addAndGet(deltaWindowSize);
        }

        /* access modifiers changed from: package-private */
        public int updateReceiveWindowSize(int deltaWindowSize) {
            return this.receiveWindowSize.addAndGet(deltaWindowSize);
        }

        /* access modifiers changed from: package-private */
        public int getReceiveWindowSizeLowerBound() {
            return this.receiveWindowSizeLowerBound;
        }

        /* access modifiers changed from: package-private */
        public void setReceiveWindowSizeLowerBound(int receiveWindowSizeLowerBound2) {
            this.receiveWindowSizeLowerBound = receiveWindowSizeLowerBound2;
        }

        /* access modifiers changed from: package-private */
        public boolean putPendingWrite(MessageEvent evt) {
            return this.pendingWriteQueue.offer(evt);
        }

        /* access modifiers changed from: package-private */
        public MessageEvent getPendingWrite() {
            return this.pendingWriteQueue.peek();
        }

        /* access modifiers changed from: package-private */
        public MessageEvent removePendingWrite() {
            return this.pendingWriteQueue.poll();
        }
    }

    private final class PriorityComparator implements Comparator<Integer> {
        PriorityComparator() {
        }

        public int compare(Integer id1, Integer id2) {
            return ((StreamState) SpdySession.this.activeStreams.get(id1)).getPriority() - ((StreamState) SpdySession.this.activeStreams.get(id2)).getPriority();
        }
    }
}
