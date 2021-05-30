package org.jboss.netty.handler.codec.spdy;

import java.net.SocketAddress;
import java.nio.channels.ClosedChannelException;
import java.util.concurrent.atomic.AtomicInteger;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.ExceptionEvent;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;

public class SpdySessionHandler extends SimpleChannelUpstreamHandler implements ChannelDownstreamHandler {
    private static final int DEFAULT_WINDOW_SIZE = 65536;
    private static final SpdyProtocolException PROTOCOL_EXCEPTION = new SpdyProtocolException();
    private volatile ChannelFuture closeSessionFuture;
    private final boolean flowControl;
    private final Object flowControlLock;
    private volatile int initialReceiveWindowSize;
    private volatile int initialSendWindowSize;
    private volatile int lastGoodStreamID;
    private volatile int localConcurrentStreams;
    private volatile int maxConcurrentStreams;
    private final AtomicInteger pings;
    private volatile boolean receivedGoAwayFrame;
    private volatile int remoteConcurrentStreams;
    private volatile boolean sentGoAwayFrame;
    private final boolean server;
    private final SpdySession spdySession;

    @Deprecated
    public SpdySessionHandler(boolean server2) {
        this(2, server2);
    }

    public SpdySessionHandler(int version, boolean server2) {
        this.spdySession = new SpdySession();
        this.initialSendWindowSize = 65536;
        this.initialReceiveWindowSize = 65536;
        this.flowControlLock = new Object();
        this.pings = new AtomicInteger();
        if (version < 2 || version > 3) {
            throw new IllegalArgumentException("unsupported version: " + version);
        }
        this.server = server2;
        this.flowControl = version >= 3;
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        int newInitialWindowSize;
        Object msg = e.getMessage();
        if (msg instanceof SpdyDataFrame) {
            SpdyDataFrame spdyDataFrame = (SpdyDataFrame) msg;
            int streamID = spdyDataFrame.getStreamId();
            if (!this.spdySession.isActiveStream(streamID)) {
                if (streamID <= this.lastGoodStreamID) {
                    issueStreamError(ctx, e.getRemoteAddress(), streamID, SpdyStreamStatus.PROTOCOL_ERROR);
                    return;
                } else if (!this.sentGoAwayFrame) {
                    issueStreamError(ctx, e.getRemoteAddress(), streamID, SpdyStreamStatus.INVALID_STREAM);
                    return;
                } else {
                    return;
                }
            } else if (this.spdySession.isRemoteSideClosed(streamID)) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID, SpdyStreamStatus.STREAM_ALREADY_CLOSED);
                return;
            } else if (isRemoteInitiatedID(streamID) || this.spdySession.hasReceivedReply(streamID)) {
                if (this.flowControl) {
                    int newWindowSize = this.spdySession.updateReceiveWindowSize(streamID, spdyDataFrame.getData().readableBytes() * -1);
                    if (newWindowSize < this.spdySession.getReceiveWindowSizeLowerBound(streamID)) {
                        issueStreamError(ctx, e.getRemoteAddress(), streamID, SpdyStreamStatus.FLOW_CONTROL_ERROR);
                        return;
                    }
                    if (newWindowSize < 0) {
                        while (spdyDataFrame.getData().readableBytes() > this.initialReceiveWindowSize) {
                            SpdyDataFrame partialDataFrame = new DefaultSpdyDataFrame(streamID);
                            partialDataFrame.setData(spdyDataFrame.getData().readSlice(this.initialReceiveWindowSize));
                            Channels.fireMessageReceived(ctx, (Object) partialDataFrame, e.getRemoteAddress());
                        }
                    }
                    if (newWindowSize <= this.initialReceiveWindowSize / 2 && !spdyDataFrame.isLast()) {
                        int deltaWindowSize = this.initialReceiveWindowSize - newWindowSize;
                        this.spdySession.updateReceiveWindowSize(streamID, deltaWindowSize);
                        Channels.write(ctx, Channels.future(e.getChannel()), new DefaultSpdyWindowUpdateFrame(streamID, deltaWindowSize), e.getRemoteAddress());
                    }
                }
                if (spdyDataFrame.isLast()) {
                    halfCloseStream(streamID, true);
                }
            } else {
                issueStreamError(ctx, e.getRemoteAddress(), streamID, SpdyStreamStatus.PROTOCOL_ERROR);
                return;
            }
        } else if (msg instanceof SpdySynStreamFrame) {
            SpdySynStreamFrame spdySynStreamFrame = (SpdySynStreamFrame) msg;
            int streamID2 = spdySynStreamFrame.getStreamId();
            if (spdySynStreamFrame.isInvalid() || !isRemoteInitiatedID(streamID2) || this.spdySession.isActiveStream(streamID2)) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID2, SpdyStreamStatus.PROTOCOL_ERROR);
                return;
            } else if (streamID2 <= this.lastGoodStreamID) {
                issueSessionError(ctx, e.getChannel(), e.getRemoteAddress(), SpdySessionStatus.PROTOCOL_ERROR);
                return;
            } else if (!acceptStream(streamID2, spdySynStreamFrame.getPriority(), spdySynStreamFrame.isLast(), spdySynStreamFrame.isUnidirectional())) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID2, SpdyStreamStatus.REFUSED_STREAM);
                return;
            }
        } else if (msg instanceof SpdySynReplyFrame) {
            SpdySynReplyFrame spdySynReplyFrame = (SpdySynReplyFrame) msg;
            int streamID3 = spdySynReplyFrame.getStreamId();
            if (spdySynReplyFrame.isInvalid() || isRemoteInitiatedID(streamID3) || this.spdySession.isRemoteSideClosed(streamID3)) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID3, SpdyStreamStatus.INVALID_STREAM);
                return;
            } else if (this.spdySession.hasReceivedReply(streamID3)) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID3, SpdyStreamStatus.STREAM_IN_USE);
                return;
            } else {
                this.spdySession.receivedReply(streamID3);
                if (spdySynReplyFrame.isLast()) {
                    halfCloseStream(streamID3, true);
                }
            }
        } else if (msg instanceof SpdyRstStreamFrame) {
            removeStream(((SpdyRstStreamFrame) msg).getStreamId());
        } else if (msg instanceof SpdySettingsFrame) {
            SpdySettingsFrame spdySettingsFrame = (SpdySettingsFrame) msg;
            int newConcurrentStreams = spdySettingsFrame.getValue(4);
            if (newConcurrentStreams >= 0) {
                updateConcurrentStreams(newConcurrentStreams, true);
            }
            if (spdySettingsFrame.isPersisted(7)) {
                spdySettingsFrame.removeValue(7);
            }
            spdySettingsFrame.setPersistValue(7, false);
            if (this.flowControl && (newInitialWindowSize = spdySettingsFrame.getValue(7)) >= 0) {
                updateInitialSendWindowSize(newInitialWindowSize);
            }
        } else if (msg instanceof SpdyPingFrame) {
            SpdyPingFrame spdyPingFrame = (SpdyPingFrame) msg;
            if (isRemoteInitiatedID(spdyPingFrame.getId())) {
                Channels.write(ctx, Channels.future(e.getChannel()), spdyPingFrame, e.getRemoteAddress());
                return;
            } else if (this.pings.get() != 0) {
                this.pings.getAndDecrement();
            } else {
                return;
            }
        } else if (msg instanceof SpdyGoAwayFrame) {
            this.receivedGoAwayFrame = true;
        } else if (msg instanceof SpdyHeadersFrame) {
            SpdyHeadersFrame spdyHeadersFrame = (SpdyHeadersFrame) msg;
            int streamID4 = spdyHeadersFrame.getStreamId();
            if (spdyHeadersFrame.isInvalid()) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID4, SpdyStreamStatus.PROTOCOL_ERROR);
                return;
            } else if (this.spdySession.isRemoteSideClosed(streamID4)) {
                issueStreamError(ctx, e.getRemoteAddress(), streamID4, SpdyStreamStatus.INVALID_STREAM);
                return;
            } else if (spdyHeadersFrame.isLast()) {
                halfCloseStream(streamID4, true);
            }
        } else if (msg instanceof SpdyWindowUpdateFrame) {
            if (this.flowControl) {
                SpdyWindowUpdateFrame spdyWindowUpdateFrame = (SpdyWindowUpdateFrame) msg;
                int streamID5 = spdyWindowUpdateFrame.getStreamId();
                int deltaWindowSize2 = spdyWindowUpdateFrame.getDeltaWindowSize();
                if (!this.spdySession.isLocalSideClosed(streamID5)) {
                    if (this.spdySession.getSendWindowSize(streamID5) > Integer.MAX_VALUE - deltaWindowSize2) {
                        issueStreamError(ctx, e.getRemoteAddress(), streamID5, SpdyStreamStatus.FLOW_CONTROL_ERROR);
                        return;
                    } else {
                        updateSendWindowSize(ctx, streamID5, deltaWindowSize2);
                        return;
                    }
                } else {
                    return;
                }
            } else {
                return;
            }
        }
        super.messageReceived(ctx, e);
    }

    public void exceptionCaught(ChannelHandlerContext ctx, ExceptionEvent e) throws Exception {
        if (e.getCause() instanceof SpdyProtocolException) {
            issueSessionError(ctx, e.getChannel(), (SocketAddress) null, SpdySessionStatus.PROTOCOL_ERROR);
        }
        super.exceptionCaught(ctx, e);
    }

    public void handleDownstream(ChannelHandlerContext ctx, ChannelEvent evt) throws Exception {
        int newInitialWindowSize;
        if (evt instanceof ChannelStateEvent) {
            ChannelStateEvent e = (ChannelStateEvent) evt;
            switch (e.getState()) {
                case OPEN:
                case CONNECTED:
                case BOUND:
                    if (Boolean.FALSE.equals(e.getValue()) || e.getValue() == null) {
                        sendGoAwayFrame(ctx, e);
                        return;
                    }
            }
        }
        if (!(evt instanceof MessageEvent)) {
            ctx.sendDownstream(evt);
            return;
        }
        MessageEvent e2 = (MessageEvent) evt;
        Object msg = e2.getMessage();
        if (msg instanceof SpdyDataFrame) {
            SpdyDataFrame spdyDataFrame = (SpdyDataFrame) msg;
            final int streamID = spdyDataFrame.getStreamId();
            if (this.spdySession.isLocalSideClosed(streamID)) {
                e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
                return;
            }
            if (this.flowControl) {
                synchronized (this.flowControlLock) {
                    int dataLength = spdyDataFrame.getData().readableBytes();
                    int sendWindowSize = this.spdySession.getSendWindowSize(streamID);
                    if (sendWindowSize >= dataLength) {
                        this.spdySession.updateSendWindowSize(streamID, dataLength * -1);
                        final SocketAddress remoteAddress = e2.getRemoteAddress();
                        final ChannelHandlerContext context = ctx;
                        e2.getFuture().addListener(new ChannelFutureListener() {
                            public void operationComplete(ChannelFuture future) throws Exception {
                                if (!future.isSuccess()) {
                                    SpdySessionHandler.this.issueStreamError(context, remoteAddress, streamID, SpdyStreamStatus.INTERNAL_ERROR);
                                }
                            }
                        });
                    } else if (sendWindowSize > 0) {
                        this.spdySession.updateSendWindowSize(streamID, sendWindowSize * -1);
                        SpdyDataFrame partialDataFrame = new DefaultSpdyDataFrame(streamID);
                        partialDataFrame.setData(spdyDataFrame.getData().readSlice(sendWindowSize));
                        this.spdySession.putPendingWrite(streamID, e2);
                        ChannelFuture writeFuture = Channels.future(e2.getChannel());
                        final SocketAddress remoteAddress2 = e2.getRemoteAddress();
                        final ChannelHandlerContext context2 = ctx;
                        e2.getFuture().addListener(new ChannelFutureListener() {
                            public void operationComplete(ChannelFuture future) throws Exception {
                                if (!future.isSuccess()) {
                                    SpdySessionHandler.this.issueStreamError(context2, remoteAddress2, streamID, SpdyStreamStatus.INTERNAL_ERROR);
                                }
                            }
                        });
                        Channels.write(ctx, writeFuture, partialDataFrame, remoteAddress2);
                        return;
                    } else {
                        this.spdySession.putPendingWrite(streamID, e2);
                        return;
                    }
                }
            }
            if (spdyDataFrame.isLast()) {
                halfCloseStream(streamID, false);
            }
        } else if (msg instanceof SpdySynStreamFrame) {
            SpdySynStreamFrame spdySynStreamFrame = (SpdySynStreamFrame) msg;
            int streamID2 = spdySynStreamFrame.getStreamId();
            if (isRemoteInitiatedID(streamID2)) {
                e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
                return;
            } else if (!acceptStream(streamID2, spdySynStreamFrame.getPriority(), spdySynStreamFrame.isUnidirectional(), spdySynStreamFrame.isLast())) {
                e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
                return;
            }
        } else if (msg instanceof SpdySynReplyFrame) {
            SpdySynReplyFrame spdySynReplyFrame = (SpdySynReplyFrame) msg;
            int streamID3 = spdySynReplyFrame.getStreamId();
            if (!isRemoteInitiatedID(streamID3) || this.spdySession.isLocalSideClosed(streamID3)) {
                e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
                return;
            } else if (spdySynReplyFrame.isLast()) {
                halfCloseStream(streamID3, false);
            }
        } else if (msg instanceof SpdyRstStreamFrame) {
            removeStream(((SpdyRstStreamFrame) msg).getStreamId());
        } else if (msg instanceof SpdySettingsFrame) {
            SpdySettingsFrame spdySettingsFrame = (SpdySettingsFrame) msg;
            int newConcurrentStreams = spdySettingsFrame.getValue(4);
            if (newConcurrentStreams >= 0) {
                updateConcurrentStreams(newConcurrentStreams, false);
            }
            if (spdySettingsFrame.isPersisted(7)) {
                spdySettingsFrame.removeValue(7);
            }
            spdySettingsFrame.setPersistValue(7, false);
            if (this.flowControl && (newInitialWindowSize = spdySettingsFrame.getValue(7)) >= 0) {
                updateInitialReceiveWindowSize(newInitialWindowSize);
            }
        } else if (msg instanceof SpdyPingFrame) {
            SpdyPingFrame spdyPingFrame = (SpdyPingFrame) msg;
            if (isRemoteInitiatedID(spdyPingFrame.getId())) {
                ChannelFuture future = e2.getFuture();
                future.setFailure(new IllegalArgumentException("invalid PING ID: " + spdyPingFrame.getId()));
                return;
            }
            this.pings.getAndIncrement();
        } else if (msg instanceof SpdyGoAwayFrame) {
            e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
            return;
        } else if (msg instanceof SpdyHeadersFrame) {
            SpdyHeadersFrame spdyHeadersFrame = (SpdyHeadersFrame) msg;
            int streamID4 = spdyHeadersFrame.getStreamId();
            if (this.spdySession.isLocalSideClosed(streamID4)) {
                e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
                return;
            } else if (spdyHeadersFrame.isLast()) {
                halfCloseStream(streamID4, false);
            }
        } else if (msg instanceof SpdyWindowUpdateFrame) {
            e2.getFuture().setFailure(PROTOCOL_EXCEPTION);
            return;
        }
        ctx.sendDownstream(evt);
    }

    private void issueSessionError(ChannelHandlerContext ctx, Channel channel, SocketAddress remoteAddress, SpdySessionStatus status) {
        sendGoAwayFrame(ctx, channel, remoteAddress, status).addListener(ChannelFutureListener.CLOSE);
    }

    /* access modifiers changed from: private */
    public void issueStreamError(ChannelHandlerContext ctx, SocketAddress remoteAddress, int streamID, SpdyStreamStatus status) {
        boolean fireMessageReceived = !this.spdySession.isRemoteSideClosed(streamID);
        removeStream(streamID);
        SpdyRstStreamFrame spdyRstStreamFrame = new DefaultSpdyRstStreamFrame(streamID, status);
        Channels.write(ctx, Channels.future(ctx.getChannel()), spdyRstStreamFrame, remoteAddress);
        if (fireMessageReceived) {
            Channels.fireMessageReceived(ctx, (Object) spdyRstStreamFrame, remoteAddress);
        }
    }

    private boolean isRemoteInitiatedID(int ID) {
        boolean serverID = SpdyCodecUtil.isServerId(ID);
        return (this.server && !serverID) || (!this.server && serverID);
    }

    private void updateConcurrentStreams(int newConcurrentStreams, boolean remote) {
        if (remote) {
            this.remoteConcurrentStreams = newConcurrentStreams;
        } else {
            this.localConcurrentStreams = newConcurrentStreams;
        }
        if (this.localConcurrentStreams == this.remoteConcurrentStreams) {
            this.maxConcurrentStreams = this.localConcurrentStreams;
        } else if (this.localConcurrentStreams == 0) {
            this.maxConcurrentStreams = this.remoteConcurrentStreams;
        } else if (this.remoteConcurrentStreams == 0) {
            this.maxConcurrentStreams = this.localConcurrentStreams;
        } else if (this.localConcurrentStreams > this.remoteConcurrentStreams) {
            this.maxConcurrentStreams = this.remoteConcurrentStreams;
        } else {
            this.maxConcurrentStreams = this.localConcurrentStreams;
        }
    }

    private synchronized void updateInitialSendWindowSize(int newInitialWindowSize) {
        int deltaWindowSize = newInitialWindowSize - this.initialSendWindowSize;
        this.initialSendWindowSize = newInitialWindowSize;
        for (Integer StreamID : this.spdySession.getActiveStreams()) {
            this.spdySession.updateSendWindowSize(StreamID.intValue(), deltaWindowSize);
        }
    }

    private synchronized void updateInitialReceiveWindowSize(int newInitialWindowSize) {
        this.initialReceiveWindowSize = newInitialWindowSize;
        this.spdySession.updateAllReceiveWindowSizes(newInitialWindowSize - this.initialReceiveWindowSize);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:21:0x0032, code lost:
        return false;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized boolean acceptStream(int r9, byte r10, boolean r11, boolean r12) {
        /*
            r8 = this;
            monitor-enter(r8)
            boolean r0 = r8.receivedGoAwayFrame     // Catch:{ all -> 0x0033 }
            r1 = 0
            if (r0 != 0) goto L_0x0031
            boolean r0 = r8.sentGoAwayFrame     // Catch:{ all -> 0x0033 }
            if (r0 == 0) goto L_0x000b
            goto L_0x0031
        L_0x000b:
            int r0 = r8.maxConcurrentStreams     // Catch:{ all -> 0x0033 }
            if (r0 == 0) goto L_0x0019
            org.jboss.netty.handler.codec.spdy.SpdySession r2 = r8.spdySession     // Catch:{ all -> 0x0033 }
            int r2 = r2.numActiveStreams()     // Catch:{ all -> 0x0033 }
            if (r2 < r0) goto L_0x0019
            monitor-exit(r8)
            return r1
        L_0x0019:
            org.jboss.netty.handler.codec.spdy.SpdySession r1 = r8.spdySession     // Catch:{ all -> 0x0033 }
            int r6 = r8.initialSendWindowSize     // Catch:{ all -> 0x0033 }
            int r7 = r8.initialReceiveWindowSize     // Catch:{ all -> 0x0033 }
            r2 = r9
            r3 = r10
            r4 = r11
            r5 = r12
            r1.acceptStream(r2, r3, r4, r5, r6, r7)     // Catch:{ all -> 0x0033 }
            boolean r1 = r8.isRemoteInitiatedID(r9)     // Catch:{ all -> 0x0033 }
            if (r1 == 0) goto L_0x002e
            r8.lastGoodStreamID = r9     // Catch:{ all -> 0x0033 }
        L_0x002e:
            r1 = 1
            monitor-exit(r8)
            return r1
        L_0x0031:
            monitor-exit(r8)
            return r1
        L_0x0033:
            r9 = move-exception
            monitor-exit(r8)
            throw r9
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.spdy.SpdySessionHandler.acceptStream(int, byte, boolean, boolean):boolean");
    }

    private void halfCloseStream(int streamID, boolean remote) {
        if (remote) {
            this.spdySession.closeRemoteSide(streamID);
        } else {
            this.spdySession.closeLocalSide(streamID);
        }
        if (this.closeSessionFuture != null && this.spdySession.noActiveStreams()) {
            this.closeSessionFuture.setSuccess();
        }
    }

    private void removeStream(int streamID) {
        this.spdySession.removeStream(streamID);
        if (this.closeSessionFuture != null && this.spdySession.noActiveStreams()) {
            this.closeSessionFuture.setSuccess();
        }
    }

    private void updateSendWindowSize(ChannelHandlerContext ctx, final int streamID, int deltaWindowSize) {
        synchronized (this.flowControlLock) {
            int newWindowSize = this.spdySession.updateSendWindowSize(streamID, deltaWindowSize);
            while (true) {
                if (newWindowSize <= 0) {
                    break;
                }
                MessageEvent e = this.spdySession.getPendingWrite(streamID);
                if (e == null) {
                    break;
                }
                SpdyDataFrame spdyDataFrame = (SpdyDataFrame) e.getMessage();
                int dataFrameSize = spdyDataFrame.getData().readableBytes();
                if (newWindowSize >= dataFrameSize) {
                    this.spdySession.removePendingWrite(streamID);
                    newWindowSize = this.spdySession.updateSendWindowSize(streamID, dataFrameSize * -1);
                    final SocketAddress remoteAddress = e.getRemoteAddress();
                    final ChannelHandlerContext context = ctx;
                    e.getFuture().addListener(new ChannelFutureListener() {
                        public void operationComplete(ChannelFuture future) throws Exception {
                            if (!future.isSuccess()) {
                                SpdySessionHandler.this.issueStreamError(context, remoteAddress, streamID, SpdyStreamStatus.INTERNAL_ERROR);
                            }
                        }
                    });
                    if (spdyDataFrame.isLast()) {
                        halfCloseStream(streamID, false);
                    }
                    Channels.write(ctx, e.getFuture(), spdyDataFrame, e.getRemoteAddress());
                } else {
                    this.spdySession.updateSendWindowSize(streamID, newWindowSize * -1);
                    SpdyDataFrame partialDataFrame = new DefaultSpdyDataFrame(streamID);
                    partialDataFrame.setData(spdyDataFrame.getData().readSlice(newWindowSize));
                    ChannelFuture writeFuture = Channels.future(e.getChannel());
                    final SocketAddress remoteAddress2 = e.getRemoteAddress();
                    final ChannelHandlerContext context2 = ctx;
                    e.getFuture().addListener(new ChannelFutureListener() {
                        public void operationComplete(ChannelFuture future) throws Exception {
                            if (!future.isSuccess()) {
                                SpdySessionHandler.this.issueStreamError(context2, remoteAddress2, streamID, SpdyStreamStatus.INTERNAL_ERROR);
                            }
                        }
                    });
                    Channels.write(ctx, writeFuture, partialDataFrame, remoteAddress2);
                    newWindowSize = 0;
                }
            }
        }
    }

    private void sendGoAwayFrame(ChannelHandlerContext ctx, ChannelStateEvent e) {
        if (!e.getChannel().isConnected()) {
            ctx.sendDownstream(e);
            return;
        }
        ChannelFuture future = sendGoAwayFrame(ctx, e.getChannel(), (SocketAddress) null, SpdySessionStatus.OK);
        if (this.spdySession.noActiveStreams()) {
            future.addListener(new ClosingChannelFutureListener(ctx, e));
            return;
        }
        this.closeSessionFuture = Channels.future(e.getChannel());
        this.closeSessionFuture.addListener(new ClosingChannelFutureListener(ctx, e));
    }

    private synchronized ChannelFuture sendGoAwayFrame(ChannelHandlerContext ctx, Channel channel, SocketAddress remoteAddress, SpdySessionStatus status) {
        if (!this.sentGoAwayFrame) {
            this.sentGoAwayFrame = true;
            SpdyGoAwayFrame spdyGoAwayFrame = new DefaultSpdyGoAwayFrame(this.lastGoodStreamID, status);
            ChannelFuture future = Channels.future(channel);
            Channels.write(ctx, future, spdyGoAwayFrame, remoteAddress);
            return future;
        }
        return Channels.succeededFuture(channel);
    }

    private static final class ClosingChannelFutureListener implements ChannelFutureListener {
        private final ChannelHandlerContext ctx;
        private final ChannelStateEvent e;

        ClosingChannelFutureListener(ChannelHandlerContext ctx2, ChannelStateEvent e2) {
            this.ctx = ctx2;
            this.e = e2;
        }

        public void operationComplete(ChannelFuture sentGoAwayFuture) throws Exception {
            if (!(sentGoAwayFuture.getCause() instanceof ClosedChannelException)) {
                Channels.close(this.ctx, this.e.getFuture());
            } else {
                this.e.getFuture().setSuccess();
            }
        }
    }
}
