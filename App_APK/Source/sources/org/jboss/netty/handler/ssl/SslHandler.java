package org.jboss.netty.handler.ssl;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.DatagramChannel;
import java.nio.channels.SocketChannel;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.regex.Pattern;
import javax.net.ssl.SSLEngine;
import javax.net.ssl.SSLEngineResult;
import javax.net.ssl.SSLException;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelState;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.DefaultChannelFuture;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.handler.codec.frame.FrameDecoder;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.DetectionUtil;
import org.jboss.netty.util.internal.NonReentrantLock;
import org.jboss.netty.util.internal.QueueFactory;
import sensor_msgs.NavSatStatus;

public class SslHandler extends FrameDecoder implements ChannelDownstreamHandler {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private static final ByteBuffer EMPTY_BUFFER = ByteBuffer.allocate(0);
    private static final Pattern IGNORABLE_CLASS_IN_STACK = Pattern.compile("^.*(Socket|DatagramChannel|SctpChannel).*$");
    private static final Pattern IGNORABLE_ERROR_MESSAGE = Pattern.compile("^.*(?:connection.*reset|connection.*closed|broken.*pipe).*$", 2);
    private static SslBufferPool defaultBufferPool;
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) SslHandler.class);
    private final SslBufferPool bufferPool;
    /* access modifiers changed from: private */
    public volatile ChannelHandlerContext ctx;
    private final Executor delegatedTaskExecutor;
    private volatile boolean enableRenegotiation;
    private final SSLEngine engine;
    private volatile ChannelFuture handshakeFuture;
    final Object handshakeLock;
    private volatile boolean handshaken;
    private boolean handshaking;
    int ignoreClosedChannelException;
    final Object ignoreClosedChannelExceptionLock;
    private volatile boolean issueHandshake;
    private int packetLength;
    private final Queue<MessageEvent> pendingEncryptedWrites;
    private final NonReentrantLock pendingEncryptedWritesLock;
    private final Queue<PendingWrite> pendingUnencryptedWrites;
    private final AtomicBoolean sentCloseNotify;
    private final AtomicBoolean sentFirstMessage;
    private final SSLEngineInboundCloseFuture sslEngineCloseFuture;
    private final boolean startTls;

    public static synchronized SslBufferPool getDefaultBufferPool() {
        SslBufferPool sslBufferPool;
        synchronized (SslHandler.class) {
            if (defaultBufferPool == null) {
                defaultBufferPool = new SslBufferPool();
            }
            sslBufferPool = defaultBufferPool;
        }
        return sslBufferPool;
    }

    public SslHandler(SSLEngine engine2) {
        this(engine2, getDefaultBufferPool(), (Executor) ImmediateExecutor.INSTANCE);
    }

    public SslHandler(SSLEngine engine2, SslBufferPool bufferPool2) {
        this(engine2, bufferPool2, (Executor) ImmediateExecutor.INSTANCE);
    }

    public SslHandler(SSLEngine engine2, boolean startTls2) {
        this(engine2, getDefaultBufferPool(), startTls2);
    }

    public SslHandler(SSLEngine engine2, SslBufferPool bufferPool2, boolean startTls2) {
        this(engine2, bufferPool2, startTls2, ImmediateExecutor.INSTANCE);
    }

    public SslHandler(SSLEngine engine2, Executor delegatedTaskExecutor2) {
        this(engine2, getDefaultBufferPool(), delegatedTaskExecutor2);
    }

    public SslHandler(SSLEngine engine2, SslBufferPool bufferPool2, Executor delegatedTaskExecutor2) {
        this(engine2, bufferPool2, false, delegatedTaskExecutor2);
    }

    public SslHandler(SSLEngine engine2, boolean startTls2, Executor delegatedTaskExecutor2) {
        this(engine2, getDefaultBufferPool(), startTls2, delegatedTaskExecutor2);
    }

    public SslHandler(SSLEngine engine2, SslBufferPool bufferPool2, boolean startTls2, Executor delegatedTaskExecutor2) {
        this.enableRenegotiation = true;
        this.handshakeLock = new Object();
        this.sentFirstMessage = new AtomicBoolean();
        this.sentCloseNotify = new AtomicBoolean();
        this.ignoreClosedChannelExceptionLock = new Object();
        this.pendingUnencryptedWrites = new LinkedList();
        this.pendingEncryptedWrites = QueueFactory.createQueue(MessageEvent.class);
        this.pendingEncryptedWritesLock = new NonReentrantLock();
        this.sslEngineCloseFuture = new SSLEngineInboundCloseFuture();
        this.packetLength = -1;
        if (engine2 == null) {
            throw new NullPointerException("engine");
        } else if (bufferPool2 == null) {
            throw new NullPointerException("bufferPool");
        } else if (delegatedTaskExecutor2 != null) {
            this.engine = engine2;
            this.bufferPool = bufferPool2;
            this.delegatedTaskExecutor = delegatedTaskExecutor2;
            this.startTls = startTls2;
        } else {
            throw new NullPointerException("delegatedTaskExecutor");
        }
    }

    public SSLEngine getEngine() {
        return this.engine;
    }

    public ChannelFuture handshake() {
        ChannelHandlerContext ctx2;
        Channel channel;
        Exception exception;
        ChannelFuture handshakeFuture2;
        if (!this.handshaken || isEnableRenegotiation()) {
            ctx2 = this.ctx;
            channel = ctx2.getChannel();
            exception = null;
            synchronized (this.handshakeLock) {
                if (this.handshaking) {
                    ChannelFuture channelFuture = this.handshakeFuture;
                    return channelFuture;
                }
                this.handshaking = true;
                try {
                    this.engine.beginHandshake();
                    runDelegatedTasks();
                    ChannelFuture future = Channels.future(channel);
                    this.handshakeFuture = future;
                    handshakeFuture2 = future;
                } catch (Exception e) {
                    ChannelFuture failedFuture = Channels.failedFuture(channel, e);
                    this.handshakeFuture = failedFuture;
                    handshakeFuture2 = failedFuture;
                    exception = e;
                }
            }
        } else {
            throw new IllegalStateException("renegotiation disabled");
        }
        if (exception == null) {
            try {
                wrapNonAppData(ctx2, channel);
            } catch (SSLException e2) {
                handshakeFuture2.setFailure(e2);
                Channels.fireExceptionCaught(ctx2, (Throwable) e2);
            }
        } else {
            Channels.fireExceptionCaught(ctx2, (Throwable) exception);
        }
        return handshakeFuture2;
        return handshakeFuture2;
    }

    @Deprecated
    public ChannelFuture handshake(Channel channel) {
        return handshake();
    }

    public ChannelFuture close() {
        ChannelHandlerContext ctx2 = this.ctx;
        Channel channel = ctx2.getChannel();
        try {
            this.engine.closeOutbound();
            return wrapNonAppData(ctx2, channel);
        } catch (SSLException e) {
            Channels.fireExceptionCaught(ctx2, (Throwable) e);
            return Channels.failedFuture(channel, e);
        }
    }

    @Deprecated
    public ChannelFuture close(Channel channel) {
        return close();
    }

    public boolean isEnableRenegotiation() {
        return this.enableRenegotiation;
    }

    public void setEnableRenegotiation(boolean enableRenegotiation2) {
        this.enableRenegotiation = enableRenegotiation2;
    }

    public void setIssueHandshake(boolean issueHandshake2) {
        this.issueHandshake = issueHandshake2;
    }

    public boolean isIssueHandshake() {
        return this.issueHandshake;
    }

    public ChannelFuture getSSLEngineInboundCloseFuture() {
        return this.sslEngineCloseFuture;
    }

    public void handleDownstream(ChannelHandlerContext context, ChannelEvent evt) throws Exception {
        PendingWrite pendingWrite;
        if (evt instanceof ChannelStateEvent) {
            ChannelStateEvent e = (ChannelStateEvent) evt;
            switch (e.getState()) {
                case OPEN:
                case CONNECTED:
                case BOUND:
                    if (Boolean.FALSE.equals(e.getValue()) || e.getValue() == null) {
                        closeOutboundAndChannel(context, e);
                        return;
                    }
            }
        }
        if (!(evt instanceof MessageEvent)) {
            context.sendDownstream(evt);
            return;
        }
        MessageEvent e2 = (MessageEvent) evt;
        if (!(e2.getMessage() instanceof ChannelBuffer)) {
            context.sendDownstream(evt);
        } else if (!this.startTls || !this.sentFirstMessage.compareAndSet(false, true)) {
            ChannelBuffer msg = (ChannelBuffer) e2.getMessage();
            if (msg.readable()) {
                pendingWrite = new PendingWrite(evt.getFuture(), msg.toByteBuffer(msg.readerIndex(), msg.readableBytes()));
            } else {
                pendingWrite = new PendingWrite(evt.getFuture(), (ByteBuffer) null);
            }
            synchronized (this.pendingUnencryptedWrites) {
                boolean offer = this.pendingUnencryptedWrites.offer(pendingWrite);
            }
            wrap(context, evt.getChannel());
        } else {
            context.sendDownstream(evt);
        }
    }

    public void channelDisconnected(ChannelHandlerContext ctx2, ChannelStateEvent e) throws Exception {
        String str;
        synchronized (this.handshakeLock) {
            if (this.handshaking) {
                this.handshakeFuture.setFailure(new ClosedChannelException());
            }
        }
        try {
            super.channelDisconnected(ctx2, e);
        } finally {
            unwrap(ctx2, e.getChannel(), ChannelBuffers.EMPTY_BUFFER, 0, 0);
            this.engine.closeOutbound();
            if (!this.sentCloseNotify.get() && this.handshaken) {
                try {
                    this.engine.closeInbound();
                } catch (SSLException ex) {
                    if (logger.isDebugEnabled()) {
                        str = "Failed to clean up SSLEngine.";
                        logger.debug(str, ex);
                    }
                }
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:13:0x0029, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void exceptionCaught(org.jboss.netty.channel.ChannelHandlerContext r5, org.jboss.netty.channel.ExceptionEvent r6) throws java.lang.Exception {
        /*
            r4 = this;
            java.lang.Throwable r0 = r6.getCause()
            boolean r1 = r0 instanceof java.io.IOException
            if (r1 == 0) goto L_0x0036
            boolean r1 = r0 instanceof java.nio.channels.ClosedChannelException
            if (r1 == 0) goto L_0x002f
            java.lang.Object r1 = r4.ignoreClosedChannelExceptionLock
            monitor-enter(r1)
            int r2 = r4.ignoreClosedChannelException     // Catch:{ all -> 0x002c }
            if (r2 <= 0) goto L_0x002a
            int r2 = r4.ignoreClosedChannelException     // Catch:{ all -> 0x002c }
            int r2 = r2 + -1
            r4.ignoreClosedChannelException = r2     // Catch:{ all -> 0x002c }
            org.jboss.netty.logging.InternalLogger r2 = logger     // Catch:{ all -> 0x002c }
            boolean r2 = r2.isDebugEnabled()     // Catch:{ all -> 0x002c }
            if (r2 == 0) goto L_0x0028
            org.jboss.netty.logging.InternalLogger r2 = logger     // Catch:{ all -> 0x002c }
            java.lang.String r3 = "Swallowing an exception raised while writing non-app data"
            r2.debug(r3, r0)     // Catch:{ all -> 0x002c }
        L_0x0028:
            monitor-exit(r1)     // Catch:{ all -> 0x002c }
            return
        L_0x002a:
            monitor-exit(r1)     // Catch:{ all -> 0x002c }
            goto L_0x0036
        L_0x002c:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x002c }
            throw r2
        L_0x002f:
            boolean r1 = r4.ignoreException(r0)
            if (r1 == 0) goto L_0x0036
            return
        L_0x0036:
            r5.sendUpstream(r6)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.ssl.SslHandler.exceptionCaught(org.jboss.netty.channel.ChannelHandlerContext, org.jboss.netty.channel.ExceptionEvent):void");
    }

    private boolean ignoreException(Throwable t) {
        if (!(t instanceof SSLException) && (t instanceof IOException) && this.engine.isOutboundDone()) {
            if (IGNORABLE_ERROR_MESSAGE.matcher(String.valueOf(t.getMessage()).toLowerCase()).matches()) {
                return true;
            }
            for (StackTraceElement element : t.getStackTrace()) {
                String classname = element.getClassName();
                String methodname = element.getMethodName();
                if (!classname.startsWith("org.jboss.netty.") && methodname.equals("read")) {
                    if (IGNORABLE_CLASS_IN_STACK.matcher(classname).matches()) {
                        return true;
                    }
                    try {
                        Class<?> clazz = getClass().getClassLoader().loadClass(classname);
                        if (!SocketChannel.class.isAssignableFrom(clazz)) {
                            if (!DatagramChannel.class.isAssignableFrom(clazz)) {
                                if (DetectionUtil.javaVersion() >= 7 && "com.sun.nio.sctp.SctpChannel".equals(clazz.getSuperclass().getName())) {
                                    return true;
                                }
                            }
                        }
                        return true;
                    } catch (ClassNotFoundException e) {
                    }
                }
            }
        }
        return false;
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx2, Channel channel, ChannelBuffer buffer) throws Exception {
        boolean tls;
        if (this.packetLength == -1) {
            if (buffer.readableBytes() < 5) {
                return null;
            }
            switch (buffer.getUnsignedByte(buffer.readerIndex())) {
                case 20:
                case 21:
                case 22:
                case 23:
                    tls = true;
                    break;
                default:
                    tls = false;
                    break;
            }
            if (tls) {
                if (buffer.getUnsignedByte(buffer.readerIndex() + 1) == 3) {
                    this.packetLength = (getShort(buffer, buffer.readerIndex() + 3) & 65535) + 5;
                    if (this.packetLength <= 5) {
                        tls = false;
                    }
                } else {
                    tls = false;
                }
            }
            if (!tls) {
                boolean sslv2 = true;
                int headerLength = (buffer.getUnsignedByte(buffer.readerIndex()) & 128) != 0 ? 2 : 3;
                int majorVersion = buffer.getUnsignedByte(buffer.readerIndex() + headerLength + 1);
                if (majorVersion == 2 || majorVersion == 3) {
                    if (headerLength == 2) {
                        this.packetLength = (getShort(buffer, buffer.readerIndex()) & Short.MAX_VALUE) + 2;
                    } else {
                        this.packetLength = (getShort(buffer, buffer.readerIndex()) & 16383) + 3;
                    }
                    if (this.packetLength <= headerLength) {
                        sslv2 = false;
                    }
                } else {
                    sslv2 = false;
                }
                if (!sslv2) {
                    NotSslRecordException e = new NotSslRecordException("not an SSL/TLS record: " + ChannelBuffers.hexDump(buffer));
                    buffer.skipBytes(buffer.readableBytes());
                    throw e;
                }
            }
        }
        if (buffer.readableBytes() < this.packetLength) {
            return null;
        }
        int packetOffset = buffer.readerIndex();
        buffer.skipBytes(this.packetLength);
        try {
            return unwrap(ctx2, channel, buffer, packetOffset, this.packetLength);
        } finally {
            this.packetLength = -1;
        }
    }

    private static short getShort(ChannelBuffer buf, int offset) {
        return (short) ((buf.getByte(offset) << 8) | (buf.getByte(offset + 1) & NavSatStatus.STATUS_NO_FIX));
    }

    /* JADX WARNING: Code restructure failed: missing block: B:123:0x01a9, code lost:
        r0 = th;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:124:0x01aa, code lost:
        r16 = r11;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private org.jboss.netty.channel.ChannelFuture wrap(org.jboss.netty.channel.ChannelHandlerContext r19, org.jboss.netty.channel.Channel r20) throws javax.net.ssl.SSLException {
        /*
            r18 = this;
            r0 = 0
            r1 = r18
            org.jboss.netty.handler.ssl.SslBufferPool r2 = r1.bufferPool
            java.nio.ByteBuffer r2 = r2.acquireBuffer()
            r3 = 1
            r4 = 0
            r5 = 0
            r6 = r19
            r7 = r0
            r8 = r2
            r9 = r4
            r2 = r1
            r4 = r3
            r3 = r20
        L_0x0015:
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r11 = r2.pendingUnencryptedWrites     // Catch:{ SSLException -> 0x016f }
            monitor-enter(r11)     // Catch:{ SSLException -> 0x016f }
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r0 = r2.pendingUnencryptedWrites     // Catch:{ all -> 0x016a }
            java.lang.Object r0 = r0.peek()     // Catch:{ all -> 0x016a }
            org.jboss.netty.handler.ssl.SslHandler$PendingWrite r0 = (org.jboss.netty.handler.ssl.SslHandler.PendingWrite) r0     // Catch:{ all -> 0x016a }
            r12 = r0
            if (r12 != 0) goto L_0x0028
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
        L_0x0024:
            r0 = r9
            r9 = r3
            goto L_0x00f1
        L_0x0028:
            java.nio.ByteBuffer r0 = r12.outAppBuf     // Catch:{ all -> 0x016a }
            r13 = r0
            if (r13 != 0) goto L_0x0046
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r0 = r2.pendingUnencryptedWrites     // Catch:{ all -> 0x016a }
            r0.remove()     // Catch:{ all -> 0x016a }
            org.jboss.netty.channel.DownstreamMessageEvent r0 = new org.jboss.netty.channel.DownstreamMessageEvent     // Catch:{ all -> 0x016a }
            org.jboss.netty.channel.ChannelFuture r14 = r12.future     // Catch:{ all -> 0x016a }
            org.jboss.netty.buffer.ChannelBuffer r15 = org.jboss.netty.buffer.ChannelBuffers.EMPTY_BUFFER     // Catch:{ all -> 0x016a }
            java.net.SocketAddress r10 = r3.getRemoteAddress()     // Catch:{ all -> 0x016a }
            r0.<init>(r3, r14, r15, r10)     // Catch:{ all -> 0x016a }
            r2.offerEncryptedWriteRequest(r0)     // Catch:{ all -> 0x016a }
            r0 = 1
        L_0x0043:
            r9 = r0
            goto L_0x00eb
        L_0x0046:
            r10 = 0
            r14 = r10
            java.lang.Object r15 = r2.handshakeLock     // Catch:{ all -> 0x015a }
            monitor-enter(r15)     // Catch:{ all -> 0x015a }
            javax.net.ssl.SSLEngine r0 = r2.engine     // Catch:{ all -> 0x0157 }
            javax.net.ssl.SSLEngineResult r0 = r0.wrap(r13, r8)     // Catch:{ all -> 0x0157 }
            r14 = r0
            monitor-exit(r15)     // Catch:{ all -> 0x0157 }
            boolean r0 = r13.hasRemaining()     // Catch:{ all -> 0x016a }
            if (r0 != 0) goto L_0x005f
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r0 = r2.pendingUnencryptedWrites     // Catch:{ all -> 0x016a }
            r0.remove()     // Catch:{ all -> 0x016a }
        L_0x005f:
            int r0 = r14.bytesProduced()     // Catch:{ all -> 0x016a }
            if (r0 <= 0) goto L_0x00a7
            r8.flip()     // Catch:{ all -> 0x016a }
            int r0 = r8.remaining()     // Catch:{ all -> 0x016a }
            org.jboss.netty.channel.ChannelHandlerContext r15 = r2.ctx     // Catch:{ all -> 0x016a }
            org.jboss.netty.channel.Channel r15 = r15.getChannel()     // Catch:{ all -> 0x016a }
            org.jboss.netty.channel.ChannelConfig r15 = r15.getConfig()     // Catch:{ all -> 0x016a }
            org.jboss.netty.buffer.ChannelBufferFactory r15 = r15.getBufferFactory()     // Catch:{ all -> 0x016a }
            org.jboss.netty.buffer.ChannelBuffer r15 = r15.getBuffer((int) r0)     // Catch:{ all -> 0x016a }
            r15.writeBytes((java.nio.ByteBuffer) r8)     // Catch:{ all -> 0x016a }
            r8.clear()     // Catch:{ all -> 0x016a }
            java.nio.ByteBuffer r10 = r12.outAppBuf     // Catch:{ all -> 0x016a }
            boolean r10 = r10.hasRemaining()     // Catch:{ all -> 0x016a }
            if (r10 == 0) goto L_0x0093
            org.jboss.netty.channel.ChannelFuture r10 = org.jboss.netty.channel.Channels.succeededFuture(r3)     // Catch:{ all -> 0x016a }
            r7 = r10
            goto L_0x0096
        L_0x0093:
            org.jboss.netty.channel.ChannelFuture r10 = r12.future     // Catch:{ all -> 0x016a }
            r7 = r10
        L_0x0096:
            org.jboss.netty.channel.DownstreamMessageEvent r10 = new org.jboss.netty.channel.DownstreamMessageEvent     // Catch:{ all -> 0x016a }
            r17 = r0
            java.net.SocketAddress r0 = r3.getRemoteAddress()     // Catch:{ all -> 0x016a }
            r10.<init>(r3, r7, r15, r0)     // Catch:{ all -> 0x016a }
            r0 = r10
            r2.offerEncryptedWriteRequest(r0)     // Catch:{ all -> 0x016a }
            r0 = 1
            goto L_0x0043
        L_0x00a7:
            javax.net.ssl.SSLEngineResult$Status r0 = r14.getStatus()     // Catch:{ all -> 0x016a }
            javax.net.ssl.SSLEngineResult$Status r10 = javax.net.ssl.SSLEngineResult.Status.CLOSED     // Catch:{ all -> 0x016a }
            if (r0 != r10) goto L_0x00b3
            r4 = 0
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
            goto L_0x0024
        L_0x00b3:
            javax.net.ssl.SSLEngineResult$HandshakeStatus r0 = r14.getHandshakeStatus()     // Catch:{ all -> 0x016a }
            r2.handleRenegotiation(r0)     // Catch:{ all -> 0x016a }
            int[] r10 = org.jboss.netty.handler.ssl.SslHandler.AnonymousClass4.$SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus     // Catch:{ all -> 0x016a }
            int r15 = r0.ordinal()     // Catch:{ all -> 0x016a }
            r10 = r10[r15]     // Catch:{ all -> 0x016a }
            switch(r10) {
                case 1: goto L_0x00e4;
                case 2: goto L_0x00e0;
                case 3: goto L_0x00dc;
                case 4: goto L_0x00c9;
                case 5: goto L_0x00c9;
                default: goto L_0x00c5;
            }     // Catch:{ all -> 0x016a }
        L_0x00c5:
            java.lang.IllegalStateException r10 = new java.lang.IllegalStateException     // Catch:{ all -> 0x016a }
            goto L_0x0142
        L_0x00c9:
            javax.net.ssl.SSLEngineResult$HandshakeStatus r10 = javax.net.ssl.SSLEngineResult.HandshakeStatus.FINISHED     // Catch:{ all -> 0x016a }
            if (r0 != r10) goto L_0x00d0
            r2.setHandshakeSuccess(r3)     // Catch:{ all -> 0x016a }
        L_0x00d0:
            javax.net.ssl.SSLEngineResult$Status r10 = r14.getStatus()     // Catch:{ all -> 0x016a }
            javax.net.ssl.SSLEngineResult$Status r15 = javax.net.ssl.SSLEngineResult.Status.CLOSED     // Catch:{ all -> 0x016a }
            if (r10 != r15) goto L_0x00d9
            r4 = 0
        L_0x00d9:
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
            goto L_0x0024
        L_0x00dc:
            r2.runDelegatedTasks()     // Catch:{ all -> 0x016a }
            goto L_0x00eb
        L_0x00e0:
            r5 = 1
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
            goto L_0x0024
        L_0x00e4:
            boolean r10 = r13.hasRemaining()     // Catch:{ all -> 0x016a }
            if (r10 == 0) goto L_0x00ee
        L_0x00eb:
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
            goto L_0x0015
        L_0x00ee:
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
            goto L_0x0024
        L_0x00f1:
            r3 = r6
            r6 = r7
            r13 = r8
            r14 = r0
            org.jboss.netty.handler.ssl.SslBufferPool r0 = r2.bufferPool
            r0.releaseBuffer(r13)
            if (r14 == 0) goto L_0x0100
            r2.flushPendingEncryptedWrites(r3)
        L_0x0100:
            if (r4 != 0) goto L_0x012e
            java.lang.IllegalStateException r0 = new java.lang.IllegalStateException
            java.lang.String r7 = "SSLEngine already closed"
            r0.<init>(r7)
            r16 = 0
        L_0x010b:
            r7 = r0
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r8 = r2.pendingUnencryptedWrites
            monitor-enter(r8)
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r0 = r2.pendingUnencryptedWrites     // Catch:{ all -> 0x012b }
            java.lang.Object r0 = r0.poll()     // Catch:{ all -> 0x012b }
            org.jboss.netty.handler.ssl.SslHandler$PendingWrite r0 = (org.jboss.netty.handler.ssl.SslHandler.PendingWrite) r0     // Catch:{ all -> 0x012b }
            r10 = r0
            if (r10 != 0) goto L_0x0120
            monitor-exit(r8)     // Catch:{ all -> 0x011c }
            goto L_0x012e
        L_0x011c:
            r0 = move-exception
            r16 = r10
            goto L_0x012c
        L_0x0120:
            monitor-exit(r8)     // Catch:{ all -> 0x011c }
            org.jboss.netty.channel.ChannelFuture r0 = r10.future
            r0.setFailure(r7)
            r0 = r7
            r16 = r10
            goto L_0x010b
        L_0x012b:
            r0 = move-exception
        L_0x012c:
            monitor-exit(r8)     // Catch:{ all -> 0x012b }
            throw r0
        L_0x012e:
            if (r5 == 0) goto L_0x013b
            org.jboss.netty.buffer.ChannelBuffer r10 = org.jboss.netty.buffer.ChannelBuffers.EMPTY_BUFFER
            r11 = 0
            r12 = 0
            r7 = r2
            r8 = r3
            r7.unwrap(r8, r9, r10, r11, r12)
        L_0x013b:
            if (r6 != 0) goto L_0x0141
            org.jboss.netty.channel.ChannelFuture r6 = org.jboss.netty.channel.Channels.succeededFuture(r9)
        L_0x0141:
            return r6
        L_0x0142:
            java.lang.StringBuilder r15 = new java.lang.StringBuilder     // Catch:{ all -> 0x016a }
            r15.<init>()     // Catch:{ all -> 0x016a }
            java.lang.String r1 = "Unknown handshake status: "
            r15.append(r1)     // Catch:{ all -> 0x016a }
            r15.append(r0)     // Catch:{ all -> 0x016a }
            java.lang.String r1 = r15.toString()     // Catch:{ all -> 0x016a }
            r10.<init>(r1)     // Catch:{ all -> 0x016a }
            throw r10     // Catch:{ all -> 0x016a }
        L_0x0157:
            r0 = move-exception
            monitor-exit(r15)     // Catch:{ all -> 0x0157 }
            throw r0     // Catch:{ all -> 0x015a }
        L_0x015a:
            r0 = move-exception
            r1 = r14
            r10 = r12
            r12 = r13
            boolean r13 = r12.hasRemaining()     // Catch:{ all -> 0x016a }
            if (r13 != 0) goto L_0x0169
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r13 = r2.pendingUnencryptedWrites     // Catch:{ all -> 0x016a }
            r13.remove()     // Catch:{ all -> 0x016a }
        L_0x0169:
            throw r0     // Catch:{ all -> 0x016a }
        L_0x016a:
            r0 = move-exception
            monitor-exit(r11)     // Catch:{ all -> 0x016a }
            throw r0     // Catch:{ SSLException -> 0x016f }
        L_0x016d:
            r0 = move-exception
            goto L_0x0175
        L_0x016f:
            r0 = move-exception
            r4 = 0
            r2.setHandshakeFailure(r3, r0)     // Catch:{ all -> 0x016d }
            throw r0     // Catch:{ all -> 0x016d }
        L_0x0175:
            r1 = r2
            r2 = r6
            r6 = r7
            r7 = r8
            r8 = r9
            org.jboss.netty.handler.ssl.SslBufferPool r9 = r1.bufferPool
            r9.releaseBuffer(r7)
            if (r8 == 0) goto L_0x0185
            r1.flushPendingEncryptedWrites(r2)
        L_0x0185:
            if (r4 != 0) goto L_0x01b0
            java.lang.IllegalStateException r9 = new java.lang.IllegalStateException
            java.lang.String r10 = "SSLEngine already closed"
            r9.<init>(r10)
            r16 = 0
        L_0x0190:
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r10 = r1.pendingUnencryptedWrites
            monitor-enter(r10)
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r11 = r1.pendingUnencryptedWrites     // Catch:{ all -> 0x01ad }
            java.lang.Object r11 = r11.poll()     // Catch:{ all -> 0x01ad }
            org.jboss.netty.handler.ssl.SslHandler$PendingWrite r11 = (org.jboss.netty.handler.ssl.SslHandler.PendingWrite) r11     // Catch:{ all -> 0x01ad }
            if (r11 == 0) goto L_0x01a7
            monitor-exit(r10)     // Catch:{ all -> 0x01a9 }
            org.jboss.netty.channel.ChannelFuture r10 = r11.future
            r10.setFailure(r9)
            r16 = r11
            goto L_0x0190
        L_0x01a7:
            monitor-exit(r10)     // Catch:{ all -> 0x01a9 }
            goto L_0x01b0
        L_0x01a9:
            r0 = move-exception
            r16 = r11
            goto L_0x01ae
        L_0x01ad:
            r0 = move-exception
        L_0x01ae:
            monitor-exit(r10)     // Catch:{ all -> 0x01ad }
            throw r0
        L_0x01b0:
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.ssl.SslHandler.wrap(org.jboss.netty.channel.ChannelHandlerContext, org.jboss.netty.channel.Channel):org.jboss.netty.channel.ChannelFuture");
    }

    /* renamed from: org.jboss.netty.handler.ssl.SslHandler$4  reason: invalid class name */
    static /* synthetic */ class AnonymousClass4 {
        static final /* synthetic */ int[] $SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus = new int[SSLEngineResult.HandshakeStatus.values().length];

        static {
            try {
                $SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[SSLEngineResult.HandshakeStatus.NEED_WRAP.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[SSLEngineResult.HandshakeStatus.NEED_UNWRAP.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
            try {
                $SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[SSLEngineResult.HandshakeStatus.NEED_TASK.ordinal()] = 3;
            } catch (NoSuchFieldError e3) {
            }
            try {
                $SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[SSLEngineResult.HandshakeStatus.FINISHED.ordinal()] = 4;
            } catch (NoSuchFieldError e4) {
            }
            try {
                $SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[SSLEngineResult.HandshakeStatus.NOT_HANDSHAKING.ordinal()] = 5;
            } catch (NoSuchFieldError e5) {
            }
            $SwitchMap$org$jboss$netty$channel$ChannelState = new int[ChannelState.values().length];
            try {
                $SwitchMap$org$jboss$netty$channel$ChannelState[ChannelState.OPEN.ordinal()] = 1;
            } catch (NoSuchFieldError e6) {
            }
            try {
                $SwitchMap$org$jboss$netty$channel$ChannelState[ChannelState.CONNECTED.ordinal()] = 2;
            } catch (NoSuchFieldError e7) {
            }
            try {
                $SwitchMap$org$jboss$netty$channel$ChannelState[ChannelState.BOUND.ordinal()] = 3;
            } catch (NoSuchFieldError e8) {
            }
        }
    }

    private void offerEncryptedWriteRequest(MessageEvent encryptedWrite) {
        boolean locked = this.pendingEncryptedWritesLock.tryLock();
        try {
            this.pendingEncryptedWrites.offer(encryptedWrite);
        } finally {
            if (locked) {
                this.pendingEncryptedWritesLock.unlock();
            }
        }
    }

    private void flushPendingEncryptedWrites(ChannelHandlerContext ctx2) {
        if (this.pendingEncryptedWritesLock.tryLock()) {
            while (true) {
                try {
                    MessageEvent poll = this.pendingEncryptedWrites.poll();
                    MessageEvent e = poll;
                    if (poll != null) {
                        ctx2.sendDownstream(e);
                    } else {
                        this.pendingEncryptedWritesLock.unlock();
                        return;
                    }
                } catch (Throwable th) {
                    this.pendingEncryptedWritesLock.unlock();
                    throw th;
                }
            }
        }
    }

    private ChannelFuture wrapNonAppData(ChannelHandlerContext ctx2, Channel channel) throws SSLException {
        SSLEngineResult result;
        ChannelFuture future = null;
        ByteBuffer outNetBuf = this.bufferPool.acquireBuffer();
        do {
            try {
                synchronized (this.handshakeLock) {
                    result = this.engine.wrap(EMPTY_BUFFER, outNetBuf);
                }
                if (result.bytesProduced() > 0) {
                    outNetBuf.flip();
                    ChannelBuffer msg = ctx2.getChannel().getConfig().getBufferFactory().getBuffer(outNetBuf.remaining());
                    msg.writeBytes(outNetBuf);
                    outNetBuf.clear();
                    future = Channels.future(channel);
                    future.addListener(new ChannelFutureListener() {
                        public void operationComplete(ChannelFuture future) throws Exception {
                            if (future.getCause() instanceof ClosedChannelException) {
                                synchronized (SslHandler.this.ignoreClosedChannelExceptionLock) {
                                    SslHandler.this.ignoreClosedChannelException++;
                                }
                            }
                        }
                    });
                    Channels.write(ctx2, future, (Object) msg);
                }
                SSLEngineResult.HandshakeStatus handshakeStatus = result.getHandshakeStatus();
                handleRenegotiation(handshakeStatus);
                switch (AnonymousClass4.$SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[handshakeStatus.ordinal()]) {
                    case 1:
                    case 5:
                        break;
                    case 2:
                        if (!Thread.holdsLock(this.handshakeLock)) {
                            unwrap(ctx2, channel, ChannelBuffers.EMPTY_BUFFER, 0, 0);
                            break;
                        }
                        break;
                    case 3:
                        runDelegatedTasks();
                        break;
                    case 4:
                        setHandshakeSuccess(channel);
                        runDelegatedTasks();
                        break;
                    default:
                        throw new IllegalStateException("Unexpected handshake status: " + handshakeStatus);
                }
            } catch (SSLException e) {
                try {
                    setHandshakeFailure(channel, e);
                    throw e;
                } catch (Throwable th) {
                    this.bufferPool.releaseBuffer(outNetBuf);
                    throw th;
                }
            }
        } while (result.bytesProduced() != 0);
        this.bufferPool.releaseBuffer(outNetBuf);
        if (future == null) {
            return Channels.succeededFuture(channel);
        }
        return future;
    }

    private ChannelBuffer unwrap(ChannelHandlerContext ctx2, Channel channel, ChannelBuffer buffer, int offset, int length) throws SSLException {
        SSLEngineResult result;
        ByteBuffer inNetBuf = buffer.toByteBuffer(offset, length);
        ByteBuffer outAppBuf = this.bufferPool.acquireBuffer();
        boolean needsWrap = false;
        while (true) {
            boolean needsHandshake = false;
            try {
                synchronized (this.handshakeLock) {
                    if (!this.handshaken && !this.handshaking && !this.engine.getUseClientMode() && !this.engine.isInboundDone() && !this.engine.isOutboundDone()) {
                        needsHandshake = true;
                    }
                }
                if (needsHandshake) {
                    handshake();
                }
                synchronized (this.handshakeLock) {
                    result = this.engine.unwrap(inNetBuf, outAppBuf);
                }
                if (result.getStatus() == SSLEngineResult.Status.CLOSED) {
                    this.sslEngineCloseFuture.setClosed();
                }
                SSLEngineResult.HandshakeStatus handshakeStatus = result.getHandshakeStatus();
                handleRenegotiation(handshakeStatus);
                switch (AnonymousClass4.$SwitchMap$javax$net$ssl$SSLEngineResult$HandshakeStatus[handshakeStatus.ordinal()]) {
                    case 1:
                        wrapNonAppData(ctx2, channel);
                        continue;
                    case 2:
                        if (inNetBuf.hasRemaining() && !this.engine.isInboundDone()) {
                            continue;
                        }
                    case 3:
                        runDelegatedTasks();
                        continue;
                    case 4:
                        setHandshakeSuccess(channel);
                        needsWrap = true;
                        break;
                    case 5:
                        needsWrap = true;
                        break;
                    default:
                        throw new IllegalStateException("Unknown handshake status: " + handshakeStatus);
                }
            } catch (SSLException e) {
                try {
                    setHandshakeFailure(channel, e);
                    throw e;
                } catch (Throwable th) {
                    this.bufferPool.releaseBuffer(outAppBuf);
                    throw th;
                }
            }
        }
        if (needsWrap && !Thread.holdsLock(this.handshakeLock) && !this.pendingEncryptedWritesLock.isHeldByCurrentThread()) {
            wrap(ctx2, channel);
        }
        outAppBuf.flip();
        if (outAppBuf.hasRemaining()) {
            ChannelBuffer frame = ctx2.getChannel().getConfig().getBufferFactory().getBuffer(outAppBuf.remaining());
            frame.writeBytes(outAppBuf);
            this.bufferPool.releaseBuffer(outAppBuf);
            return frame;
        }
        this.bufferPool.releaseBuffer(outAppBuf);
        return null;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:25:0x0036, code lost:
        if (r1 == false) goto L_0x003c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:26:0x0038, code lost:
        handshake();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:27:0x003c, code lost:
        org.jboss.netty.channel.Channels.fireExceptionCaught(r4.ctx, (java.lang.Throwable) new javax.net.ssl.SSLException("renegotiation attempted by peer; closing the connection"));
        org.jboss.netty.channel.Channels.close(r4.ctx, org.jboss.netty.channel.Channels.succeededFuture(r4.ctx.getChannel()));
     */
    /* JADX WARNING: Code restructure failed: missing block: B:34:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:35:?, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void handleRenegotiation(javax.net.ssl.SSLEngineResult.HandshakeStatus r5) {
        /*
            r4 = this;
            javax.net.ssl.SSLEngineResult$HandshakeStatus r0 = javax.net.ssl.SSLEngineResult.HandshakeStatus.NOT_HANDSHAKING
            if (r5 == r0) goto L_0x005d
            javax.net.ssl.SSLEngineResult$HandshakeStatus r0 = javax.net.ssl.SSLEngineResult.HandshakeStatus.FINISHED
            if (r5 != r0) goto L_0x0009
            goto L_0x005d
        L_0x0009:
            boolean r0 = r4.handshaken
            if (r0 != 0) goto L_0x000e
            return
        L_0x000e:
            java.lang.Object r0 = r4.handshakeLock
            monitor-enter(r0)
            r1 = 0
            boolean r2 = r4.handshaking     // Catch:{ all -> 0x005a }
            if (r2 == 0) goto L_0x0018
            monitor-exit(r0)     // Catch:{ all -> 0x005a }
            return
        L_0x0018:
            javax.net.ssl.SSLEngine r2 = r4.engine     // Catch:{ all -> 0x005a }
            boolean r2 = r2.isInboundDone()     // Catch:{ all -> 0x005a }
            if (r2 != 0) goto L_0x0058
            javax.net.ssl.SSLEngine r2 = r4.engine     // Catch:{ all -> 0x005a }
            boolean r2 = r2.isOutboundDone()     // Catch:{ all -> 0x005a }
            if (r2 == 0) goto L_0x0029
            goto L_0x0058
        L_0x0029:
            boolean r2 = r4.isEnableRenegotiation()     // Catch:{ all -> 0x005a }
            if (r2 == 0) goto L_0x0031
            r1 = 1
            goto L_0x0035
        L_0x0031:
            r1 = 0
            r2 = 1
            r4.handshaking = r2     // Catch:{ all -> 0x005a }
        L_0x0035:
            monitor-exit(r0)     // Catch:{ all -> 0x005a }
            if (r1 == 0) goto L_0x003c
            r4.handshake()
            goto L_0x0057
        L_0x003c:
            org.jboss.netty.channel.ChannelHandlerContext r0 = r4.ctx
            javax.net.ssl.SSLException r2 = new javax.net.ssl.SSLException
            java.lang.String r3 = "renegotiation attempted by peer; closing the connection"
            r2.<init>(r3)
            org.jboss.netty.channel.Channels.fireExceptionCaught((org.jboss.netty.channel.ChannelHandlerContext) r0, (java.lang.Throwable) r2)
            org.jboss.netty.channel.ChannelHandlerContext r0 = r4.ctx
            org.jboss.netty.channel.ChannelHandlerContext r2 = r4.ctx
            org.jboss.netty.channel.Channel r2 = r2.getChannel()
            org.jboss.netty.channel.ChannelFuture r2 = org.jboss.netty.channel.Channels.succeededFuture(r2)
            org.jboss.netty.channel.Channels.close(r0, r2)
        L_0x0057:
            return
        L_0x0058:
            monitor-exit(r0)     // Catch:{ all -> 0x005a }
            return
        L_0x005a:
            r2 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x005a }
            throw r2
        L_0x005d:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.ssl.SslHandler.handleRenegotiation(javax.net.ssl.SSLEngineResult$HandshakeStatus):void");
    }

    private void runDelegatedTasks() {
        final Runnable task;
        while (true) {
            synchronized (this.handshakeLock) {
                task = this.engine.getDelegatedTask();
            }
            if (task != null) {
                this.delegatedTaskExecutor.execute(new Runnable() {
                    public void run() {
                        synchronized (SslHandler.this.handshakeLock) {
                            task.run();
                        }
                    }
                });
            } else {
                return;
            }
        }
        while (true) {
        }
    }

    private void setHandshakeSuccess(Channel channel) {
        synchronized (this.handshakeLock) {
            this.handshaking = false;
            this.handshaken = true;
            if (this.handshakeFuture == null) {
                this.handshakeFuture = Channels.future(channel);
            }
        }
        this.handshakeFuture.setSuccess();
    }

    /* JADX WARNING: Code restructure failed: missing block: B:19:0x0034, code lost:
        r4.handshakeFuture.setFailure(r6);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:20:0x0039, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void setHandshakeFailure(org.jboss.netty.channel.Channel r5, javax.net.ssl.SSLException r6) {
        /*
            r4 = this;
            java.lang.Object r0 = r4.handshakeLock
            monitor-enter(r0)
            boolean r1 = r4.handshaking     // Catch:{ all -> 0x003a }
            if (r1 != 0) goto L_0x0009
            monitor-exit(r0)     // Catch:{ all -> 0x003a }
            return
        L_0x0009:
            r1 = 0
            r4.handshaking = r1     // Catch:{ all -> 0x003a }
            r4.handshaken = r1     // Catch:{ all -> 0x003a }
            org.jboss.netty.channel.ChannelFuture r1 = r4.handshakeFuture     // Catch:{ all -> 0x003a }
            if (r1 != 0) goto L_0x0018
            org.jboss.netty.channel.ChannelFuture r1 = org.jboss.netty.channel.Channels.future(r5)     // Catch:{ all -> 0x003a }
            r4.handshakeFuture = r1     // Catch:{ all -> 0x003a }
        L_0x0018:
            javax.net.ssl.SSLEngine r1 = r4.engine     // Catch:{ all -> 0x003a }
            r1.closeOutbound()     // Catch:{ all -> 0x003a }
            javax.net.ssl.SSLEngine r1 = r4.engine     // Catch:{ SSLException -> 0x0023 }
            r1.closeInbound()     // Catch:{ SSLException -> 0x0023 }
            goto L_0x0033
        L_0x0023:
            r1 = move-exception
            org.jboss.netty.logging.InternalLogger r2 = logger     // Catch:{ all -> 0x003a }
            boolean r2 = r2.isDebugEnabled()     // Catch:{ all -> 0x003a }
            if (r2 == 0) goto L_0x0033
            org.jboss.netty.logging.InternalLogger r2 = logger     // Catch:{ all -> 0x003a }
            java.lang.String r3 = "SSLEngine.closeInbound() raised an exception after a handshake failure."
            r2.debug(r3, r1)     // Catch:{ all -> 0x003a }
        L_0x0033:
            monitor-exit(r0)     // Catch:{ all -> 0x003a }
            org.jboss.netty.channel.ChannelFuture r0 = r4.handshakeFuture
            r0.setFailure(r6)
            return
        L_0x003a:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x003a }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.ssl.SslHandler.setHandshakeFailure(org.jboss.netty.channel.Channel, javax.net.ssl.SSLException):void");
    }

    private void closeOutboundAndChannel(ChannelHandlerContext context, ChannelStateEvent e) {
        if (!e.getChannel().isConnected()) {
            context.sendDownstream(e);
            return;
        }
        boolean success = false;
        try {
            unwrap(context, e.getChannel(), ChannelBuffers.EMPTY_BUFFER, 0, 0);
        } catch (SSLException ex) {
            if (logger.isDebugEnabled()) {
                logger.debug("Failed to unwrap before sending a close_notify message", ex);
            }
        } catch (Throwable th) {
            if (0 == 0) {
                context.sendDownstream(e);
            }
            throw th;
        }
        if (this.engine.isInboundDone()) {
            success = true;
        } else if (this.sentCloseNotify.compareAndSet(false, true)) {
            this.engine.closeOutbound();
            try {
                wrapNonAppData(context, e.getChannel()).addListener(new ClosingChannelFutureListener(context, e));
                success = true;
            } catch (SSLException ex2) {
                if (logger.isDebugEnabled()) {
                    logger.debug("Failed to encode a close_notify message", ex2);
                }
            }
        }
        if (!success) {
            context.sendDownstream(e);
        }
    }

    private static final class PendingWrite {
        final ChannelFuture future;
        final ByteBuffer outAppBuf;

        PendingWrite(ChannelFuture future2, ByteBuffer outAppBuf2) {
            this.future = future2;
            this.outAppBuf = outAppBuf2;
        }
    }

    private static final class ClosingChannelFutureListener implements ChannelFutureListener {
        private final ChannelHandlerContext context;
        private final ChannelStateEvent e;

        ClosingChannelFutureListener(ChannelHandlerContext context2, ChannelStateEvent e2) {
            this.context = context2;
            this.e = e2;
        }

        public void operationComplete(ChannelFuture closeNotifyFuture) throws Exception {
            if (!(closeNotifyFuture.getCause() instanceof ClosedChannelException)) {
                Channels.close(this.context, this.e.getFuture());
            } else {
                this.e.getFuture().setSuccess();
            }
        }
    }

    public void beforeAdd(ChannelHandlerContext ctx2) throws Exception {
        super.beforeAdd(ctx2);
        this.ctx = ctx2;
    }

    public void afterRemove(ChannelHandlerContext ctx2) throws Exception {
        Throwable cause = null;
        while (true) {
            PendingWrite pw = this.pendingUnencryptedWrites.poll();
            if (pw == null) {
                break;
            }
            if (cause == null) {
                cause = new IOException("Unable to write data");
            }
            pw.future.setFailure(cause);
        }
        while (true) {
            MessageEvent ev = this.pendingEncryptedWrites.poll();
            if (ev == null) {
                break;
            }
            if (cause == null) {
                cause = new IOException("Unable to write data");
            }
            ev.getFuture().setFailure(cause);
        }
        if (cause != null) {
            Channels.fireExceptionCaughtLater(ctx2, cause);
        }
    }

    public void channelConnected(final ChannelHandlerContext ctx2, final ChannelStateEvent e) throws Exception {
        if (this.issueHandshake) {
            handshake().addListener(new ChannelFutureListener() {
                public void operationComplete(ChannelFuture future) throws Exception {
                    if (!future.isSuccess()) {
                        Channels.fireExceptionCaught(future.getChannel(), future.getCause());
                    } else {
                        ctx2.sendUpstream(e);
                    }
                }
            });
        } else {
            super.channelConnected(ctx2, e);
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:12:0x0024, code lost:
        if (r0 != null) goto L_0x002c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:14:?, code lost:
        r0 = new java.nio.channels.ClosedChannelException();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:15:0x002c, code lost:
        r2.getFuture().setFailure(r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:5:0x000f, code lost:
        r2 = r4.pendingEncryptedWrites.poll();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:6:0x0017, code lost:
        if (r2 != null) goto L_0x0024;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void channelClosed(org.jboss.netty.channel.ChannelHandlerContext r5, org.jboss.netty.channel.ChannelStateEvent r6) throws java.lang.Exception {
        /*
            r4 = this;
            r0 = 0
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r1 = r4.pendingUnencryptedWrites
            monitor-enter(r1)
        L_0x0004:
            java.util.Queue<org.jboss.netty.handler.ssl.SslHandler$PendingWrite> r2 = r4.pendingUnencryptedWrites     // Catch:{ all -> 0x0042 }
            java.lang.Object r2 = r2.poll()     // Catch:{ all -> 0x0042 }
            org.jboss.netty.handler.ssl.SslHandler$PendingWrite r2 = (org.jboss.netty.handler.ssl.SslHandler.PendingWrite) r2     // Catch:{ all -> 0x0042 }
            if (r2 != 0) goto L_0x0034
        L_0x000f:
            java.util.Queue<org.jboss.netty.channel.MessageEvent> r2 = r4.pendingEncryptedWrites     // Catch:{ all -> 0x0042 }
            java.lang.Object r2 = r2.poll()     // Catch:{ all -> 0x0042 }
            org.jboss.netty.channel.MessageEvent r2 = (org.jboss.netty.channel.MessageEvent) r2     // Catch:{ all -> 0x0042 }
            if (r2 != 0) goto L_0x0024
            monitor-exit(r1)     // Catch:{ all -> 0x0042 }
            if (r0 == 0) goto L_0x0020
            org.jboss.netty.channel.Channels.fireExceptionCaught((org.jboss.netty.channel.ChannelHandlerContext) r5, (java.lang.Throwable) r0)
        L_0x0020:
            super.channelClosed(r5, r6)
            return
        L_0x0024:
            if (r0 != 0) goto L_0x002c
            java.nio.channels.ClosedChannelException r3 = new java.nio.channels.ClosedChannelException     // Catch:{ all -> 0x0042 }
            r3.<init>()     // Catch:{ all -> 0x0042 }
            r0 = r3
        L_0x002c:
            org.jboss.netty.channel.ChannelFuture r3 = r2.getFuture()     // Catch:{ all -> 0x0042 }
            r3.setFailure(r0)     // Catch:{ all -> 0x0042 }
            goto L_0x000f
        L_0x0034:
            if (r0 != 0) goto L_0x003c
            java.nio.channels.ClosedChannelException r3 = new java.nio.channels.ClosedChannelException     // Catch:{ all -> 0x0042 }
            r3.<init>()     // Catch:{ all -> 0x0042 }
            r0 = r3
        L_0x003c:
            org.jboss.netty.channel.ChannelFuture r3 = r2.future     // Catch:{ all -> 0x0042 }
            r3.setFailure(r0)     // Catch:{ all -> 0x0042 }
            goto L_0x0004
        L_0x0042:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x0042 }
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.ssl.SslHandler.channelClosed(org.jboss.netty.channel.ChannelHandlerContext, org.jboss.netty.channel.ChannelStateEvent):void");
    }

    private final class SSLEngineInboundCloseFuture extends DefaultChannelFuture {
        public SSLEngineInboundCloseFuture() {
            super((Channel) null, true);
        }

        /* access modifiers changed from: package-private */
        public void setClosed() {
            super.setSuccess();
        }

        public Channel getChannel() {
            if (SslHandler.this.ctx == null) {
                return null;
            }
            return SslHandler.this.ctx.getChannel();
        }

        public boolean setSuccess() {
            return false;
        }

        public boolean setFailure(Throwable cause) {
            return false;
        }
    }
}
