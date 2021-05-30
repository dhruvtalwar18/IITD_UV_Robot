package org.jboss.netty.handler.codec.compression;

import java.util.concurrent.atomic.AtomicBoolean;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.LifeCycleAwareChannelHandler;
import org.jboss.netty.handler.codec.oneone.OneToOneEncoder;
import org.jboss.netty.util.internal.jzlib.JZlib;
import org.jboss.netty.util.internal.jzlib.ZStream;

public class ZlibEncoder extends OneToOneEncoder implements LifeCycleAwareChannelHandler {
    private static final byte[] EMPTY_ARRAY = new byte[0];
    private volatile ChannelHandlerContext ctx;
    private final AtomicBoolean finished;
    private final ZStream z;

    public ZlibEncoder() {
        this(6);
    }

    public ZlibEncoder(int compressionLevel) {
        this(ZlibWrapper.ZLIB, compressionLevel);
    }

    public ZlibEncoder(ZlibWrapper wrapper) {
        this(wrapper, 6);
    }

    public ZlibEncoder(ZlibWrapper wrapper, int compressionLevel) {
        this(wrapper, compressionLevel, 15, 8);
    }

    public ZlibEncoder(ZlibWrapper wrapper, int compressionLevel, int windowBits, int memLevel) {
        this.z = new ZStream();
        this.finished = new AtomicBoolean();
        if (compressionLevel < 0 || compressionLevel > 9) {
            throw new IllegalArgumentException("compressionLevel: " + compressionLevel + " (expected: 0-9)");
        } else if (windowBits < 9 || windowBits > 15) {
            throw new IllegalArgumentException("windowBits: " + windowBits + " (expected: 9-15)");
        } else if (memLevel < 1 || memLevel > 9) {
            throw new IllegalArgumentException("memLevel: " + memLevel + " (expected: 1-9)");
        } else if (wrapper == null) {
            throw new NullPointerException("wrapper");
        } else if (wrapper != ZlibWrapper.ZLIB_OR_NONE) {
            synchronized (this.z) {
                int resultCode = this.z.deflateInit(compressionLevel, windowBits, memLevel, ZlibUtil.convertWrapperType(wrapper));
                if (resultCode != 0) {
                    ZlibUtil.fail(this.z, "initialization failure", resultCode);
                }
            }
        } else {
            throw new IllegalArgumentException("wrapper '" + ZlibWrapper.ZLIB_OR_NONE + "' is not " + "allowed for compression.");
        }
    }

    public ZlibEncoder(byte[] dictionary) {
        this(6, dictionary);
    }

    public ZlibEncoder(int compressionLevel, byte[] dictionary) {
        this(compressionLevel, 15, 8, dictionary);
    }

    public ZlibEncoder(int compressionLevel, int windowBits, int memLevel, byte[] dictionary) {
        this.z = new ZStream();
        this.finished = new AtomicBoolean();
        if (compressionLevel < 0 || compressionLevel > 9) {
            throw new IllegalArgumentException("compressionLevel: " + compressionLevel + " (expected: 0-9)");
        } else if (windowBits < 9 || windowBits > 15) {
            throw new IllegalArgumentException("windowBits: " + windowBits + " (expected: 9-15)");
        } else if (memLevel < 1 || memLevel > 9) {
            throw new IllegalArgumentException("memLevel: " + memLevel + " (expected: 1-9)");
        } else if (dictionary != null) {
            synchronized (this.z) {
                int resultCode = this.z.deflateInit(compressionLevel, windowBits, memLevel, JZlib.W_ZLIB);
                if (resultCode != 0) {
                    ZlibUtil.fail(this.z, "initialization failure", resultCode);
                } else {
                    int resultCode2 = this.z.deflateSetDictionary(dictionary, dictionary.length);
                    if (resultCode2 != 0) {
                        ZlibUtil.fail(this.z, "failed to set the dictionary", resultCode2);
                    }
                }
            }
        } else {
            throw new NullPointerException("dictionary");
        }
    }

    public ChannelFuture close() {
        ChannelHandlerContext ctx2 = this.ctx;
        if (ctx2 != null) {
            return finishEncode(ctx2, (ChannelEvent) null);
        }
        throw new IllegalStateException("not added to a pipeline");
    }

    public boolean isClosed() {
        return this.finished.get();
    }

    /* access modifiers changed from: protected */
    public Object encode(ChannelHandlerContext ctx2, Channel channel, Object msg) throws Exception {
        ChannelBuffer result;
        if (!(msg instanceof ChannelBuffer) || this.finished.get()) {
            return msg;
        }
        synchronized (this.z) {
            try {
                ChannelBuffer uncompressed = (ChannelBuffer) msg;
                byte[] in = new byte[uncompressed.readableBytes()];
                uncompressed.readBytes(in);
                this.z.next_in = in;
                this.z.next_in_index = 0;
                this.z.avail_in = in.length;
                double length = (double) in.length;
                Double.isNaN(length);
                byte[] out = new byte[(((int) Math.ceil(length * 1.001d)) + 12)];
                this.z.next_out = out;
                this.z.next_out_index = 0;
                this.z.avail_out = out.length;
                int resultCode = this.z.deflate(2);
                if (resultCode != 0) {
                    ZlibUtil.fail(this.z, "compression failure", resultCode);
                }
                if (this.z.next_out_index != 0) {
                    result = ctx2.getChannel().getConfig().getBufferFactory().getBuffer(uncompressed.order(), out, 0, this.z.next_out_index);
                } else {
                    result = ChannelBuffers.EMPTY_BUFFER;
                }
                ChannelBuffer uncompressed2 = result;
                try {
                    this.z.next_in = null;
                    this.z.next_out = null;
                    return uncompressed2;
                } catch (Throwable th) {
                    th = th;
                    throw th;
                }
            } catch (Throwable th2) {
                th = th2;
                throw th;
            }
        }
    }

    public void handleDownstream(ChannelHandlerContext ctx2, ChannelEvent evt) throws Exception {
        if (evt instanceof ChannelStateEvent) {
            ChannelStateEvent e = (ChannelStateEvent) evt;
            switch (e.getState()) {
                case OPEN:
                case CONNECTED:
                case BOUND:
                    if (Boolean.FALSE.equals(e.getValue()) || e.getValue() == null) {
                        finishEncode(ctx2, evt);
                        return;
                    }
            }
        }
        super.handleDownstream(ctx2, evt);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:28:0x0099, code lost:
        if (r2 == null) goto L_0x009e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x009b, code lost:
        org.jboss.netty.channel.Channels.write(r10, r1, (java.lang.Object) r2);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:30:0x009e, code lost:
        if (r11 == null) goto L_0x00a8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:31:0x00a0, code lost:
        r1.addListener(new org.jboss.netty.handler.codec.compression.ZlibEncoder.AnonymousClass1(r9));
     */
    /* JADX WARNING: Code restructure failed: missing block: B:32:0x00a8, code lost:
        return r1;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private org.jboss.netty.channel.ChannelFuture finishEncode(final org.jboss.netty.channel.ChannelHandlerContext r10, final org.jboss.netty.channel.ChannelEvent r11) {
        /*
            r9 = this;
            java.util.concurrent.atomic.AtomicBoolean r0 = r9.finished
            r1 = 1
            r2 = 0
            boolean r0 = r0.compareAndSet(r2, r1)
            if (r0 != 0) goto L_0x0018
            if (r11 == 0) goto L_0x000f
            r10.sendDownstream(r11)
        L_0x000f:
            org.jboss.netty.channel.Channel r0 = r10.getChannel()
            org.jboss.netty.channel.ChannelFuture r0 = org.jboss.netty.channel.Channels.succeededFuture(r0)
            return r0
        L_0x0018:
            org.jboss.netty.util.internal.jzlib.ZStream r0 = r9.z
            monitor-enter(r0)
            r3 = 0
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00af }
            byte[] r5 = EMPTY_ARRAY     // Catch:{ all -> 0x00af }
            r4.next_in = r5     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00af }
            r4.next_in_index = r2     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00af }
            r4.avail_in = r2     // Catch:{ all -> 0x00af }
            r4 = 32
            byte[] r4 = new byte[r4]     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r5 = r9.z     // Catch:{ all -> 0x00af }
            r5.next_out = r4     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r5 = r9.z     // Catch:{ all -> 0x00af }
            r5.next_out_index = r2     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r5 = r9.z     // Catch:{ all -> 0x00af }
            int r6 = r4.length     // Catch:{ all -> 0x00af }
            r5.avail_out = r6     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r5 = r9.z     // Catch:{ all -> 0x00af }
            r6 = 4
            int r5 = r5.deflate(r6)     // Catch:{ all -> 0x00af }
            if (r5 == 0) goto L_0x0058
            if (r5 == r1) goto L_0x0058
            org.jboss.netty.channel.Channel r1 = r10.getChannel()     // Catch:{ all -> 0x00af }
            org.jboss.netty.util.internal.jzlib.ZStream r2 = r9.z     // Catch:{ all -> 0x00af }
            java.lang.String r6 = "compression failure"
            org.jboss.netty.handler.codec.compression.CompressionException r2 = org.jboss.netty.handler.codec.compression.ZlibUtil.exception(r2, r6, r5)     // Catch:{ all -> 0x00af }
            org.jboss.netty.channel.ChannelFuture r1 = org.jboss.netty.channel.Channels.failedFuture(r1, r2)     // Catch:{ all -> 0x00af }
            r2 = 0
            goto L_0x008a
        L_0x0058:
            org.jboss.netty.util.internal.jzlib.ZStream r1 = r9.z     // Catch:{ all -> 0x00af }
            int r1 = r1.next_out_index     // Catch:{ all -> 0x00af }
            if (r1 == 0) goto L_0x0080
            org.jboss.netty.channel.Channel r1 = r10.getChannel()     // Catch:{ all -> 0x00af }
            org.jboss.netty.channel.ChannelFuture r1 = org.jboss.netty.channel.Channels.future(r1)     // Catch:{ all -> 0x00af }
            org.jboss.netty.channel.Channel r6 = r10.getChannel()     // Catch:{ all -> 0x007b }
            org.jboss.netty.channel.ChannelConfig r6 = r6.getConfig()     // Catch:{ all -> 0x007b }
            org.jboss.netty.buffer.ChannelBufferFactory r6 = r6.getBufferFactory()     // Catch:{ all -> 0x007b }
            org.jboss.netty.util.internal.jzlib.ZStream r7 = r9.z     // Catch:{ all -> 0x007b }
            int r7 = r7.next_out_index     // Catch:{ all -> 0x007b }
            org.jboss.netty.buffer.ChannelBuffer r2 = r6.getBuffer(r4, r2, r7)     // Catch:{ all -> 0x007b }
            goto L_0x008a
        L_0x007b:
            r2 = move-exception
            r8 = r2
            r2 = r1
            r1 = r8
            goto L_0x00b1
        L_0x0080:
            org.jboss.netty.channel.Channel r1 = r10.getChannel()     // Catch:{ all -> 0x00af }
            org.jboss.netty.channel.ChannelFuture r1 = org.jboss.netty.channel.Channels.future(r1)     // Catch:{ all -> 0x00af }
            org.jboss.netty.buffer.ChannelBuffer r2 = org.jboss.netty.buffer.ChannelBuffers.EMPTY_BUFFER     // Catch:{ all -> 0x007b }
        L_0x008a:
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00a9 }
            r4.deflateEnd()     // Catch:{ all -> 0x00a9 }
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00a9 }
            r4.next_in = r3     // Catch:{ all -> 0x00a9 }
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00a9 }
            r4.next_out = r3     // Catch:{ all -> 0x00a9 }
            monitor-exit(r0)     // Catch:{ all -> 0x00a9 }
            if (r2 == 0) goto L_0x009e
            org.jboss.netty.channel.Channels.write((org.jboss.netty.channel.ChannelHandlerContext) r10, (org.jboss.netty.channel.ChannelFuture) r1, (java.lang.Object) r2)
        L_0x009e:
            if (r11 == 0) goto L_0x00a8
            org.jboss.netty.handler.codec.compression.ZlibEncoder$1 r0 = new org.jboss.netty.handler.codec.compression.ZlibEncoder$1
            r0.<init>(r10, r11)
            r1.addListener(r0)
        L_0x00a8:
            return r1
        L_0x00a9:
            r3 = move-exception
            r8 = r2
            r2 = r1
            r1 = r3
            r3 = r8
            goto L_0x00c0
        L_0x00af:
            r1 = move-exception
            r2 = r3
        L_0x00b1:
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00bf }
            r4.deflateEnd()     // Catch:{ all -> 0x00bf }
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00bf }
            r4.next_in = r3     // Catch:{ all -> 0x00bf }
            org.jboss.netty.util.internal.jzlib.ZStream r4 = r9.z     // Catch:{ all -> 0x00bf }
            r4.next_out = r3     // Catch:{ all -> 0x00bf }
            throw r1     // Catch:{ all -> 0x00bf }
        L_0x00bf:
            r1 = move-exception
        L_0x00c0:
            monitor-exit(r0)     // Catch:{ all -> 0x00bf }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.compression.ZlibEncoder.finishEncode(org.jboss.netty.channel.ChannelHandlerContext, org.jboss.netty.channel.ChannelEvent):org.jboss.netty.channel.ChannelFuture");
    }

    public void beforeAdd(ChannelHandlerContext ctx2) throws Exception {
        this.ctx = ctx2;
    }

    public void afterAdd(ChannelHandlerContext ctx2) throws Exception {
    }

    public void beforeRemove(ChannelHandlerContext ctx2) throws Exception {
    }

    public void afterRemove(ChannelHandlerContext ctx2) throws Exception {
    }
}
