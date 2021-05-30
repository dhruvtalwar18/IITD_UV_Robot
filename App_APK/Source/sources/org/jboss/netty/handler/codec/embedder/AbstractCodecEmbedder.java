package org.jboss.netty.handler.codec.embedder;

import java.util.ConcurrentModificationException;
import java.util.LinkedList;
import java.util.Queue;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelPipelineException;
import org.jboss.netty.channel.ChannelSink;
import org.jboss.netty.channel.ChannelUpstreamHandler;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.DefaultChannelPipeline;
import org.jboss.netty.channel.ExceptionEvent;
import org.jboss.netty.channel.MessageEvent;

abstract class AbstractCodecEmbedder<E> implements CodecEmbedder<E> {
    private final Channel channel;
    private final ChannelPipeline pipeline;
    final Queue<Object> productQueue;
    private final AbstractCodecEmbedder<E>.EmbeddedChannelSink sink;

    protected AbstractCodecEmbedder(ChannelHandler... handlers) {
        this.sink = new EmbeddedChannelSink();
        this.productQueue = new LinkedList();
        this.pipeline = new EmbeddedChannelPipeline();
        configurePipeline(handlers);
        this.channel = new EmbeddedChannel(this.pipeline, this.sink);
        fireInitialEvents();
    }

    protected AbstractCodecEmbedder(ChannelBufferFactory bufferFactory, ChannelHandler... handlers) {
        this(handlers);
        getChannel().getConfig().setBufferFactory(bufferFactory);
    }

    private void fireInitialEvents() {
        Channels.fireChannelOpen(this.channel);
        Channels.fireChannelBound(this.channel, this.channel.getLocalAddress());
        Channels.fireChannelConnected(this.channel, this.channel.getRemoteAddress());
    }

    private void configurePipeline(ChannelHandler... handlers) {
        if (handlers == null) {
            throw new NullPointerException("handlers");
        } else if (handlers.length != 0) {
            int i = 0;
            while (i < handlers.length) {
                if (handlers[i] != null) {
                    this.pipeline.addLast(String.valueOf(i), handlers[i]);
                    i++;
                } else {
                    throw new NullPointerException("handlers[" + i + "]");
                }
            }
            this.pipeline.addLast("SINK", this.sink);
        } else {
            throw new IllegalArgumentException("handlers should contain at least one " + ChannelHandler.class.getSimpleName() + '.');
        }
    }

    public boolean finish() {
        Channels.close(this.channel);
        Channels.fireChannelDisconnected(this.channel);
        Channels.fireChannelUnbound(this.channel);
        Channels.fireChannelClosed(this.channel);
        return !this.productQueue.isEmpty();
    }

    /* access modifiers changed from: protected */
    public final Channel getChannel() {
        return this.channel;
    }

    /* access modifiers changed from: protected */
    public final boolean isEmpty() {
        return this.productQueue.isEmpty();
    }

    public final E poll() {
        return this.productQueue.poll();
    }

    public final E peek() {
        return this.productQueue.peek();
    }

    public final Object[] pollAll() {
        int size = size();
        Object[] a = new Object[size];
        int i = 0;
        while (i < size) {
            E product = poll();
            if (product != null) {
                a[i] = product;
                i++;
            } else {
                throw new ConcurrentModificationException();
            }
        }
        return a;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v5, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r4v7, resolved type: T[]} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public final <T> T[] pollAll(T[] r4) {
        /*
            r3 = this;
            if (r4 == 0) goto L_0x002d
            int r0 = r3.size()
            int r1 = r4.length
            if (r1 >= r0) goto L_0x0018
            java.lang.Class r1 = r4.getClass()
            java.lang.Class r1 = r1.getComponentType()
            java.lang.Object r1 = java.lang.reflect.Array.newInstance(r1, r0)
            r4 = r1
            java.lang.Object[] r4 = (java.lang.Object[]) r4
        L_0x0018:
            r1 = r4
            r4 = 0
        L_0x001a:
            java.lang.Object r2 = r3.poll()
            if (r2 != 0) goto L_0x0028
            int r4 = r1.length
            if (r4 <= r0) goto L_0x0027
            r4 = 0
            r1[r0] = r4
        L_0x0027:
            return r1
        L_0x0028:
            r1[r4] = r2
            int r4 = r4 + 1
            goto L_0x001a
        L_0x002d:
            java.lang.NullPointerException r0 = new java.lang.NullPointerException
            java.lang.String r1 = "a"
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.embedder.AbstractCodecEmbedder.pollAll(java.lang.Object[]):java.lang.Object[]");
    }

    public final int size() {
        return this.productQueue.size();
    }

    public ChannelPipeline getPipeline() {
        return this.pipeline;
    }

    private final class EmbeddedChannelSink implements ChannelSink, ChannelUpstreamHandler {
        static final /* synthetic */ boolean $assertionsDisabled = false;

        static {
            Class<AbstractCodecEmbedder> cls = AbstractCodecEmbedder.class;
        }

        EmbeddedChannelSink() {
        }

        public void handleUpstream(ChannelHandlerContext ctx, ChannelEvent e) {
            handleEvent(e);
        }

        public void eventSunk(ChannelPipeline pipeline, ChannelEvent e) {
            handleEvent(e);
        }

        private void handleEvent(ChannelEvent e) {
            if (e instanceof MessageEvent) {
                boolean offer = AbstractCodecEmbedder.this.productQueue.offer(((MessageEvent) e).getMessage());
            } else if (e instanceof ExceptionEvent) {
                throw new CodecEmbedderException(((ExceptionEvent) e).getCause());
            }
        }

        public void exceptionCaught(ChannelPipeline pipeline, ChannelEvent e, ChannelPipelineException cause) throws Exception {
            Throwable actualCause = cause.getCause();
            if (actualCause == null) {
                actualCause = cause;
            }
            throw new CodecEmbedderException(actualCause);
        }

        public ChannelFuture execute(ChannelPipeline pipeline, Runnable task) {
            try {
                task.run();
                return Channels.succeededFuture(pipeline.getChannel());
            } catch (Throwable t) {
                return Channels.failedFuture(pipeline.getChannel(), t);
            }
        }
    }

    private static final class EmbeddedChannelPipeline extends DefaultChannelPipeline {
        EmbeddedChannelPipeline() {
        }

        /* access modifiers changed from: protected */
        public void notifyHandlerException(ChannelEvent e, Throwable t) {
            while ((t instanceof ChannelPipelineException) && t.getCause() != null) {
                t = t.getCause();
            }
            if (t instanceof CodecEmbedderException) {
                throw ((CodecEmbedderException) t);
            }
            throw new CodecEmbedderException(t);
        }
    }
}
