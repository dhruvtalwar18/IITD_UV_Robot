package org.jboss.netty.bootstrap;

import java.net.SocketAddress;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelException;
import org.jboss.netty.channel.ChannelFactory;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.ChildChannelStateEvent;
import org.jboss.netty.channel.ExceptionEvent;
import org.jboss.netty.channel.ServerChannelFactory;
import org.jboss.netty.channel.SimpleChannelUpstreamHandler;
import org.xbill.DNS.TTL;

public class ServerBootstrap extends Bootstrap {
    private volatile ChannelHandler parentHandler;

    public ServerBootstrap() {
    }

    public ServerBootstrap(ChannelFactory channelFactory) {
        super(channelFactory);
    }

    public void setFactory(ChannelFactory factory) {
        if (factory == null) {
            throw new NullPointerException("factory");
        } else if (factory instanceof ServerChannelFactory) {
            super.setFactory(factory);
        } else {
            throw new IllegalArgumentException("factory must be a " + ServerChannelFactory.class.getSimpleName() + ": " + factory.getClass());
        }
    }

    public ChannelHandler getParentHandler() {
        return this.parentHandler;
    }

    public void setParentHandler(ChannelHandler parentHandler2) {
        this.parentHandler = parentHandler2;
    }

    public Channel bind() {
        SocketAddress localAddress = (SocketAddress) getOption("localAddress");
        if (localAddress != null) {
            return bind(localAddress);
        }
        throw new IllegalStateException("localAddress option is not set.");
    }

    public Channel bind(SocketAddress localAddress) {
        if (localAddress != null) {
            BlockingQueue<ChannelFuture> futureQueue = new LinkedBlockingQueue<>();
            ChannelHandler binder = new Binder(localAddress, futureQueue);
            ChannelHandler parentHandler2 = getParentHandler();
            ChannelPipeline bossPipeline = Channels.pipeline();
            bossPipeline.addLast("binder", binder);
            if (parentHandler2 != null) {
                bossPipeline.addLast("userHandler", parentHandler2);
            }
            Channel channel = getFactory().newChannel(bossPipeline);
            ChannelFuture future = null;
            boolean interrupted = false;
            do {
                try {
                    future = futureQueue.poll(TTL.MAX_VALUE, TimeUnit.SECONDS);
                    continue;
                } catch (InterruptedException e) {
                    interrupted = true;
                    continue;
                }
            } while (future == null);
            if (interrupted) {
                Thread.currentThread().interrupt();
            }
            future.awaitUninterruptibly();
            if (future.isSuccess()) {
                return channel;
            }
            future.getChannel().close().awaitUninterruptibly();
            throw new ChannelException("Failed to bind to: " + localAddress, future.getCause());
        }
        throw new NullPointerException("localAddress");
    }

    private final class Binder extends SimpleChannelUpstreamHandler {
        static final /* synthetic */ boolean $assertionsDisabled = false;
        private final Map<String, Object> childOptions = new HashMap();
        private final BlockingQueue<ChannelFuture> futureQueue;
        private final SocketAddress localAddress;

        static {
            Class<ServerBootstrap> cls = ServerBootstrap.class;
        }

        Binder(SocketAddress localAddress2, BlockingQueue<ChannelFuture> futureQueue2) {
            this.localAddress = localAddress2;
            this.futureQueue = futureQueue2;
        }

        /* JADX INFO: finally extract failed */
        public void channelOpen(ChannelHandlerContext ctx, ChannelStateEvent evt) {
            try {
                evt.getChannel().getConfig().setPipelineFactory(ServerBootstrap.this.getPipelineFactory());
                Map<String, Object> allOptions = ServerBootstrap.this.getOptions();
                Map<String, Object> parentOptions = new HashMap<>();
                for (Map.Entry<String, Object> e : allOptions.entrySet()) {
                    if (e.getKey().startsWith("child.")) {
                        this.childOptions.put(e.getKey().substring(6), e.getValue());
                    } else if (!e.getKey().equals("pipelineFactory")) {
                        parentOptions.put(e.getKey(), e.getValue());
                    }
                }
                evt.getChannel().getConfig().setOptions(parentOptions);
                ctx.sendUpstream(evt);
                boolean offer = this.futureQueue.offer(evt.getChannel().bind(this.localAddress));
            } catch (Throwable th) {
                ctx.sendUpstream(evt);
                throw th;
            }
        }

        public void childChannelOpen(ChannelHandlerContext ctx, ChildChannelStateEvent e) throws Exception {
            try {
                e.getChildChannel().getConfig().setOptions(this.childOptions);
            } catch (Throwable t) {
                Channels.fireExceptionCaught(e.getChildChannel(), t);
            }
            ctx.sendUpstream(e);
        }

        public void exceptionCaught(ChannelHandlerContext ctx, ExceptionEvent e) throws Exception {
            boolean offer = this.futureQueue.offer(Channels.failedFuture(e.getChannel(), e.getCause()));
            ctx.sendUpstream(e);
        }
    }
}
