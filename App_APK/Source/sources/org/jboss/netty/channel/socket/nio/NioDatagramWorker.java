package org.jboss.netty.channel.socket.nio;

import java.io.IOException;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.AsynchronousCloseException;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.DatagramChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.util.Queue;
import java.util.concurrent.Executor;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelException;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.ReceiveBufferSizePredictor;
import org.jboss.netty.channel.socket.nio.SocketSendBufferPool;

public class NioDatagramWorker extends AbstractNioWorker {
    static final /* synthetic */ boolean $assertionsDisabled = false;

    public /* bridge */ /* synthetic */ void executeInIoThread(Runnable x0) {
        super.executeInIoThread(x0);
    }

    public /* bridge */ /* synthetic */ void executeInIoThread(Runnable x0, boolean x1) {
        super.executeInIoThread(x0, x1);
    }

    public /* bridge */ /* synthetic */ void run() {
        super.run();
    }

    NioDatagramWorker(Executor executor) {
        super(executor);
    }

    NioDatagramWorker(Executor executor, boolean allowShutdownOnIdle) {
        super(executor, allowShutdownOnIdle);
    }

    /* access modifiers changed from: protected */
    public boolean read(SelectionKey key) {
        NioDatagramChannel channel = (NioDatagramChannel) key.attachment();
        ReceiveBufferSizePredictor predictor = channel.getConfig().getReceiveBufferSizePredictor();
        ChannelBufferFactory bufferFactory = channel.getConfig().getBufferFactory();
        DatagramChannel nioChannel = (DatagramChannel) key.channel();
        ByteBuffer byteBuffer = ByteBuffer.allocate(predictor.nextReceiveBufferSize()).order(bufferFactory.getDefaultOrder());
        boolean failure = true;
        SocketAddress remoteAddress = null;
        try {
            remoteAddress = nioChannel.receive(byteBuffer);
            failure = false;
        } catch (ClosedChannelException e) {
        } catch (Throwable t) {
            Channels.fireExceptionCaught((Channel) channel, t);
        }
        if (remoteAddress != null) {
            byteBuffer.flip();
            int readBytes = byteBuffer.remaining();
            if (readBytes > 0) {
                predictor.previousReceiveBufferSize(readBytes);
                Channels.fireMessageReceived((Channel) channel, (Object) bufferFactory.getBuffer(byteBuffer), remoteAddress);
            }
        }
        if (!failure) {
            return true;
        }
        key.cancel();
        close(channel, Channels.succeededFuture(channel));
        return false;
    }

    /* access modifiers changed from: protected */
    public boolean scheduleWriteIfNecessary(AbstractNioChannel<?> channel) {
        Thread workerThread = this.thread;
        if (workerThread != null && Thread.currentThread() == workerThread) {
            return false;
        }
        if (channel.writeTaskInTaskQueue.compareAndSet(false, true)) {
            boolean offer = this.writeTaskQueue.offer(channel.writeTask);
        }
        Selector selector = this.selector;
        if (selector != null && this.wakenUp.compareAndSet(false, true)) {
            selector.wakeup();
        }
        return true;
    }

    static void disconnect(NioDatagramChannel channel, ChannelFuture future) {
        boolean connected = channel.isConnected();
        boolean iothread = isIoThread(channel);
        try {
            channel.getDatagramChannel().disconnect();
            future.setSuccess();
            if (!connected) {
                return;
            }
            if (iothread) {
                Channels.fireChannelDisconnected((Channel) channel);
            } else {
                Channels.fireChannelDisconnectedLater(channel);
            }
        } catch (Throwable t) {
            future.setFailure(t);
            if (iothread) {
                Channels.fireExceptionCaught((Channel) channel, t);
            } else {
                Channels.fireExceptionCaughtLater((Channel) channel, t);
            }
        }
    }

    /* access modifiers changed from: protected */
    public Runnable createRegisterTask(AbstractNioChannel<?> channel, ChannelFuture future) {
        return new ChannelRegistionTask((NioDatagramChannel) channel, future);
    }

    private final class ChannelRegistionTask implements Runnable {
        private final NioDatagramChannel channel;
        private final ChannelFuture future;

        ChannelRegistionTask(NioDatagramChannel channel2, ChannelFuture future2) {
            this.channel = channel2;
            this.future = future2;
        }

        public void run() {
            if (this.channel.getLocalAddress() == null) {
                if (this.future != null) {
                    this.future.setFailure(new ClosedChannelException());
                }
                NioDatagramWorker.this.close(this.channel, Channels.succeededFuture(this.channel));
                return;
            }
            try {
                synchronized (this.channel.interestOpsLock) {
                    this.channel.getDatagramChannel().register(NioDatagramWorker.this.selector, this.channel.getRawInterestOps(), this.channel);
                }
                if (this.future != null) {
                    this.future.setSuccess();
                }
            } catch (IOException e) {
                if (this.future != null) {
                    this.future.setFailure(e);
                }
                NioDatagramWorker.this.close(this.channel, Channels.succeededFuture(this.channel));
                if (!(e instanceof ClosedChannelException)) {
                    throw new ChannelException("Failed to register a socket to the selector.", e);
                }
            }
        }
    }

    public void writeFromUserCode(AbstractNioChannel<?> channel) {
        if (!channel.isBound()) {
            cleanUpWriteBuffer(channel);
        } else if (!scheduleWriteIfNecessary(channel) && !channel.writeSuspended && !channel.inWriteNowLoop) {
            write0(channel);
        }
    }

    /* access modifiers changed from: protected */
    public void write0(AbstractNioChannel<?> channel) {
        SocketSendBufferPool.SendBuffer buf;
        long localWrittenBytes;
        AbstractNioChannel<?> abstractNioChannel = channel;
        boolean addOpWrite = false;
        boolean removeOpWrite = false;
        long writtenBytes = 0;
        SocketSendBufferPool sendBufferPool = this.sendBufferPool;
        DatagramChannel ch = ((NioDatagramChannel) abstractNioChannel).getDatagramChannel();
        Queue<MessageEvent> writeBuffer = abstractNioChannel.writeBufferQueue;
        int writeSpinCount = channel.getConfig().getWriteSpinCount();
        synchronized (abstractNioChannel.writeLock) {
            abstractNioChannel.inWriteNowLoop = true;
            while (true) {
                MessageEvent evt = abstractNioChannel.currentWriteEvent;
                if (evt == null) {
                    MessageEvent poll = writeBuffer.poll();
                    evt = poll;
                    abstractNioChannel.currentWriteEvent = poll;
                    if (poll == null) {
                        removeOpWrite = true;
                        abstractNioChannel.writeSuspended = false;
                        break;
                    }
                    SocketSendBufferPool.SendBuffer acquire = sendBufferPool.acquire(evt.getMessage());
                    buf = acquire;
                    abstractNioChannel.currentWriteBuffer = acquire;
                } else {
                    buf = abstractNioChannel.currentWriteBuffer;
                }
                MessageEvent evt2 = evt;
                try {
                    SocketAddress raddr = evt2.getRemoteAddress();
                    if (raddr != null) {
                        localWrittenBytes = 0;
                        int i = writeSpinCount;
                        while (true) {
                            if (i <= 0) {
                                break;
                            }
                            localWrittenBytes = buf.transferTo(ch, raddr);
                            if (localWrittenBytes != 0) {
                                writtenBytes += localWrittenBytes;
                                break;
                            } else if (buf.finished()) {
                                break;
                            } else {
                                i--;
                            }
                        }
                    } else {
                        localWrittenBytes = 0;
                        int i2 = writeSpinCount;
                        while (true) {
                            if (i2 <= 0) {
                                break;
                            }
                            localWrittenBytes = buf.transferTo(ch);
                            if (localWrittenBytes != 0) {
                                writtenBytes += localWrittenBytes;
                                break;
                            } else if (buf.finished()) {
                                break;
                            } else {
                                i2--;
                            }
                        }
                    }
                    if (localWrittenBytes <= 0) {
                        if (!buf.finished()) {
                            addOpWrite = true;
                            abstractNioChannel.writeSuspended = true;
                            break;
                        }
                    }
                    buf.release();
                    ChannelFuture future = evt2.getFuture();
                    abstractNioChannel.currentWriteEvent = null;
                    abstractNioChannel.currentWriteBuffer = null;
                    evt2 = null;
                    buf = null;
                    future.setSuccess();
                } catch (AsynchronousCloseException e) {
                } catch (Throwable t) {
                    buf.release();
                    abstractNioChannel.currentWriteEvent = null;
                    abstractNioChannel.currentWriteBuffer = null;
                    evt2.getFuture().setFailure(t);
                    Channels.fireExceptionCaught((Channel) abstractNioChannel, t);
                }
            }
            abstractNioChannel.inWriteNowLoop = false;
            if (addOpWrite) {
                setOpWrite(channel);
            } else if (removeOpWrite) {
                clearOpWrite(channel);
            }
        }
        Channels.fireWriteComplete((Channel) abstractNioChannel, writtenBytes);
    }
}
