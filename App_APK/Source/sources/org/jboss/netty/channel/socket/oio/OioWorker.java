package org.jboss.netty.channel.socket.oio;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PushbackInputStream;
import java.net.SocketAddress;
import java.net.SocketException;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.WritableByteChannel;
import java.util.regex.Pattern;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.DefaultFileRegion;
import org.jboss.netty.channel.FileRegion;

class OioWorker extends AbstractOioWorker<OioSocketChannel> {
    private static final Pattern SOCKET_CLOSED_MESSAGE = Pattern.compile("^.*(?:Socket.*closed).*$", 2);

    OioWorker(OioSocketChannel channel) {
        super(channel);
    }

    public void run() {
        if ((this.channel instanceof OioAcceptedSocketChannel) && ((OioSocketChannel) this.channel).isOpen()) {
            Channels.fireChannelConnected((Channel) this.channel, (SocketAddress) ((OioSocketChannel) this.channel).getRemoteAddress());
        }
        super.run();
    }

    /* access modifiers changed from: package-private */
    public boolean process() throws IOException {
        PushbackInputStream in = ((OioSocketChannel) this.channel).getInputStream();
        int bytesToRead = in.available();
        if (bytesToRead > 0) {
            byte[] buf = new byte[bytesToRead];
            Channels.fireMessageReceived((Channel) this.channel, (Object) ((OioSocketChannel) this.channel).getConfig().getBufferFactory().getBuffer(buf, 0, in.read(buf)));
            return true;
        }
        int b = in.read();
        if (b < 0) {
            return false;
        }
        in.unread(b);
        return true;
    }

    static void write(OioSocketChannel channel, ChannelFuture future, Object message) {
        int length;
        OioSocketChannel channel2 = channel;
        Object message2 = message;
        boolean iothread = isIoThread(channel);
        OutputStream out = channel.getOutputStream();
        if (out == null) {
            Exception e = new ClosedChannelException();
            future.setFailure(e);
            if (iothread) {
                Channels.fireExceptionCaught((Channel) channel2, (Throwable) e);
            } else {
                Channels.fireExceptionCaughtLater((Channel) channel2, (Throwable) e);
            }
        } else {
            ChannelFuture future2 = future;
            int length2 = 0;
            try {
                if (message2 instanceof FileRegion) {
                    FileRegion fr = (FileRegion) message2;
                    try {
                        synchronized (out) {
                            WritableByteChannel bchannel = java.nio.channels.Channels.newChannel(out);
                            do {
                                long transferTo = fr.transferTo(bchannel, (long) length2);
                                long i = transferTo;
                                if (transferTo <= 0) {
                                    break;
                                }
                                length2 = (int) (((long) length2) + i);
                            } while (((long) length2) < fr.getCount());
                        }
                        length = length2;
                        if ((fr instanceof DefaultFileRegion) != 0 && ((DefaultFileRegion) fr).releaseAfterTransfer()) {
                            fr.releaseExternalResources();
                        }
                    } catch (Throwable th) {
                        channel2 = channel;
                        future2 = future;
                        Object obj = message;
                        if ((fr instanceof DefaultFileRegion) && ((DefaultFileRegion) fr).releaseAfterTransfer()) {
                            fr.releaseExternalResources();
                        }
                        throw th;
                    }
                } else {
                    ChannelBuffer a = (ChannelBuffer) message2;
                    int length3 = a.readableBytes();
                    synchronized (out) {
                        a.getBytes(a.readerIndex(), out, length3);
                    }
                    length = length3;
                }
                future.setSuccess();
                if (iothread) {
                    Channels.fireWriteComplete((Channel) channel2, (long) length);
                } else {
                    Channels.fireWriteCompleteLater(channel2, (long) length);
                }
            } catch (Throwable th2) {
                t = th2;
                if ((t instanceof SocketException) && SOCKET_CLOSED_MESSAGE.matcher(String.valueOf(t.getMessage())).matches()) {
                    t = new ClosedChannelException();
                }
                future2.setFailure(t);
                if (iothread) {
                    Channels.fireExceptionCaught((Channel) channel2, t);
                } else {
                    Channels.fireExceptionCaughtLater((Channel) channel2, t);
                }
            }
        }
    }
}
