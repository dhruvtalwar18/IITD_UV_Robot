package org.jboss.netty.handler.codec.frame;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;

public class DelimiterBasedFrameDecoder extends FrameDecoder {
    private final ChannelBuffer[] delimiters;
    private boolean discardingTooLongFrame;
    private final boolean failFast;
    private final int maxFrameLength;
    private final boolean stripDelimiter;
    private int tooLongFrameLength;

    public DelimiterBasedFrameDecoder(int maxFrameLength2, ChannelBuffer delimiter) {
        this(maxFrameLength2, true, delimiter);
    }

    public DelimiterBasedFrameDecoder(int maxFrameLength2, boolean stripDelimiter2, ChannelBuffer delimiter) {
        this(maxFrameLength2, stripDelimiter2, false, delimiter);
    }

    public DelimiterBasedFrameDecoder(int maxFrameLength2, boolean stripDelimiter2, boolean failFast2, ChannelBuffer delimiter) {
        validateMaxFrameLength(maxFrameLength2);
        validateDelimiter(delimiter);
        this.delimiters = new ChannelBuffer[]{delimiter.slice(delimiter.readerIndex(), delimiter.readableBytes())};
        this.maxFrameLength = maxFrameLength2;
        this.stripDelimiter = stripDelimiter2;
        this.failFast = failFast2;
    }

    public DelimiterBasedFrameDecoder(int maxFrameLength2, ChannelBuffer... delimiters2) {
        this(maxFrameLength2, true, delimiters2);
    }

    public DelimiterBasedFrameDecoder(int maxFrameLength2, boolean stripDelimiter2, ChannelBuffer... delimiters2) {
        this(maxFrameLength2, stripDelimiter2, false, delimiters2);
    }

    public DelimiterBasedFrameDecoder(int maxFrameLength2, boolean stripDelimiter2, boolean failFast2, ChannelBuffer... delimiters2) {
        validateMaxFrameLength(maxFrameLength2);
        if (delimiters2 == null) {
            throw new NullPointerException("delimiters");
        } else if (delimiters2.length != 0) {
            this.delimiters = new ChannelBuffer[delimiters2.length];
            for (int i = 0; i < delimiters2.length; i++) {
                ChannelBuffer d = delimiters2[i];
                validateDelimiter(d);
                this.delimiters[i] = d.slice(d.readerIndex(), d.readableBytes());
            }
            this.maxFrameLength = maxFrameLength2;
            this.stripDelimiter = stripDelimiter2;
            this.failFast = failFast2;
        } else {
            throw new IllegalArgumentException("empty delimiters");
        }
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer) throws Exception {
        ChannelBuffer minDelim = null;
        int minFrameLength = Integer.MAX_VALUE;
        for (ChannelBuffer delim : this.delimiters) {
            int frameLength = indexOf(buffer, delim);
            if (frameLength >= 0 && frameLength < minFrameLength) {
                minFrameLength = frameLength;
                minDelim = delim;
            }
        }
        if (minDelim != null) {
            int minDelimLength = minDelim.capacity();
            if (this.discardingTooLongFrame) {
                this.discardingTooLongFrame = false;
                buffer.skipBytes(minFrameLength + minDelimLength);
                int tooLongFrameLength2 = this.tooLongFrameLength;
                this.tooLongFrameLength = 0;
                if (!this.failFast) {
                    fail(ctx, (long) tooLongFrameLength2);
                }
                return null;
            } else if (minFrameLength > this.maxFrameLength) {
                buffer.skipBytes(minFrameLength + minDelimLength);
                fail(ctx, (long) minFrameLength);
                return null;
            } else if (!this.stripDelimiter) {
                return buffer.readBytes(minFrameLength + minDelimLength);
            } else {
                ChannelBuffer frame = buffer.readBytes(minFrameLength);
                buffer.skipBytes(minDelimLength);
                return frame;
            }
        } else {
            if (this.discardingTooLongFrame != 0) {
                this.tooLongFrameLength += buffer.readableBytes();
                buffer.skipBytes(buffer.readableBytes());
            } else if (buffer.readableBytes() > this.maxFrameLength) {
                this.tooLongFrameLength = buffer.readableBytes();
                buffer.skipBytes(buffer.readableBytes());
                this.discardingTooLongFrame = true;
                if (this.failFast) {
                    fail(ctx, (long) this.tooLongFrameLength);
                }
            }
            return null;
        }
    }

    private void fail(ChannelHandlerContext ctx, long frameLength) {
        if (frameLength > 0) {
            Channel channel = ctx.getChannel();
            Channels.fireExceptionCaught(channel, (Throwable) new TooLongFrameException("frame length exceeds " + this.maxFrameLength + ": " + frameLength + " - discarded"));
            return;
        }
        Channel channel2 = ctx.getChannel();
        Channels.fireExceptionCaught(channel2, (Throwable) new TooLongFrameException("frame length exceeds " + this.maxFrameLength + " - discarding"));
    }

    private static int indexOf(ChannelBuffer haystack, ChannelBuffer needle) {
        for (int i = haystack.readerIndex(); i < haystack.writerIndex(); i++) {
            int haystackIndex = i;
            int needleIndex = 0;
            while (needleIndex < needle.capacity() && haystack.getByte(haystackIndex) == needle.getByte(needleIndex)) {
                haystackIndex++;
                if (haystackIndex == haystack.writerIndex() && needleIndex != needle.capacity() - 1) {
                    return -1;
                }
                needleIndex++;
            }
            if (needleIndex == needle.capacity()) {
                return i - haystack.readerIndex();
            }
        }
        return -1;
    }

    private static void validateDelimiter(ChannelBuffer delimiter) {
        if (delimiter == null) {
            throw new NullPointerException("delimiter");
        } else if (!delimiter.readable()) {
            throw new IllegalArgumentException("empty delimiter");
        }
    }

    private static void validateMaxFrameLength(int maxFrameLength2) {
        if (maxFrameLength2 <= 0) {
            throw new IllegalArgumentException("maxFrameLength must be a positive integer: " + maxFrameLength2);
        }
    }
}
