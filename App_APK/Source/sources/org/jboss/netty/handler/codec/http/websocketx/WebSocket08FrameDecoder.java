package org.jboss.netty.handler.codec.http.websocketx;

import android.support.v4.media.session.PlaybackStateCompat;
import com.google.common.base.Ascii;
import com.google.common.primitives.UnsignedBytes;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.frame.CorruptedFrameException;
import org.jboss.netty.handler.codec.frame.TooLongFrameException;
import org.jboss.netty.handler.codec.replay.ReplayingDecoder;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.xbill.DNS.TTL;

public class WebSocket08FrameDecoder extends ReplayingDecoder<State> {
    private static final byte OPCODE_BINARY = 2;
    private static final byte OPCODE_CLOSE = 8;
    private static final byte OPCODE_CONT = 0;
    private static final byte OPCODE_PING = 9;
    private static final byte OPCODE_PONG = 10;
    private static final byte OPCODE_TEXT = 1;
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) WebSocket08FrameDecoder.class);
    private final boolean allowExtensions;
    private int fragmentedFramesCount;
    private UTF8Output fragmentedFramesText;
    private boolean frameFinalFlag;
    private int frameOpcode;
    private ChannelBuffer framePayload;
    private int framePayloadBytesRead;
    private long framePayloadLength;
    private int frameRsv;
    private final boolean maskedPayload;
    private ChannelBuffer maskingKey;
    private final long maxFramePayloadLength;
    private boolean receivedClosingHandshake;

    public enum State {
        FRAME_START,
        MASKING_KEY,
        PAYLOAD,
        CORRUPT
    }

    public WebSocket08FrameDecoder(boolean maskedPayload2, boolean allowExtensions2) {
        this(maskedPayload2, allowExtensions2, Long.MAX_VALUE);
    }

    public WebSocket08FrameDecoder(boolean maskedPayload2, boolean allowExtensions2, long maxFramePayloadLength2) {
        super(State.FRAME_START);
        this.maskedPayload = maskedPayload2;
        this.allowExtensions = allowExtensions2;
        this.maxFramePayloadLength = maxFramePayloadLength2;
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer, State state) throws Exception {
        int i;
        Channel channel2 = channel;
        ChannelBuffer channelBuffer = buffer;
        if (this.receivedClosingHandshake) {
            channelBuffer.skipBytes(actualReadableBytes());
            return null;
        }
        switch (state) {
            case FRAME_START:
                this.framePayloadBytesRead = 0;
                this.framePayloadLength = -1;
                this.framePayload = null;
                byte b = buffer.readByte();
                this.frameFinalFlag = (b & UnsignedBytes.MAX_POWER_OF_TWO) != 0;
                this.frameRsv = (b & 112) >> 4;
                this.frameOpcode = b & Ascii.SI;
                if (logger.isDebugEnabled()) {
                    logger.debug("Decoding WebSocket Frame opCode=" + this.frameOpcode);
                }
                int b2 = buffer.readByte();
                boolean frameMasked = (b2 & 128) != 0;
                int framePayloadLen1 = b2 & 127;
                if (this.frameRsv != 0 && !this.allowExtensions) {
                    protocolViolation(channel2, "RSV != 0 and no extension negotiated, RSV:" + this.frameRsv);
                    return null;
                } else if (!this.maskedPayload || frameMasked) {
                    if (this.frameOpcode > 7) {
                        if (!this.frameFinalFlag) {
                            protocolViolation(channel2, "fragmented control frame");
                            return null;
                        } else if (framePayloadLen1 > 125) {
                            protocolViolation(channel2, "control frame with payload length > 125 octets");
                            return null;
                        } else if (this.frameOpcode != 8 && this.frameOpcode != 9 && this.frameOpcode != 10) {
                            protocolViolation(channel2, "control frame using reserved opcode " + this.frameOpcode);
                            return null;
                        } else if (this.frameOpcode == 8 && framePayloadLen1 == 1) {
                            protocolViolation(channel2, "received close control frame with payload len 1");
                            return null;
                        }
                    } else if (this.frameOpcode != 0 && this.frameOpcode != 1 && this.frameOpcode != 2) {
                        protocolViolation(channel2, "data frame using reserved opcode " + this.frameOpcode);
                        return null;
                    } else if (this.fragmentedFramesCount == 0 && this.frameOpcode == 0) {
                        protocolViolation(channel2, "received continuation data frame outside fragmented message");
                        return null;
                    } else if (!(this.fragmentedFramesCount == 0 || this.frameOpcode == 0 || this.frameOpcode == 9)) {
                        protocolViolation(channel2, "received non-continuation data frame while inside fragmented message");
                        return null;
                    }
                    if (framePayloadLen1 == 126) {
                        this.framePayloadLength = (long) buffer.readUnsignedShort();
                        if (this.framePayloadLength < 126) {
                            protocolViolation(channel2, "invalid data frame length (not using minimal length encoding)");
                            return null;
                        }
                    } else if (framePayloadLen1 == 127) {
                        this.framePayloadLength = buffer.readLong();
                        if (this.framePayloadLength < PlaybackStateCompat.ACTION_PREPARE_FROM_SEARCH) {
                            protocolViolation(channel2, "invalid data frame length (not using minimal length encoding)");
                            return null;
                        }
                    } else {
                        this.framePayloadLength = (long) framePayloadLen1;
                    }
                    if (this.framePayloadLength <= this.maxFramePayloadLength) {
                        if (logger.isDebugEnabled()) {
                            logger.debug("Decoding WebSocket Frame length=" + this.framePayloadLength);
                        }
                        checkpoint(State.MASKING_KEY);
                        int i2 = b2;
                        break;
                    } else {
                        protocolViolation(channel2, "Max frame length of " + this.maxFramePayloadLength + " has been exceeded.");
                        return null;
                    }
                } else {
                    protocolViolation(channel2, "unmasked client to server frame");
                    return null;
                }
                break;
            case MASKING_KEY:
                break;
            case PAYLOAD:
                break;
            case CORRUPT:
                buffer.readByte();
                return null;
            default:
                throw new Error("Shouldn't reach here.");
        }
        if (this.maskedPayload != 0) {
            this.maskingKey = channelBuffer.readBytes(4);
        }
        checkpoint(State.PAYLOAD);
        int rbytes = actualReadableBytes();
        ChannelBuffer payloadBuffer = null;
        long willHaveReadByteCount = (long) (this.framePayloadBytesRead + rbytes);
        if (willHaveReadByteCount == this.framePayloadLength) {
            payloadBuffer = channelBuffer.readBytes(rbytes);
        } else if (willHaveReadByteCount < this.framePayloadLength) {
            ChannelBuffer payloadBuffer2 = channelBuffer.readBytes(rbytes);
            if (this.framePayload == null) {
                this.framePayload = channel.getConfig().getBufferFactory().getBuffer(toFrameLength(this.framePayloadLength));
            }
            this.framePayload.writeBytes(payloadBuffer2);
            this.framePayloadBytesRead += rbytes;
            return null;
        } else if (willHaveReadByteCount > this.framePayloadLength) {
            payloadBuffer = channelBuffer.readBytes(toFrameLength(this.framePayloadLength - ((long) this.framePayloadBytesRead)));
        }
        checkpoint(State.FRAME_START);
        if (this.framePayload == null) {
            this.framePayload = payloadBuffer;
        } else {
            this.framePayload.writeBytes(payloadBuffer);
        }
        if (this.maskedPayload) {
            unmask(this.framePayload);
        }
        if (this.frameOpcode == 9) {
            return new PingWebSocketFrame(this.frameFinalFlag, this.frameRsv, this.framePayload);
        }
        if (this.frameOpcode == 10) {
            return new PongWebSocketFrame(this.frameFinalFlag, this.frameRsv, this.framePayload);
        }
        if (this.frameOpcode == 8) {
            checkCloseFrameBody(channel2, this.framePayload);
            this.receivedClosingHandshake = true;
            return new CloseWebSocketFrame(this.frameFinalFlag, this.frameRsv, this.framePayload);
        }
        String aggregatedText = null;
        if (this.frameFinalFlag) {
            if (this.frameOpcode != 9) {
                this.fragmentedFramesCount = 0;
                if (this.frameOpcode == 1 || this.fragmentedFramesText != null) {
                    checkUTF8String(channel2, this.framePayload.array());
                    aggregatedText = this.fragmentedFramesText.toString();
                    this.fragmentedFramesText = null;
                }
            }
            i = 1;
        } else {
            if (this.fragmentedFramesCount == 0) {
                this.fragmentedFramesText = null;
                if (this.frameOpcode == 1) {
                    checkUTF8String(channel2, this.framePayload.array());
                }
            } else if (this.fragmentedFramesText != null) {
                checkUTF8String(channel2, this.framePayload.array());
            }
            i = 1;
            this.fragmentedFramesCount++;
        }
        if (this.frameOpcode == i) {
            return new TextWebSocketFrame(this.frameFinalFlag, this.frameRsv, this.framePayload);
        }
        if (this.frameOpcode == 2) {
            return new BinaryWebSocketFrame(this.frameFinalFlag, this.frameRsv, this.framePayload);
        }
        if (this.frameOpcode == 0) {
            return new ContinuationWebSocketFrame(this.frameFinalFlag, this.frameRsv, this.framePayload, aggregatedText);
        }
        throw new UnsupportedOperationException("Cannot decode web socket frame with opcode: " + this.frameOpcode);
    }

    private void unmask(ChannelBuffer frame) {
        byte[] bytes = frame.array();
        for (int i = 0; i < bytes.length; i++) {
            frame.setByte(i, frame.getByte(i) ^ this.maskingKey.getByte(i % 4));
        }
    }

    private void protocolViolation(Channel channel, String reason) throws CorruptedFrameException {
        checkpoint(State.CORRUPT);
        if (channel.isConnected()) {
            channel.write(ChannelBuffers.EMPTY_BUFFER).addListener(ChannelFutureListener.CLOSE);
            channel.close().awaitUninterruptibly();
        }
        throw new CorruptedFrameException(reason);
    }

    private static int toFrameLength(long l) throws TooLongFrameException {
        if (l <= TTL.MAX_VALUE) {
            return (int) l;
        }
        throw new TooLongFrameException("Length:" + l);
    }

    private void checkUTF8String(Channel channel, byte[] bytes) throws CorruptedFrameException {
        try {
            if (this.fragmentedFramesText == null) {
                this.fragmentedFramesText = new UTF8Output(bytes);
            } else {
                this.fragmentedFramesText.write(bytes);
            }
        } catch (UTF8Exception e) {
            protocolViolation(channel, "invalid UTF-8 bytes");
        }
    }

    /* access modifiers changed from: protected */
    public void checkCloseFrameBody(Channel channel, ChannelBuffer buffer) throws CorruptedFrameException {
        if (buffer != null && buffer.capacity() != 0) {
            if (buffer.capacity() == 1) {
                protocolViolation(channel, "Invalid close frame body");
            }
            int idx = buffer.readerIndex();
            buffer.readerIndex(0);
            int statusCode = buffer.readShort();
            if ((statusCode >= 0 && statusCode <= 999) || ((statusCode >= 1004 && statusCode <= 1006) || (statusCode >= 1012 && statusCode <= 2999))) {
                protocolViolation(channel, "Invalid close frame status code: " + statusCode);
            }
            if (buffer.readableBytes() > 0) {
                byte[] b = new byte[buffer.readableBytes()];
                buffer.readBytes(b);
                try {
                    new UTF8Output(b);
                } catch (UTF8Exception e) {
                    protocolViolation(channel, "Invalid close frame reason text. Invalid UTF-8 bytes");
                }
            }
            buffer.readerIndex(idx);
        }
    }
}
