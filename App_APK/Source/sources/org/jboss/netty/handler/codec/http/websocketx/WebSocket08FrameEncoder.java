package org.jboss.netty.handler.codec.http.websocketx;

import com.google.common.primitives.UnsignedBytes;
import java.nio.ByteBuffer;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.frame.TooLongFrameException;
import org.jboss.netty.handler.codec.oneone.OneToOneEncoder;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;

public class WebSocket08FrameEncoder extends OneToOneEncoder {
    private static final byte OPCODE_BINARY = 2;
    private static final byte OPCODE_CLOSE = 8;
    private static final byte OPCODE_CONT = 0;
    private static final byte OPCODE_PING = 9;
    private static final byte OPCODE_PONG = 10;
    private static final byte OPCODE_TEXT = 1;
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) WebSocket08FrameEncoder.class);
    private final boolean maskPayload;

    public WebSocket08FrameEncoder(boolean maskPayload2) {
        this.maskPayload = maskPayload2;
    }

    /* access modifiers changed from: protected */
    public Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        byte opcode;
        ChannelBuffer header;
        ChannelBuffer body;
        Object obj = msg;
        if (!(obj instanceof WebSocketFrame)) {
            return obj;
        }
        WebSocketFrame frame = (WebSocketFrame) obj;
        ChannelBuffer data = frame.getBinaryData();
        if (data == null) {
            data = ChannelBuffers.EMPTY_BUFFER;
        }
        if (frame instanceof TextWebSocketFrame) {
            opcode = 1;
        } else if (frame instanceof PingWebSocketFrame) {
            opcode = 9;
        } else if (frame instanceof PongWebSocketFrame) {
            opcode = 10;
        } else if (frame instanceof CloseWebSocketFrame) {
            opcode = 8;
        } else if (frame instanceof BinaryWebSocketFrame) {
            opcode = 2;
        } else if (frame instanceof ContinuationWebSocketFrame) {
            opcode = 0;
        } else {
            throw new UnsupportedOperationException("Cannot encode frame of type: " + frame.getClass().getName());
        }
        int length = data.readableBytes();
        if (logger.isDebugEnabled()) {
            InternalLogger internalLogger = logger;
            internalLogger.debug("Encoding WebSocket Frame opCode=" + opcode + " length=" + length);
        }
        int b0 = 0;
        if (frame.isFinalFragment()) {
            b0 = 0 | 128;
        }
        int b02 = b0 | ((frame.getRsv() % 8) << 4) | (opcode % UnsignedBytes.MAX_POWER_OF_TWO);
        if (opcode != 9 || length <= 125) {
            int maskLength = this.maskPayload ? 4 : 0;
            if (length <= 125) {
                header = ChannelBuffers.buffer(maskLength + 2);
                header.writeByte(b02);
                header.writeByte((byte) (this.maskPayload ? ((byte) length) | UnsignedBytes.MAX_POWER_OF_TWO : (byte) length));
            } else {
                int i = 255;
                if (length <= 65535) {
                    header = ChannelBuffers.buffer(maskLength + 4);
                    header.writeByte(b02);
                    header.writeByte(this.maskPayload ? 254 : 126);
                    header.writeByte(255 & (length >>> 8));
                    header.writeByte(length & 255);
                } else {
                    header = ChannelBuffers.buffer(maskLength + 10);
                    header.writeByte(b02);
                    if (!this.maskPayload) {
                        i = 127;
                    }
                    header.writeByte(i);
                    header.writeLong((long) length);
                }
            }
            if (this.maskPayload) {
                byte[] mask = ByteBuffer.allocate(4).putInt(Integer.valueOf((int) (Math.random() * 2.147483647E9d)).intValue()).array();
                header.writeBytes(mask);
                body = ChannelBuffers.buffer(length);
                int counter = 0;
                while (data.readableBytes() > 0) {
                    body.writeByte(mask[counter % 4] ^ data.readByte());
                    counter++;
                }
            } else {
                body = data;
            }
            return ChannelBuffers.wrappedBuffer(header, body);
        }
        throw new TooLongFrameException("invalid payload for PING (payload length must be <= 125, was " + length);
    }
}
