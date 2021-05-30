package org.jboss.netty.handler.codec.spdy;

import com.google.common.primitives.UnsignedBytes;
import javax.jmdns.impl.constants.DNSRecordClass;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.handler.codec.frame.FrameDecoder;
import org.jboss.netty.handler.codec.frame.TooLongFrameException;
import sensor_msgs.NavSatStatus;

public class SpdyFrameDecoder extends FrameDecoder {
    private ChannelBuffer decompressed;
    private byte flags;
    private final SpdyHeaderBlockDecompressor headerBlockDecompressor;
    private int headerSize;
    private int length;
    private final int maxChunkSize;
    private final int maxHeaderSize;
    private int numHeaders;
    private SpdyHeaderBlock spdyHeaderBlock;
    private SpdySettingsFrame spdySettingsFrame;
    private final int spdyVersion;
    private State state;
    private int streamID;
    private int type;
    private int version;

    private enum State {
        READ_COMMON_HEADER,
        READ_CONTROL_FRAME,
        READ_SETTINGS_FRAME,
        READ_HEADER_BLOCK_FRAME,
        READ_HEADER_BLOCK,
        READ_DATA_FRAME,
        DISCARD_FRAME,
        FRAME_ERROR
    }

    @Deprecated
    public SpdyFrameDecoder() {
        this(2);
    }

    public SpdyFrameDecoder(int version2) {
        this(version2, 8192, 16384);
    }

    public SpdyFrameDecoder(int version2, int maxChunkSize2, int maxHeaderSize2) {
        super(false);
        if (version2 < 2 || version2 > 3) {
            throw new IllegalArgumentException("unsupported version: " + version2);
        } else if (maxChunkSize2 <= 0) {
            throw new IllegalArgumentException("maxChunkSize must be a positive integer: " + maxChunkSize2);
        } else if (maxHeaderSize2 > 0) {
            this.spdyVersion = version2;
            this.maxChunkSize = maxChunkSize2;
            this.maxHeaderSize = maxHeaderSize2;
            this.headerBlockDecompressor = SpdyHeaderBlockDecompressor.newInstance(version2);
            this.state = State.READ_COMMON_HEADER;
        } else {
            throw new IllegalArgumentException("maxHeaderSize must be a positive integer: " + maxHeaderSize2);
        }
    }

    /* access modifiers changed from: protected */
    public Object decodeLast(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer) throws Exception {
        try {
            return decode(ctx, channel, buffer);
        } finally {
            this.headerBlockDecompressor.end();
        }
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer) throws Exception {
        byte ID_flags;
        int ID;
        ChannelHandlerContext channelHandlerContext = ctx;
        ChannelBuffer channelBuffer = buffer;
        boolean z = false;
        switch (this.state) {
            case READ_COMMON_HEADER:
                this.state = readCommonHeader(channelBuffer);
                if (this.state == State.FRAME_ERROR) {
                    if (this.version != this.spdyVersion) {
                        fireProtocolException(channelHandlerContext, "Unsupported version: " + this.version);
                    } else {
                        fireInvalidControlFrameException(ctx);
                    }
                }
                if (this.length == 0) {
                    if (this.state != State.READ_DATA_FRAME) {
                        this.state = State.READ_COMMON_HEADER;
                    } else if (this.streamID == 0) {
                        this.state = State.FRAME_ERROR;
                        fireProtocolException(channelHandlerContext, "Received invalid data frame");
                        return null;
                    } else {
                        SpdyDataFrame spdyDataFrame = new DefaultSpdyDataFrame(this.streamID);
                        if ((this.flags & 1) != 0) {
                            z = true;
                        }
                        spdyDataFrame.setLast(z);
                        this.state = State.READ_COMMON_HEADER;
                        return spdyDataFrame;
                    }
                }
                return null;
            case READ_CONTROL_FRAME:
                try {
                    Object frame = readControlFrame(channelBuffer);
                    if (frame != null) {
                        this.state = State.READ_COMMON_HEADER;
                    }
                    return frame;
                } catch (IllegalArgumentException e) {
                    this.state = State.FRAME_ERROR;
                    fireInvalidControlFrameException(ctx);
                    return null;
                }
            case READ_SETTINGS_FRAME:
                if (this.spdySettingsFrame == null) {
                    if (buffer.readableBytes() < 4) {
                        return null;
                    }
                    int numEntries = SpdyCodecUtil.getUnsignedInt(channelBuffer, buffer.readerIndex());
                    channelBuffer.skipBytes(4);
                    this.length -= 4;
                    if ((this.length & 7) == 0 && (this.length >> 3) == numEntries) {
                        this.spdySettingsFrame = new DefaultSpdySettingsFrame();
                        this.spdySettingsFrame.setClearPreviouslyPersistedSettings((this.flags & 1) != 0);
                    } else {
                        this.state = State.FRAME_ERROR;
                        fireInvalidControlFrameException(ctx);
                        return null;
                    }
                }
                int readableEntries = Math.min(buffer.readableBytes() >> 3, this.length >> 3);
                for (int i = 0; i < readableEntries; i++) {
                    if (this.version < 3) {
                        ID = (buffer.readByte() & NavSatStatus.STATUS_NO_FIX) | ((buffer.readByte() & NavSatStatus.STATUS_NO_FIX) << 8) | ((buffer.readByte() & NavSatStatus.STATUS_NO_FIX) << 16);
                        ID_flags = buffer.readByte();
                    } else {
                        ID_flags = buffer.readByte();
                        ID = SpdyCodecUtil.getUnsignedMedium(channelBuffer, buffer.readerIndex());
                        channelBuffer.skipBytes(3);
                    }
                    int value = SpdyCodecUtil.getSignedInt(channelBuffer, buffer.readerIndex());
                    channelBuffer.skipBytes(4);
                    if (ID == 0) {
                        this.state = State.FRAME_ERROR;
                        this.spdySettingsFrame = null;
                        fireInvalidControlFrameException(ctx);
                        return null;
                    }
                    if (!this.spdySettingsFrame.isSet(ID)) {
                        this.spdySettingsFrame.setValue(ID, value, (ID_flags & 1) != 0, (ID_flags & 2) != 0);
                    }
                }
                this.length -= readableEntries * 8;
                if (this.length != 0) {
                    return null;
                }
                this.state = State.READ_COMMON_HEADER;
                Object frame2 = this.spdySettingsFrame;
                this.spdySettingsFrame = null;
                return frame2;
            case READ_HEADER_BLOCK_FRAME:
                try {
                    this.spdyHeaderBlock = readHeaderBlockFrame(channelBuffer);
                    if (this.spdyHeaderBlock != null) {
                        if (this.length == 0) {
                            this.state = State.READ_COMMON_HEADER;
                            Object frame3 = this.spdyHeaderBlock;
                            this.spdyHeaderBlock = null;
                            return frame3;
                        }
                        this.state = State.READ_HEADER_BLOCK;
                    }
                    return null;
                } catch (IllegalArgumentException e2) {
                    this.state = State.FRAME_ERROR;
                    fireInvalidControlFrameException(ctx);
                    return null;
                }
            case READ_HEADER_BLOCK:
                int compressedBytes = Math.min(buffer.readableBytes(), this.length);
                this.length -= compressedBytes;
                try {
                    decodeHeaderBlock(channelBuffer.readSlice(compressedBytes));
                    if (this.spdyHeaderBlock != null && this.spdyHeaderBlock.isInvalid()) {
                        Object frame4 = this.spdyHeaderBlock;
                        this.spdyHeaderBlock = null;
                        this.decompressed = null;
                        if (this.length == 0) {
                            this.state = State.READ_COMMON_HEADER;
                        }
                        return frame4;
                    } else if (this.length != 0) {
                        return null;
                    } else {
                        Object frame5 = this.spdyHeaderBlock;
                        this.spdyHeaderBlock = null;
                        this.state = State.READ_COMMON_HEADER;
                        return frame5;
                    }
                } catch (Exception e3) {
                    this.state = State.FRAME_ERROR;
                    this.spdyHeaderBlock = null;
                    this.decompressed = null;
                    Channels.fireExceptionCaught(channelHandlerContext, (Throwable) e3);
                    return null;
                }
            case READ_DATA_FRAME:
                if (this.streamID == 0) {
                    this.state = State.FRAME_ERROR;
                    fireProtocolException(channelHandlerContext, "Received invalid data frame");
                    return null;
                }
                int dataLength = Math.min(this.maxChunkSize, this.length);
                if (buffer.readableBytes() < dataLength) {
                    return null;
                }
                SpdyDataFrame spdyDataFrame2 = new DefaultSpdyDataFrame(this.streamID);
                spdyDataFrame2.setData(channelBuffer.readBytes(dataLength));
                this.length -= dataLength;
                if (this.length == 0) {
                    if ((this.flags & 1) != 0) {
                        z = true;
                    }
                    spdyDataFrame2.setLast(z);
                    this.state = State.READ_COMMON_HEADER;
                }
                return spdyDataFrame2;
            case DISCARD_FRAME:
                int numBytes = Math.min(buffer.readableBytes(), this.length);
                channelBuffer.skipBytes(numBytes);
                this.length -= numBytes;
                if (this.length == 0) {
                    this.state = State.READ_COMMON_HEADER;
                }
                return null;
            case FRAME_ERROR:
                channelBuffer.skipBytes(buffer.readableBytes());
                return null;
            default:
                throw new Error("Shouldn't reach here.");
        }
    }

    private State readCommonHeader(ChannelBuffer buffer) {
        if (buffer.readableBytes() < 8) {
            return State.READ_COMMON_HEADER;
        }
        int frameOffset = buffer.readerIndex();
        int flagsOffset = frameOffset + 4;
        int lengthOffset = frameOffset + 5;
        buffer.skipBytes(8);
        boolean control = (buffer.getByte(frameOffset) & UnsignedBytes.MAX_POWER_OF_TWO) != 0;
        this.flags = buffer.getByte(flagsOffset);
        this.length = SpdyCodecUtil.getUnsignedMedium(buffer, lengthOffset);
        if (control) {
            this.version = SpdyCodecUtil.getUnsignedShort(buffer, frameOffset) & DNSRecordClass.CLASS_MASK;
            this.type = SpdyCodecUtil.getUnsignedShort(buffer, frameOffset + 2);
            if (this.version != this.spdyVersion || !isValidControlFrameHeader()) {
                return State.FRAME_ERROR;
            }
            if (willGenerateControlFrame()) {
                int i = this.type;
                if (i == 4) {
                    return State.READ_SETTINGS_FRAME;
                }
                if (i != 8) {
                    switch (i) {
                        case 1:
                        case 2:
                            break;
                        default:
                            return State.READ_CONTROL_FRAME;
                    }
                }
                return State.READ_HEADER_BLOCK_FRAME;
            } else if (this.length != 0) {
                return State.DISCARD_FRAME;
            } else {
                return State.READ_COMMON_HEADER;
            }
        } else {
            this.streamID = SpdyCodecUtil.getUnsignedInt(buffer, frameOffset);
            return State.READ_DATA_FRAME;
        }
    }

    private Object readControlFrame(ChannelBuffer buffer) {
        int i = this.type;
        int minLength = 8;
        if (i != 3) {
            if (i != 9) {
                switch (i) {
                    case 6:
                        if (buffer.readableBytes() < 4) {
                            return null;
                        }
                        int ID = SpdyCodecUtil.getSignedInt(buffer, buffer.readerIndex());
                        buffer.skipBytes(4);
                        return new DefaultSpdyPingFrame(ID);
                    case 7:
                        if (this.version < 3) {
                            minLength = 4;
                        }
                        if (buffer.readableBytes() < minLength) {
                            return null;
                        }
                        int lastGoodStreamID = SpdyCodecUtil.getUnsignedInt(buffer, buffer.readerIndex());
                        buffer.skipBytes(4);
                        if (this.version < 3) {
                            return new DefaultSpdyGoAwayFrame(lastGoodStreamID);
                        }
                        int statusCode = SpdyCodecUtil.getSignedInt(buffer, buffer.readerIndex());
                        buffer.skipBytes(4);
                        return new DefaultSpdyGoAwayFrame(lastGoodStreamID, statusCode);
                    default:
                        throw new Error("Shouldn't reach here.");
                }
            } else if (buffer.readableBytes() < 8) {
                return null;
            } else {
                int streamID2 = SpdyCodecUtil.getUnsignedInt(buffer, buffer.readerIndex());
                int deltaWindowSize = SpdyCodecUtil.getUnsignedInt(buffer, buffer.readerIndex() + 4);
                buffer.skipBytes(8);
                return new DefaultSpdyWindowUpdateFrame(streamID2, deltaWindowSize);
            }
        } else if (buffer.readableBytes() < 8) {
            return null;
        } else {
            int streamID3 = SpdyCodecUtil.getUnsignedInt(buffer, buffer.readerIndex());
            int statusCode2 = SpdyCodecUtil.getSignedInt(buffer, buffer.readerIndex() + 4);
            buffer.skipBytes(8);
            return new DefaultSpdyRstStreamFrame(streamID3, statusCode2);
        }
    }

    private SpdyHeaderBlock readHeaderBlockFrame(ChannelBuffer buffer) {
        int i = this.type;
        int minLength = 8;
        boolean z = false;
        if (i != 8) {
            switch (i) {
                case 1:
                    if (buffer.readableBytes() < (this.version < 3 ? 12 : 10)) {
                        return null;
                    }
                    int offset = buffer.readerIndex();
                    int streamID2 = SpdyCodecUtil.getUnsignedInt(buffer, offset);
                    int associatedToStreamID = SpdyCodecUtil.getUnsignedInt(buffer, offset + 4);
                    byte priority = (byte) ((buffer.getByte(offset + 8) >> 5) & 7);
                    if (this.version < 3) {
                        priority = (byte) (priority >> 1);
                    }
                    buffer.skipBytes(10);
                    this.length -= 10;
                    if (this.version < 3 && this.length == 2 && buffer.getShort(buffer.readerIndex()) == 0) {
                        buffer.skipBytes(2);
                        this.length = 0;
                    }
                    SpdySynStreamFrame spdySynStreamFrame = new DefaultSpdySynStreamFrame(streamID2, associatedToStreamID, priority);
                    spdySynStreamFrame.setLast((this.flags & 1) != 0);
                    if ((this.flags & 2) != 0) {
                        z = true;
                    }
                    spdySynStreamFrame.setUnidirectional(z);
                    return spdySynStreamFrame;
                case 2:
                    if (this.version >= 3) {
                        minLength = 4;
                    }
                    if (buffer.readableBytes() < minLength) {
                        return null;
                    }
                    int streamID3 = SpdyCodecUtil.getUnsignedInt(buffer, buffer.readerIndex());
                    buffer.skipBytes(4);
                    this.length -= 4;
                    if (this.version < 3) {
                        buffer.skipBytes(2);
                        this.length -= 2;
                    }
                    if (this.version < 3 && this.length == 2 && buffer.getShort(buffer.readerIndex()) == 0) {
                        buffer.skipBytes(2);
                        this.length = 0;
                    }
                    SpdySynReplyFrame spdySynReplyFrame = new DefaultSpdySynReplyFrame(streamID3);
                    if ((this.flags & 1) != 0) {
                        z = true;
                    }
                    spdySynReplyFrame.setLast(z);
                    return spdySynReplyFrame;
                default:
                    throw new Error("Shouldn't reach here.");
            }
        } else if (buffer.readableBytes() < 4) {
            return null;
        } else {
            if (this.version < 3 && this.length > 4 && buffer.readableBytes() < 8) {
                return null;
            }
            int streamID4 = SpdyCodecUtil.getUnsignedInt(buffer, buffer.readerIndex());
            buffer.skipBytes(4);
            this.length -= 4;
            if (this.version < 3 && this.length != 0) {
                buffer.skipBytes(2);
                this.length -= 2;
            }
            if (this.version < 3 && this.length == 2 && buffer.getShort(buffer.readerIndex()) == 0) {
                buffer.skipBytes(2);
                this.length = 0;
            }
            SpdyHeadersFrame spdyHeadersFrame = new DefaultSpdyHeadersFrame(streamID4);
            if ((this.flags & 1) != 0) {
                z = true;
            }
            spdyHeadersFrame.setLast(z);
            return spdyHeadersFrame;
        }
    }

    private boolean ensureBytes(int bytes) throws Exception {
        if (this.decompressed.readableBytes() >= bytes) {
            return true;
        }
        this.headerBlockDecompressor.decode(this.decompressed);
        if (this.decompressed.readableBytes() >= bytes) {
            return true;
        }
        return false;
    }

    private int readLengthField() {
        if (this.version < 3) {
            return this.decompressed.readUnsignedShort();
        }
        return this.decompressed.readInt();
    }

    private void decodeHeaderBlock(ChannelBuffer buffer) throws Exception {
        if (this.decompressed == null) {
            this.headerSize = 0;
            this.numHeaders = -1;
            this.decompressed = ChannelBuffers.dynamicBuffer(8192);
        }
        this.headerBlockDecompressor.setInput(buffer);
        this.headerBlockDecompressor.decode(this.decompressed);
        if (this.spdyHeaderBlock == null) {
            this.decompressed = null;
            return;
        }
        int lengthFieldSize = this.version < 3 ? 2 : 4;
        if (this.numHeaders == -1) {
            if (this.decompressed.readableBytes() >= lengthFieldSize) {
                this.numHeaders = readLengthField();
                if (this.numHeaders < 0) {
                    this.spdyHeaderBlock.setInvalid();
                    return;
                }
            } else {
                return;
            }
        }
        while (this.numHeaders > 0) {
            int headerSize2 = this.headerSize;
            this.decompressed.markReaderIndex();
            if (!ensureBytes(lengthFieldSize)) {
                this.decompressed.resetReaderIndex();
                this.decompressed.discardReadBytes();
                return;
            }
            int nameLength = readLengthField();
            if (nameLength <= 0) {
                this.spdyHeaderBlock.setInvalid();
                return;
            }
            int headerSize3 = headerSize2 + nameLength;
            if (headerSize3 > this.maxHeaderSize) {
                throw new TooLongFrameException("Header block exceeds " + this.maxHeaderSize);
            } else if (!ensureBytes(nameLength)) {
                this.decompressed.resetReaderIndex();
                this.decompressed.discardReadBytes();
                return;
            } else {
                byte[] nameBytes = new byte[nameLength];
                this.decompressed.readBytes(nameBytes);
                String name = new String(nameBytes, "UTF-8");
                if (this.spdyHeaderBlock.containsHeader(name)) {
                    this.spdyHeaderBlock.setInvalid();
                    return;
                } else if (!ensureBytes(lengthFieldSize)) {
                    this.decompressed.resetReaderIndex();
                    this.decompressed.discardReadBytes();
                    return;
                } else {
                    int valueLength = readLengthField();
                    if (valueLength <= 0) {
                        this.spdyHeaderBlock.setInvalid();
                        return;
                    }
                    int headerSize4 = headerSize3 + valueLength;
                    if (headerSize4 > this.maxHeaderSize) {
                        throw new TooLongFrameException("Header block exceeds " + this.maxHeaderSize);
                    } else if (!ensureBytes(valueLength)) {
                        this.decompressed.resetReaderIndex();
                        this.decompressed.discardReadBytes();
                        return;
                    } else {
                        byte[] valueBytes = new byte[valueLength];
                        this.decompressed.readBytes(valueBytes);
                        int index = 0;
                        int offset = 0;
                        while (index < valueLength) {
                            while (index < valueBytes.length && valueBytes[index] != 0) {
                                index++;
                            }
                            if (index >= valueBytes.length || valueBytes[index + 1] != 0) {
                                try {
                                    this.spdyHeaderBlock.addHeader(name, new String(valueBytes, offset, index - offset, "UTF-8"));
                                    index++;
                                    offset = index;
                                } catch (IllegalArgumentException e) {
                                    this.spdyHeaderBlock.setInvalid();
                                    return;
                                }
                            } else {
                                this.spdyHeaderBlock.setInvalid();
                                return;
                            }
                        }
                        this.numHeaders--;
                        this.headerSize = headerSize4;
                    }
                }
            }
        }
        this.decompressed = null;
    }

    /* JADX WARNING: Removed duplicated region for block: B:20:0x0032 A[ORIG_RETURN, RETURN, SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:41:0x0063 A[ORIG_RETURN, RETURN, SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:48:0x0076 A[ORIG_RETURN, RETURN, SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private boolean isValidControlFrameHeader() {
        /*
            r6 = this;
            int r0 = r6.type
            r1 = 3
            r2 = 8
            r3 = 4
            r4 = 0
            r5 = 1
            switch(r0) {
                case 1: goto L_0x006c;
                case 2: goto L_0x005b;
                case 3: goto L_0x0050;
                case 4: goto L_0x0049;
                case 5: goto L_0x0042;
                case 6: goto L_0x003b;
                case 7: goto L_0x002a;
                case 8: goto L_0x0013;
                case 9: goto L_0x000c;
                default: goto L_0x000b;
            }
        L_0x000b:
            return r5
        L_0x000c:
            int r0 = r6.length
            if (r0 != r2) goto L_0x0012
            r4 = 1
        L_0x0012:
            return r4
        L_0x0013:
            int r0 = r6.version
            if (r0 >= r1) goto L_0x0023
            int r0 = r6.length
            if (r0 == r3) goto L_0x0021
            int r0 = r6.length
            if (r0 < r2) goto L_0x0020
            goto L_0x0021
        L_0x0020:
            goto L_0x0022
        L_0x0021:
            r4 = 1
        L_0x0022:
            return r4
        L_0x0023:
            int r0 = r6.length
            if (r0 < r3) goto L_0x0029
            r4 = 1
        L_0x0029:
            return r4
        L_0x002a:
            int r0 = r6.version
            if (r0 >= r1) goto L_0x0035
            int r0 = r6.length
            if (r0 != r3) goto L_0x0034
        L_0x0032:
            r4 = 1
            goto L_0x003a
        L_0x0034:
            goto L_0x003a
        L_0x0035:
            int r0 = r6.length
            if (r0 != r2) goto L_0x0034
            goto L_0x0032
        L_0x003a:
            return r4
        L_0x003b:
            int r0 = r6.length
            if (r0 != r3) goto L_0x0041
            r4 = 1
        L_0x0041:
            return r4
        L_0x0042:
            int r0 = r6.length
            if (r0 != 0) goto L_0x0048
            r4 = 1
        L_0x0048:
            return r4
        L_0x0049:
            int r0 = r6.length
            if (r0 < r3) goto L_0x004f
            r4 = 1
        L_0x004f:
            return r4
        L_0x0050:
            byte r0 = r6.flags
            if (r0 != 0) goto L_0x005a
            int r0 = r6.length
            if (r0 != r2) goto L_0x005a
            r4 = 1
        L_0x005a:
            return r4
        L_0x005b:
            int r0 = r6.version
            if (r0 >= r1) goto L_0x0066
            int r0 = r6.length
            if (r0 < r2) goto L_0x0065
        L_0x0063:
            r4 = 1
            goto L_0x006b
        L_0x0065:
            goto L_0x006b
        L_0x0066:
            int r0 = r6.length
            if (r0 < r3) goto L_0x0065
            goto L_0x0063
        L_0x006b:
            return r4
        L_0x006c:
            int r0 = r6.version
            if (r0 >= r1) goto L_0x0079
            int r0 = r6.length
            r1 = 12
            if (r0 < r1) goto L_0x0078
        L_0x0076:
            r4 = 1
            goto L_0x0080
        L_0x0078:
            goto L_0x0080
        L_0x0079:
            int r0 = r6.length
            r1 = 10
            if (r0 < r1) goto L_0x0078
            goto L_0x0076
        L_0x0080:
            return r4
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.spdy.SpdyFrameDecoder.isValidControlFrameHeader():boolean");
    }

    private boolean willGenerateControlFrame() {
        switch (this.type) {
            case 1:
            case 2:
            case 3:
            case 4:
            case 6:
            case 7:
            case 8:
            case 9:
                return true;
            default:
                return false;
        }
    }

    private void fireInvalidControlFrameException(ChannelHandlerContext ctx) {
        String message = "Received invalid control frame";
        switch (this.type) {
            case 1:
                message = "Received invalid SYN_STREAM control frame";
                break;
            case 2:
                message = "Received invalid SYN_REPLY control frame";
                break;
            case 3:
                message = "Received invalid RST_STREAM control frame";
                break;
            case 4:
                message = "Received invalid SETTINGS control frame";
                break;
            case 5:
                message = "Received invalid NOOP control frame";
                break;
            case 6:
                message = "Received invalid PING control frame";
                break;
            case 7:
                message = "Received invalid GOAWAY control frame";
                break;
            case 8:
                message = "Received invalid HEADERS control frame";
                break;
            case 9:
                message = "Received invalid WINDOW_UPDATE control frame";
                break;
            case 10:
                message = "Received invalid CREDENTIAL control frame";
                break;
        }
        fireProtocolException(ctx, message);
    }

    private static void fireProtocolException(ChannelHandlerContext ctx, String message) {
        Channels.fireExceptionCaught(ctx, (Throwable) new SpdyProtocolException(message));
    }
}
