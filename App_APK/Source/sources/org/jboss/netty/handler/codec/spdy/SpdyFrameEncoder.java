package org.jboss.netty.handler.codec.spdy;

import com.google.common.base.Ascii;
import java.nio.ByteOrder;
import java.util.Set;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.handler.codec.oneone.OneToOneEncoder;
import sensor_msgs.NavSatStatus;

public class SpdyFrameEncoder extends OneToOneEncoder {
    private volatile boolean finished;
    private final SpdyHeaderBlockCompressor headerBlockCompressor;
    private final int version;

    @Deprecated
    public SpdyFrameEncoder() {
        this(2, 6, 15, 8);
    }

    public SpdyFrameEncoder(int version2) {
        this(version2, 6, 15, 8);
    }

    @Deprecated
    public SpdyFrameEncoder(int compressionLevel, int windowBits, int memLevel) {
        this(2, compressionLevel, windowBits, memLevel);
    }

    public SpdyFrameEncoder(int version2, int compressionLevel, int windowBits, int memLevel) {
        if (version2 < 2 || version2 > 3) {
            throw new IllegalArgumentException("unknown version: " + version2);
        }
        this.version = version2;
        this.headerBlockCompressor = SpdyHeaderBlockCompressor.newInstance(version2, compressionLevel, windowBits, memLevel);
    }

    public void handleDownstream(ChannelHandlerContext ctx, ChannelEvent evt) throws Exception {
        if (evt instanceof ChannelStateEvent) {
            ChannelStateEvent e = (ChannelStateEvent) evt;
            switch (e.getState()) {
                case OPEN:
                case CONNECTED:
                case BOUND:
                    if (Boolean.FALSE.equals(e.getValue()) || e.getValue() == null) {
                        synchronized (this.headerBlockCompressor) {
                            this.finished = true;
                            this.headerBlockCompressor.end();
                        }
                        break;
                    }
            }
        }
        super.handleDownstream(ctx, evt);
    }

    /* access modifiers changed from: protected */
    public Object encode(ChannelHandlerContext ctx, Channel channel, Object msg) throws Exception {
        Object obj = msg;
        int length = 8;
        if (obj instanceof SpdyDataFrame) {
            SpdyDataFrame spdyDataFrame = (SpdyDataFrame) obj;
            ChannelBuffer data = spdyDataFrame.getData();
            byte flags = spdyDataFrame.isLast();
            ChannelBuffer header = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, 8);
            header.writeInt(spdyDataFrame.getStreamId() & Integer.MAX_VALUE);
            header.writeByte((int) flags);
            header.writeMedium(data.readableBytes());
            return ChannelBuffers.wrappedBuffer(header, data);
        }
        int length2 = 12;
        if (obj instanceof SpdySynStreamFrame) {
            SpdySynStreamFrame spdySynStreamFrame = (SpdySynStreamFrame) obj;
            ChannelBuffer data2 = compressHeaderBlock(encodeHeaderBlock(this.version, spdySynStreamFrame));
            byte flags2 = spdySynStreamFrame.isLast();
            if (spdySynStreamFrame.isUnidirectional()) {
                flags2 = (byte) (flags2 | 2);
            }
            int headerBlockLength = data2.readableBytes();
            if (this.version >= 3) {
                length2 = headerBlockLength + 10;
            } else if (headerBlockLength != 0) {
                length2 = headerBlockLength + 10;
            }
            ChannelBuffer frame = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, length2 + 8);
            frame.writeShort(32768 | this.version);
            frame.writeShort(1);
            frame.writeByte(flags2);
            frame.writeMedium(length2);
            frame.writeInt(spdySynStreamFrame.getStreamId());
            frame.writeInt(spdySynStreamFrame.getAssociatedToStreamId());
            if (this.version < 3) {
                byte priority = spdySynStreamFrame.getPriority();
                if (priority > 3) {
                    priority = 3;
                }
                frame.writeShort((priority & NavSatStatus.STATUS_NO_FIX) << Ascii.SO);
            } else {
                frame.writeShort((spdySynStreamFrame.getPriority() & NavSatStatus.STATUS_NO_FIX) << 13);
            }
            if (this.version < 3 && data2.readableBytes() == 0) {
                frame.writeShort(0);
            }
            return ChannelBuffers.wrappedBuffer(frame, data2);
        } else if (obj instanceof SpdySynReplyFrame) {
            SpdySynReplyFrame spdySynReplyFrame = (SpdySynReplyFrame) obj;
            ChannelBuffer data3 = compressHeaderBlock(encodeHeaderBlock(this.version, spdySynReplyFrame));
            byte flags3 = spdySynReplyFrame.isLast();
            int headerBlockLength2 = data3.readableBytes();
            if (this.version >= 3) {
                length = headerBlockLength2 + 4;
            } else if (headerBlockLength2 != 0) {
                length = headerBlockLength2 + 6;
            }
            ChannelBuffer frame2 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, length + 8);
            frame2.writeShort(32768 | this.version);
            frame2.writeShort(2);
            frame2.writeByte((int) flags3);
            frame2.writeMedium(length);
            frame2.writeInt(spdySynReplyFrame.getStreamId());
            if (this.version < 3) {
                if (data3.readableBytes() == 0) {
                    frame2.writeInt(0);
                } else {
                    frame2.writeShort(0);
                }
            }
            return ChannelBuffers.wrappedBuffer(frame2, data3);
        } else if (obj instanceof SpdyRstStreamFrame) {
            SpdyRstStreamFrame spdyRstStreamFrame = (SpdyRstStreamFrame) obj;
            ChannelBuffer frame3 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, 16);
            frame3.writeShort(this.version | 32768);
            frame3.writeShort(3);
            frame3.writeInt(8);
            frame3.writeInt(spdyRstStreamFrame.getStreamId());
            frame3.writeInt(spdyRstStreamFrame.getStatus().getCode());
            return frame3;
        } else {
            int length3 = 4;
            if (obj instanceof SpdySettingsFrame) {
                SpdySettingsFrame spdySettingsFrame = (SpdySettingsFrame) obj;
                byte flags4 = spdySettingsFrame.clearPreviouslyPersistedSettings();
                Set<Integer> IDs = spdySettingsFrame.getIds();
                int numEntries = IDs.size();
                int length4 = (numEntries * 8) + 4;
                ChannelBuffer frame4 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, length4 + 8);
                frame4.writeShort(32768 | this.version);
                frame4.writeShort(4);
                frame4.writeByte((int) flags4);
                frame4.writeMedium(length4);
                frame4.writeInt(numEntries);
                for (Integer ID : IDs) {
                    int id = ID.intValue();
                    byte ID_flags = 0;
                    if (spdySettingsFrame.isPersistValue(id)) {
                        ID_flags = (byte) (0 | 1);
                    }
                    if (spdySettingsFrame.isPersisted(id)) {
                        ID_flags = (byte) (ID_flags | 2);
                    }
                    if (this.version < 3) {
                        frame4.writeByte((id >> 0) & 255);
                        frame4.writeByte((id >> 8) & 255);
                        frame4.writeByte((id >> 16) & 255);
                        frame4.writeByte(ID_flags);
                    } else {
                        frame4.writeByte(ID_flags);
                        frame4.writeMedium(id);
                    }
                    frame4.writeInt(spdySettingsFrame.getValue(id));
                }
                return frame4;
            } else if (obj instanceof SpdyNoOpFrame) {
                ChannelBuffer frame5 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, 8);
                frame5.writeShort(this.version | 32768);
                frame5.writeShort(5);
                frame5.writeInt(0);
                return frame5;
            } else if (obj instanceof SpdyPingFrame) {
                ChannelBuffer frame6 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, 12);
                frame6.writeShort(this.version | 32768);
                frame6.writeShort(6);
                frame6.writeInt(4);
                frame6.writeInt(((SpdyPingFrame) obj).getId());
                return frame6;
            } else if (obj instanceof SpdyGoAwayFrame) {
                SpdyGoAwayFrame spdyGoAwayFrame = (SpdyGoAwayFrame) obj;
                if (this.version < 3) {
                    length = 4;
                }
                int length5 = length;
                ChannelBuffer frame7 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, length5 + 8);
                frame7.writeShort(this.version | 32768);
                frame7.writeShort(7);
                frame7.writeInt(length5);
                frame7.writeInt(spdyGoAwayFrame.getLastGoodStreamId());
                if (this.version >= 3) {
                    frame7.writeInt(spdyGoAwayFrame.getStatus().getCode());
                }
                return frame7;
            } else if (obj instanceof SpdyHeadersFrame) {
                SpdyHeadersFrame spdyHeadersFrame = (SpdyHeadersFrame) obj;
                ChannelBuffer data4 = compressHeaderBlock(encodeHeaderBlock(this.version, spdyHeadersFrame));
                byte flags5 = spdyHeadersFrame.isLast();
                int headerBlockLength3 = data4.readableBytes();
                if (this.version >= 3) {
                    length3 = headerBlockLength3 + 4;
                } else if (headerBlockLength3 != 0) {
                    length3 = headerBlockLength3 + 6;
                }
                ChannelBuffer frame8 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, length3 + 8);
                frame8.writeShort(32768 | this.version);
                frame8.writeShort(8);
                frame8.writeByte((int) flags5);
                frame8.writeMedium(length3);
                frame8.writeInt(spdyHeadersFrame.getStreamId());
                if (this.version < 3 && data4.readableBytes() != 0) {
                    frame8.writeShort(0);
                }
                return ChannelBuffers.wrappedBuffer(frame8, data4);
            } else if (!(obj instanceof SpdyWindowUpdateFrame)) {
                return obj;
            } else {
                SpdyWindowUpdateFrame spdyWindowUpdateFrame = (SpdyWindowUpdateFrame) obj;
                ChannelBuffer frame9 = ChannelBuffers.buffer(ByteOrder.BIG_ENDIAN, 16);
                frame9.writeShort(this.version | 32768);
                frame9.writeShort(9);
                frame9.writeInt(8);
                frame9.writeInt(spdyWindowUpdateFrame.getStreamId());
                frame9.writeInt(spdyWindowUpdateFrame.getDeltaWindowSize());
                return frame9;
            }
        }
    }

    private static void writeLengthField(int version2, ChannelBuffer buffer, int length) {
        if (version2 < 3) {
            buffer.writeShort(length);
        } else {
            buffer.writeInt(length);
        }
    }

    private static void setLengthField(int version2, ChannelBuffer buffer, int writerIndex, int length) {
        if (version2 < 3) {
            buffer.setShort(writerIndex, length);
        } else {
            buffer.setInt(writerIndex, length);
        }
    }

    private static ChannelBuffer encodeHeaderBlock(int version2, SpdyHeaderBlock headerFrame) throws Exception {
        Set<String> names = headerFrame.getHeaderNames();
        int numHeaders = names.size();
        if (numHeaders == 0) {
            return ChannelBuffers.EMPTY_BUFFER;
        }
        if (numHeaders <= 65535) {
            ChannelBuffer headerBlock = ChannelBuffers.dynamicBuffer(ByteOrder.BIG_ENDIAN, 256);
            writeLengthField(version2, headerBlock, numHeaders);
            for (String name : names) {
                byte[] nameBytes = name.getBytes("UTF-8");
                writeLengthField(version2, headerBlock, nameBytes.length);
                headerBlock.writeBytes(nameBytes);
                int savedIndex = headerBlock.writerIndex();
                int valueLength = 0;
                writeLengthField(version2, headerBlock, 0);
                for (String value : headerFrame.getHeaders(name)) {
                    byte[] valueBytes = value.getBytes("UTF-8");
                    headerBlock.writeBytes(valueBytes);
                    headerBlock.writeByte(0);
                    valueLength += valueBytes.length + 1;
                }
                int valueLength2 = valueLength - 1;
                if (valueLength2 <= 65535) {
                    setLengthField(version2, headerBlock, savedIndex, valueLength2);
                    headerBlock.writerIndex(headerBlock.writerIndex() - 1);
                } else {
                    throw new IllegalArgumentException("header exceeds allowable length: " + name);
                }
            }
            return headerBlock;
        }
        throw new IllegalArgumentException("header block contains too many headers");
    }

    private synchronized ChannelBuffer compressHeaderBlock(ChannelBuffer uncompressed) throws Exception {
        if (uncompressed.readableBytes() == 0) {
            return ChannelBuffers.EMPTY_BUFFER;
        }
        ChannelBuffer compressed = ChannelBuffers.dynamicBuffer();
        synchronized (this.headerBlockCompressor) {
            if (!this.finished) {
                this.headerBlockCompressor.setInput(uncompressed);
                this.headerBlockCompressor.encode(compressed);
            }
        }
        return compressed;
    }
}
