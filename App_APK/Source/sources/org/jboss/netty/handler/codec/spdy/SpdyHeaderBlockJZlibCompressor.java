package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.handler.codec.compression.CompressionException;
import org.jboss.netty.util.internal.jzlib.JZlib;
import org.jboss.netty.util.internal.jzlib.ZStream;

class SpdyHeaderBlockJZlibCompressor extends SpdyHeaderBlockCompressor {
    private final ZStream z = new ZStream();

    public SpdyHeaderBlockJZlibCompressor(int version, int compressionLevel, int windowBits, int memLevel) {
        int resultCode;
        if (version < 2 || version > 3) {
            throw new IllegalArgumentException("unsupported version: " + version);
        } else if (compressionLevel < 0 || compressionLevel > 9) {
            throw new IllegalArgumentException("compressionLevel: " + compressionLevel + " (expected: 0-9)");
        } else if (windowBits < 9 || windowBits > 15) {
            throw new IllegalArgumentException("windowBits: " + windowBits + " (expected: 9-15)");
        } else if (memLevel < 1 || memLevel > 9) {
            throw new IllegalArgumentException("memLevel: " + memLevel + " (expected: 1-9)");
        } else {
            int resultCode2 = this.z.deflateInit(compressionLevel, windowBits, memLevel, JZlib.W_ZLIB);
            if (resultCode2 == 0) {
                if (version < 3) {
                    resultCode = this.z.deflateSetDictionary(SpdyCodecUtil.SPDY2_DICT, SpdyCodecUtil.SPDY2_DICT.length);
                } else {
                    resultCode = this.z.deflateSetDictionary(SpdyCodecUtil.SPDY_DICT, SpdyCodecUtil.SPDY_DICT.length);
                }
                if (resultCode != 0) {
                    throw new CompressionException("failed to set the SPDY dictionary: " + resultCode);
                }
                return;
            }
            throw new CompressionException("failed to initialize an SPDY header block deflater: " + resultCode2);
        }
    }

    public void setInput(ChannelBuffer decompressed) {
        byte[] in = new byte[decompressed.readableBytes()];
        decompressed.readBytes(in);
        this.z.next_in = in;
        this.z.next_in_index = 0;
        this.z.avail_in = in.length;
    }

    public void encode(ChannelBuffer compressed) {
        try {
            double length = (double) this.z.next_in.length;
            Double.isNaN(length);
            byte[] out = new byte[(((int) Math.ceil(length * 1.001d)) + 12)];
            this.z.next_out = out;
            this.z.next_out_index = 0;
            this.z.avail_out = out.length;
            int resultCode = this.z.deflate(2);
            if (resultCode == 0) {
                if (this.z.next_out_index != 0) {
                    compressed.writeBytes(out, 0, this.z.next_out_index);
                }
                return;
            }
            throw new CompressionException("compression failure: " + resultCode);
        } finally {
            this.z.next_in = null;
            this.z.next_out = null;
        }
    }

    public void end() {
        this.z.deflateEnd();
        this.z.next_in = null;
        this.z.next_out = null;
    }
}
