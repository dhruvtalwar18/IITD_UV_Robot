package org.jboss.netty.handler.codec.spdy;

import java.util.zip.Deflater;
import org.jboss.netty.buffer.ChannelBuffer;

class SpdyHeaderBlockZlibCompressor extends SpdyHeaderBlockCompressor {
    private final Deflater compressor;
    private final byte[] out = new byte[8192];

    public SpdyHeaderBlockZlibCompressor(int version, int compressionLevel) {
        if (version < 2 || version > 3) {
            throw new IllegalArgumentException("unsupported version: " + version);
        } else if (compressionLevel < 0 || compressionLevel > 9) {
            throw new IllegalArgumentException("compressionLevel: " + compressionLevel + " (expected: 0-9)");
        } else {
            this.compressor = new Deflater(compressionLevel);
            if (version < 3) {
                this.compressor.setDictionary(SpdyCodecUtil.SPDY2_DICT);
            } else {
                this.compressor.setDictionary(SpdyCodecUtil.SPDY_DICT);
            }
        }
    }

    public void setInput(ChannelBuffer decompressed) {
        byte[] in = new byte[decompressed.readableBytes()];
        decompressed.readBytes(in);
        this.compressor.setInput(in);
    }

    public void encode(ChannelBuffer compressed) {
        while (!this.compressor.needsInput()) {
            compressed.writeBytes(this.out, 0, this.compressor.deflate(this.out, 0, this.out.length, 2));
        }
    }

    public void end() {
        this.compressor.end();
    }
}
