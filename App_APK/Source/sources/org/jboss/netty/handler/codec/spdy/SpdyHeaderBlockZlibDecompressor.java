package org.jboss.netty.handler.codec.spdy;

import java.util.zip.DataFormatException;
import java.util.zip.Inflater;
import org.jboss.netty.buffer.ChannelBuffer;

class SpdyHeaderBlockZlibDecompressor extends SpdyHeaderBlockDecompressor {
    private final Inflater decompressor = new Inflater();
    private final byte[] out = new byte[8192];
    private final int version;

    public SpdyHeaderBlockZlibDecompressor(int version2) {
        if (version2 < 2 || version2 > 3) {
            throw new IllegalArgumentException("unsupported version: " + version2);
        }
        this.version = version2;
    }

    public void setInput(ChannelBuffer compressed) {
        byte[] in = new byte[compressed.readableBytes()];
        compressed.readBytes(in);
        this.decompressor.setInput(in);
    }

    public void decode(ChannelBuffer decompressed) throws Exception {
        try {
            int numBytes = this.decompressor.inflate(this.out);
            if (numBytes == 0 && this.decompressor.needsDictionary()) {
                if (this.version < 3) {
                    this.decompressor.setDictionary(SpdyCodecUtil.SPDY2_DICT);
                } else {
                    this.decompressor.setDictionary(SpdyCodecUtil.SPDY_DICT);
                }
                numBytes = this.decompressor.inflate(this.out);
            }
            decompressed.writeBytes(this.out, 0, numBytes);
        } catch (DataFormatException e) {
            throw new SpdyProtocolException("Received invalid header block", e);
        }
    }

    public void end() {
        this.decompressor.end();
    }
}
