package org.jboss.netty.handler.codec.base64;

import com.google.common.base.Ascii;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.buffer.HeapChannelBufferFactory;

public final class Base64 {
    private static final byte EQUALS_SIGN = 61;
    private static final byte EQUALS_SIGN_ENC = -1;
    private static final int MAX_LINE_LENGTH = 76;
    private static final byte NEW_LINE = 10;
    private static final byte WHITE_SPACE_ENC = -5;

    private static byte[] alphabet(Base64Dialect dialect) {
        if (dialect != null) {
            return dialect.alphabet;
        }
        throw new NullPointerException("dialect");
    }

    private static byte[] decodabet(Base64Dialect dialect) {
        if (dialect != null) {
            return dialect.decodabet;
        }
        throw new NullPointerException("dialect");
    }

    private static boolean breakLines(Base64Dialect dialect) {
        if (dialect != null) {
            return dialect.breakLinesByDefault;
        }
        throw new NullPointerException("dialect");
    }

    public static ChannelBuffer encode(ChannelBuffer src) {
        return encode(src, Base64Dialect.STANDARD);
    }

    public static ChannelBuffer encode(ChannelBuffer src, Base64Dialect dialect) {
        return encode(src, breakLines(dialect), dialect);
    }

    public static ChannelBuffer encode(ChannelBuffer src, ChannelBufferFactory bufferFactory) {
        return encode(src, Base64Dialect.STANDARD, bufferFactory);
    }

    public static ChannelBuffer encode(ChannelBuffer src, Base64Dialect dialect, ChannelBufferFactory bufferFactory) {
        return encode(src, breakLines(dialect), dialect, bufferFactory);
    }

    public static ChannelBuffer encode(ChannelBuffer src, boolean breakLines) {
        return encode(src, breakLines, Base64Dialect.STANDARD);
    }

    public static ChannelBuffer encode(ChannelBuffer src, boolean breakLines, Base64Dialect dialect) {
        return encode(src, breakLines, dialect, HeapChannelBufferFactory.getInstance());
    }

    public static ChannelBuffer encode(ChannelBuffer src, boolean breakLines, ChannelBufferFactory bufferFactory) {
        return encode(src, breakLines, Base64Dialect.STANDARD, bufferFactory);
    }

    public static ChannelBuffer encode(ChannelBuffer src, boolean breakLines, Base64Dialect dialect, ChannelBufferFactory bufferFactory) {
        if (src != null) {
            ChannelBuffer dest = encode(src, src.readerIndex(), src.readableBytes(), breakLines, dialect, bufferFactory);
            src.readerIndex(src.writerIndex());
            return dest;
        }
        throw new NullPointerException("src");
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len) {
        return encode(src, off, len, Base64Dialect.STANDARD);
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, Base64Dialect dialect) {
        return encode(src, off, len, breakLines(dialect), dialect);
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, ChannelBufferFactory bufferFactory) {
        return encode(src, off, len, Base64Dialect.STANDARD, bufferFactory);
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, Base64Dialect dialect, ChannelBufferFactory bufferFactory) {
        return encode(src, off, len, breakLines(dialect), dialect, bufferFactory);
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, boolean breakLines) {
        return encode(src, off, len, breakLines, Base64Dialect.STANDARD);
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, boolean breakLines, Base64Dialect dialect) {
        return encode(src, off, len, breakLines, dialect, HeapChannelBufferFactory.getInstance());
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, boolean breakLines, ChannelBufferFactory bufferFactory) {
        return encode(src, off, len, breakLines, Base64Dialect.STANDARD, bufferFactory);
    }

    public static ChannelBuffer encode(ChannelBuffer src, int off, int len, boolean breakLines, Base64Dialect dialect, ChannelBufferFactory bufferFactory) {
        int i = len;
        ChannelBufferFactory channelBufferFactory = bufferFactory;
        if (src == null) {
            throw new NullPointerException("src");
        } else if (dialect == null) {
            throw new NullPointerException("dialect");
        } else if (channelBufferFactory != null) {
            int len43 = (i * 4) / 3;
            ChannelBuffer dest = channelBufferFactory.getBuffer(src.order(), (i % 3 > 0 ? 4 : 0) + len43 + (breakLines ? len43 / 76 : 0));
            int len2 = i - 2;
            int d = 0;
            int e = 0;
            int lineLength = 0;
            while (true) {
                int lineLength2 = lineLength;
                if (d >= len2) {
                    break;
                }
                int d2 = d;
                int len22 = len2;
                encode3to4(src, d + off, 3, dest, e, dialect);
                lineLength = lineLength2 + 4;
                if (breakLines && lineLength == 76) {
                    dest.setByte(e + 4, 10);
                    e++;
                    lineLength = 0;
                }
                d = d2 + 3;
                e += 4;
                len2 = len22;
            }
            int d3 = d;
            int i2 = len2;
            if (d3 < i) {
                encode3to4(src, d3 + off, i - d3, dest, e, dialect);
                e += 4;
            }
            return dest.slice(0, e);
        } else {
            throw new NullPointerException("bufferFactory");
        }
    }

    private static void encode3to4(ChannelBuffer src, int srcOffset, int numSigBytes, ChannelBuffer dest, int destOffset, Base64Dialect dialect) {
        byte[] ALPHABET = alphabet(dialect);
        int i = 0;
        int i2 = (numSigBytes > 0 ? (src.getByte(srcOffset) << Ascii.CAN) >>> 8 : 0) | (numSigBytes > 1 ? (src.getByte(srcOffset + 1) << Ascii.CAN) >>> 16 : 0);
        if (numSigBytes > 2) {
            i = (src.getByte(srcOffset + 2) << Ascii.CAN) >>> 24;
        }
        int inBuff = i | i2;
        switch (numSigBytes) {
            case 1:
                dest.setByte(destOffset, ALPHABET[inBuff >>> 18]);
                dest.setByte(destOffset + 1, ALPHABET[(inBuff >>> 12) & 63]);
                dest.setByte(destOffset + 2, 61);
                dest.setByte(destOffset + 3, 61);
                return;
            case 2:
                dest.setByte(destOffset, ALPHABET[inBuff >>> 18]);
                dest.setByte(destOffset + 1, ALPHABET[(inBuff >>> 12) & 63]);
                dest.setByte(destOffset + 2, ALPHABET[(inBuff >>> 6) & 63]);
                dest.setByte(destOffset + 3, 61);
                return;
            case 3:
                dest.setByte(destOffset, ALPHABET[inBuff >>> 18]);
                dest.setByte(destOffset + 1, ALPHABET[(inBuff >>> 12) & 63]);
                dest.setByte(destOffset + 2, ALPHABET[(inBuff >>> 6) & 63]);
                dest.setByte(destOffset + 3, ALPHABET[inBuff & 63]);
                return;
            default:
                return;
        }
    }

    public static ChannelBuffer decode(ChannelBuffer src) {
        return decode(src, Base64Dialect.STANDARD);
    }

    public static ChannelBuffer decode(ChannelBuffer src, Base64Dialect dialect) {
        return decode(src, dialect, HeapChannelBufferFactory.getInstance());
    }

    public static ChannelBuffer decode(ChannelBuffer src, ChannelBufferFactory bufferFactory) {
        return decode(src, Base64Dialect.STANDARD, bufferFactory);
    }

    public static ChannelBuffer decode(ChannelBuffer src, Base64Dialect dialect, ChannelBufferFactory bufferFactory) {
        if (src != null) {
            ChannelBuffer dest = decode(src, src.readerIndex(), src.readableBytes(), dialect, bufferFactory);
            src.readerIndex(src.writerIndex());
            return dest;
        }
        throw new NullPointerException("src");
    }

    public static ChannelBuffer decode(ChannelBuffer src, int off, int len) {
        return decode(src, off, len, Base64Dialect.STANDARD);
    }

    public static ChannelBuffer decode(ChannelBuffer src, int off, int len, Base64Dialect dialect) {
        return decode(src, off, len, dialect, HeapChannelBufferFactory.getInstance());
    }

    public static ChannelBuffer decode(ChannelBuffer src, int off, int len, ChannelBufferFactory bufferFactory) {
        return decode(src, off, len, Base64Dialect.STANDARD, bufferFactory);
    }

    public static ChannelBuffer decode(ChannelBuffer src, int off, int len, Base64Dialect dialect, ChannelBufferFactory bufferFactory) {
        Base64Dialect base64Dialect = dialect;
        ChannelBufferFactory channelBufferFactory = bufferFactory;
        if (src == null) {
            throw new NullPointerException("src");
        } else if (base64Dialect == null) {
            throw new NullPointerException("dialect");
        } else if (channelBufferFactory != null) {
            byte[] DECODABET = decodabet(dialect);
            ChannelBuffer dest = channelBufferFactory.getBuffer(src.order(), (len * 3) / 4);
            int outBuffPosn = 0;
            byte[] b4 = new byte[4];
            int b4Posn = 0;
            int i = off;
            while (i < off + len) {
                byte sbiCrop = (byte) (src.getByte(i) & Ascii.DEL);
                byte sbiDecode = DECODABET[sbiCrop];
                if (sbiDecode >= -5) {
                    if (sbiDecode >= -1) {
                        int b4Posn2 = b4Posn + 1;
                        b4[b4Posn] = sbiCrop;
                        if (b4Posn2 > 3) {
                            outBuffPosn += decode4to3(b4, 0, dest, outBuffPosn, base64Dialect);
                            b4Posn = 0;
                            if (sbiCrop == 61) {
                                break;
                            }
                        } else {
                            b4Posn = b4Posn2;
                        }
                    }
                    i++;
                } else {
                    throw new IllegalArgumentException("bad Base64 input character at " + i + ": " + src.getUnsignedByte(i) + " (decimal)");
                }
            }
            return dest.slice(0, outBuffPosn);
        } else {
            throw new NullPointerException("bufferFactory");
        }
    }

    private static int decode4to3(byte[] src, int srcOffset, ChannelBuffer dest, int destOffset, Base64Dialect dialect) {
        byte[] DECODABET = decodabet(dialect);
        if (src[srcOffset + 2] == 61) {
            dest.setByte(destOffset, (byte) ((((DECODABET[src[srcOffset]] & 255) << Ascii.DC2) | ((DECODABET[src[srcOffset + 1]] & 255) << Ascii.FF)) >>> 16));
            return 1;
        } else if (src[srcOffset + 3] == 61) {
            int outBuff = ((DECODABET[src[srcOffset]] & 255) << Ascii.DC2) | ((DECODABET[src[srcOffset + 1]] & 255) << Ascii.FF) | ((DECODABET[src[srcOffset + 2]] & 255) << 6);
            dest.setByte(destOffset, (byte) (outBuff >>> 16));
            dest.setByte(destOffset + 1, (byte) (outBuff >>> 8));
            return 2;
        } else {
            try {
                int outBuff2 = ((DECODABET[src[srcOffset]] & 255) << Ascii.DC2) | ((DECODABET[src[srcOffset + 1]] & 255) << Ascii.FF) | ((DECODABET[src[srcOffset + 2]] & 255) << 6) | (DECODABET[src[srcOffset + 3]] & 255);
                dest.setByte(destOffset, (byte) (outBuff2 >> 16));
                dest.setByte(destOffset + 1, (byte) (outBuff2 >> 8));
                dest.setByte(destOffset + 2, (byte) outBuff2);
                return 3;
            } catch (IndexOutOfBoundsException e) {
                throw new IllegalArgumentException("not encoded in Base64");
            }
        }
    }

    private Base64() {
    }
}
