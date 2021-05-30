package org.apache.ws.commons.util;

import com.google.common.base.Ascii;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.StringWriter;
import java.io.Writer;
import java.lang.reflect.UndeclaredThrowableException;
import org.apache.commons.io.IOUtils;
import org.jboss.netty.handler.codec.http.HttpConstants;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;
import rocon_app_manager_msgs.ErrorCodes;

public class Base64 {
    public static final String LINE_SEPARATOR = "\n";
    public static final int LINE_SIZE = 76;
    /* access modifiers changed from: private */
    public static final byte[] base64ToInt = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, HttpConstants.COLON, HttpConstants.SEMICOLON, 60, HttpConstants.EQUALS, -1, -1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI, 16, 17, Ascii.DC2, 19, 20, 21, Ascii.SYN, Ascii.ETB, Ascii.CAN, Ascii.EM, -1, -1, -1, -1, -1, -1, Ascii.SUB, Ascii.ESC, Ascii.FS, Ascii.GS, 30, 31, 32, ErrorCodes.NOT_CURRENT_REMOTE_CONTROLLER, 34, ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, 37, 38, 39, 40, 41, 42, 43, HttpConstants.COMMA, 45, 46, 47, 48, 49, 50, 51};
    /* access modifiers changed from: private */
    public static final char[] intToBase64 = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', IOUtils.DIR_SEPARATOR_UNIX};

    public static class DecodingException extends IOException {
        private static final long serialVersionUID = 3257006574836135478L;

        DecodingException(String pMessage) {
            super(pMessage);
        }
    }

    public static class SAXIOException extends IOException {
        private static final long serialVersionUID = 3258131345216451895L;
        final SAXException saxException;

        SAXIOException(SAXException e) {
            this.saxException = e;
        }

        public SAXException getSAXException() {
            return this.saxException;
        }
    }

    public static abstract class Encoder {
        private final char[] charBuffer;
        private int charOffset;
        private int lineChars = 0;
        private int num;
        private int numBytes;
        private final String sep;
        private final int skipChars;
        private final int wrapSize;

        /* access modifiers changed from: protected */
        public abstract void writeBuffer(char[] cArr, int i, int i2) throws IOException;

        protected Encoder(char[] pBuffer, int pWrapSize, String pSep) {
            int i = 0;
            this.charBuffer = pBuffer;
            this.sep = pSep == null ? null : "\n";
            this.skipChars = pWrapSize == 0 ? 4 : this.sep.length() + 4;
            this.wrapSize = this.skipChars != 4 ? pWrapSize : i;
            if (this.wrapSize < 0 || this.wrapSize % 4 > 0) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Illegal argument for wrap size: ");
                stringBuffer.append(pWrapSize);
                stringBuffer.append("(Expected nonnegative multiple of 4)");
                throw new IllegalArgumentException(stringBuffer.toString());
            } else if (pBuffer.length < this.skipChars) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("The buffer must contain at least ");
                stringBuffer2.append(this.skipChars);
                stringBuffer2.append(" characters, but has ");
                stringBuffer2.append(pBuffer.length);
                throw new IllegalArgumentException(stringBuffer2.toString());
            }
        }

        private void wrap() {
            for (int j = 0; j < this.sep.length(); j++) {
                char[] cArr = this.charBuffer;
                int i = this.charOffset;
                this.charOffset = i + 1;
                cArr[i] = this.sep.charAt(j);
            }
            this.lineChars = 0;
        }

        /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v3, resolved type: byte} */
        /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v6, resolved type: byte} */
        /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v7, resolved type: byte} */
        /* JADX WARNING: Multi-variable type inference failed */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public void write(byte[] r8, int r9, int r10) throws java.io.IOException {
            /*
                r7 = this;
                r0 = 0
                r1 = r9
                r9 = 0
            L_0x0003:
                if (r9 >= r10) goto L_0x00a0
                int r2 = r1 + 1
                byte r1 = r8[r1]
                if (r1 >= 0) goto L_0x000d
                int r1 = r1 + 256
            L_0x000d:
                int r3 = r7.num
                int r3 = r3 << 8
                int r3 = r3 + r1
                r7.num = r3
                int r3 = r7.numBytes
                int r3 = r3 + 1
                r7.numBytes = r3
                r4 = 3
                if (r3 != r4) goto L_0x009b
                char[] r3 = r7.charBuffer
                int r4 = r7.charOffset
                int r5 = r4 + 1
                r7.charOffset = r5
                char[] r5 = org.apache.ws.commons.util.Base64.intToBase64
                int r6 = r7.num
                int r6 = r6 >> 18
                char r5 = r5[r6]
                r3[r4] = r5
                char[] r3 = r7.charBuffer
                int r4 = r7.charOffset
                int r5 = r4 + 1
                r7.charOffset = r5
                char[] r5 = org.apache.ws.commons.util.Base64.intToBase64
                int r6 = r7.num
                int r6 = r6 >> 12
                r6 = r6 & 63
                char r5 = r5[r6]
                r3[r4] = r5
                char[] r3 = r7.charBuffer
                int r4 = r7.charOffset
                int r5 = r4 + 1
                r7.charOffset = r5
                char[] r5 = org.apache.ws.commons.util.Base64.intToBase64
                int r6 = r7.num
                int r6 = r6 >> 6
                r6 = r6 & 63
                char r5 = r5[r6]
                r3[r4] = r5
                char[] r3 = r7.charBuffer
                int r4 = r7.charOffset
                int r5 = r4 + 1
                r7.charOffset = r5
                char[] r5 = org.apache.ws.commons.util.Base64.intToBase64
                int r6 = r7.num
                r6 = r6 & 63
                char r5 = r5[r6]
                r3[r4] = r5
                int r3 = r7.wrapSize
                if (r3 <= 0) goto L_0x0084
                int r3 = r7.lineChars
                int r3 = r3 + 4
                r7.lineChars = r3
                int r3 = r7.lineChars
                int r4 = r7.wrapSize
                if (r3 < r4) goto L_0x0084
                r7.wrap()
            L_0x0084:
                r7.num = r0
                r7.numBytes = r0
                int r3 = r7.charOffset
                int r4 = r7.skipChars
                int r3 = r3 + r4
                char[] r4 = r7.charBuffer
                int r4 = r4.length
                if (r3 <= r4) goto L_0x009b
                char[] r3 = r7.charBuffer
                int r4 = r7.charOffset
                r7.writeBuffer(r3, r0, r4)
                r7.charOffset = r0
            L_0x009b:
                int r9 = r9 + 1
                r1 = r2
                goto L_0x0003
            L_0x00a0:
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: org.apache.ws.commons.util.Base64.Encoder.write(byte[], int, int):void");
        }

        public void flush() throws IOException {
            if (this.numBytes > 0) {
                if (this.numBytes == 1) {
                    char[] cArr = this.charBuffer;
                    int i = this.charOffset;
                    this.charOffset = i + 1;
                    cArr[i] = Base64.intToBase64[this.num >> 2];
                    char[] cArr2 = this.charBuffer;
                    int i2 = this.charOffset;
                    this.charOffset = i2 + 1;
                    cArr2[i2] = Base64.intToBase64[(this.num << 4) & 63];
                    char[] cArr3 = this.charBuffer;
                    int i3 = this.charOffset;
                    this.charOffset = i3 + 1;
                    cArr3[i3] = '=';
                    char[] cArr4 = this.charBuffer;
                    int i4 = this.charOffset;
                    this.charOffset = i4 + 1;
                    cArr4[i4] = '=';
                } else {
                    char[] cArr5 = this.charBuffer;
                    int i5 = this.charOffset;
                    this.charOffset = i5 + 1;
                    cArr5[i5] = Base64.intToBase64[this.num >> 10];
                    char[] cArr6 = this.charBuffer;
                    int i6 = this.charOffset;
                    this.charOffset = i6 + 1;
                    cArr6[i6] = Base64.intToBase64[(this.num >> 4) & 63];
                    char[] cArr7 = this.charBuffer;
                    int i7 = this.charOffset;
                    this.charOffset = i7 + 1;
                    cArr7[i7] = Base64.intToBase64[(this.num << 2) & 63];
                    char[] cArr8 = this.charBuffer;
                    int i8 = this.charOffset;
                    this.charOffset = i8 + 1;
                    cArr8[i8] = '=';
                }
                this.lineChars += 4;
                this.num = 0;
                this.numBytes = 0;
            }
            if (this.wrapSize > 0 && this.lineChars > 0) {
                wrap();
            }
            if (this.charOffset > 0) {
                writeBuffer(this.charBuffer, 0, this.charOffset);
                this.charOffset = 0;
            }
        }
    }

    public static class EncoderOutputStream extends OutputStream {
        private final Encoder encoder;
        private final byte[] oneByte = new byte[1];

        public EncoderOutputStream(Encoder pEncoder) {
            this.encoder = pEncoder;
        }

        public void write(int b) throws IOException {
            this.oneByte[0] = (byte) b;
            this.encoder.write(this.oneByte, 0, 1);
        }

        public void write(byte[] pBuffer, int pOffset, int pLen) throws IOException {
            this.encoder.write(pBuffer, pOffset, pLen);
        }

        public void close() throws IOException {
            this.encoder.flush();
        }
    }

    public static OutputStream newEncoder(Writer pWriter) {
        return newEncoder(pWriter, 76, "\n");
    }

    public static OutputStream newEncoder(final Writer pWriter, int pLineSize, String pSeparator) {
        return new EncoderOutputStream(new Encoder(new char[4096], pLineSize, pSeparator) {
            /* access modifiers changed from: protected */
            public void writeBuffer(char[] pBuffer, int pOffset, int pLen) throws IOException {
                pWriter.write(pBuffer, pOffset, pLen);
            }
        });
    }

    public static class SAXEncoder extends Encoder {
        private final ContentHandler handler;

        public SAXEncoder(char[] pBuffer, int pWrapSize, String pSep, ContentHandler pHandler) {
            super(pBuffer, pWrapSize, pSep);
            this.handler = pHandler;
        }

        /* access modifiers changed from: protected */
        public void writeBuffer(char[] pChars, int pOffset, int pLen) throws IOException {
            try {
                this.handler.characters(pChars, pOffset, pLen);
            } catch (SAXException e) {
                throw new SAXIOException(e);
            }
        }
    }

    public static String encode(byte[] pBuffer, int pOffset, int pLength) {
        return encode(pBuffer, pOffset, pLength, 76, "\n");
    }

    public static String encode(byte[] pBuffer, int pOffset, int pLength, int pLineSize, String pSeparator) {
        StringWriter sw = new StringWriter();
        OutputStream ostream = newEncoder(sw, pLineSize, pSeparator);
        try {
            ostream.write(pBuffer, pOffset, pLength);
            ostream.close();
            return sw.toString();
        } catch (IOException e) {
            throw new UndeclaredThrowableException(e);
        }
    }

    public static String encode(byte[] pBuffer) {
        return encode(pBuffer, 0, pBuffer.length);
    }

    public static abstract class Decoder {
        private final byte[] byteBuffer;
        private int byteBufferOffset;
        private int eofBytes;
        private int num;
        private int numBytes;

        /* access modifiers changed from: protected */
        public abstract void writeBuffer(byte[] bArr, int i, int i2) throws IOException;

        protected Decoder(int pBufLen) {
            this.byteBuffer = new byte[pBufLen];
        }

        public void write(char[] pData, int pOffset, int pLen) throws IOException {
            byte result;
            int pOffset2 = pOffset;
            int i = 0;
            while (i < pLen) {
                int pOffset3 = pOffset2 + 1;
                char pOffset4 = pData[pOffset2];
                if (!Character.isWhitespace(pOffset4)) {
                    if (pOffset4 == '=') {
                        this.eofBytes++;
                        this.num <<= 6;
                        int i2 = this.numBytes + 1;
                        this.numBytes = i2;
                        switch (i2) {
                            case 1:
                            case 2:
                                throw new DecodingException("Unexpected end of stream character (=)");
                            case 3:
                                break;
                            case 4:
                                byte[] bArr = this.byteBuffer;
                                int i3 = this.byteBufferOffset;
                                this.byteBufferOffset = i3 + 1;
                                bArr[i3] = (byte) (this.num >> 16);
                                if (this.eofBytes == 1) {
                                    byte[] bArr2 = this.byteBuffer;
                                    int i4 = this.byteBufferOffset;
                                    this.byteBufferOffset = i4 + 1;
                                    bArr2[i4] = (byte) (this.num >> 8);
                                }
                                writeBuffer(this.byteBuffer, 0, this.byteBufferOffset);
                                this.byteBufferOffset = 0;
                                break;
                            case 5:
                                throw new DecodingException("Trailing garbage detected");
                            default:
                                throw new IllegalStateException("Invalid value for numBytes");
                        }
                    } else if (this.eofBytes > 0) {
                        throw new DecodingException("Base64 characters after end of stream character (=) detected.");
                    } else if (pOffset4 >= 0 && pOffset4 < Base64.base64ToInt.length && (result = Base64.base64ToInt[pOffset4]) >= 0) {
                        this.num = (this.num << 6) + result;
                        int i5 = this.numBytes + 1;
                        this.numBytes = i5;
                        if (i5 == 4) {
                            byte[] bArr3 = this.byteBuffer;
                            int i6 = this.byteBufferOffset;
                            this.byteBufferOffset = i6 + 1;
                            bArr3[i6] = (byte) (this.num >> 16);
                            byte[] bArr4 = this.byteBuffer;
                            int i7 = this.byteBufferOffset;
                            this.byteBufferOffset = i7 + 1;
                            bArr4[i7] = (byte) ((this.num >> 8) & 255);
                            byte[] bArr5 = this.byteBuffer;
                            int i8 = this.byteBufferOffset;
                            this.byteBufferOffset = i8 + 1;
                            bArr5[i8] = (byte) (this.num & 255);
                            if (this.byteBufferOffset + 3 > this.byteBuffer.length) {
                                writeBuffer(this.byteBuffer, 0, this.byteBufferOffset);
                                this.byteBufferOffset = 0;
                            }
                            this.num = 0;
                            this.numBytes = 0;
                        }
                    } else if (Character.isWhitespace(pOffset4) == 0) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Invalid Base64 character: ");
                        stringBuffer.append(pOffset4);
                        throw new DecodingException(stringBuffer.toString());
                    }
                }
                i++;
                pOffset2 = pOffset3;
            }
        }

        public void flush() throws IOException {
            if (this.numBytes != 0 && this.numBytes != 4) {
                throw new DecodingException("Unexpected end of file");
            } else if (this.byteBufferOffset > 0) {
                writeBuffer(this.byteBuffer, 0, this.byteBufferOffset);
                this.byteBufferOffset = 0;
            }
        }
    }

    public Writer newDecoder(final OutputStream pStream) {
        return new Writer() {
            private final Decoder decoder = new Decoder(this, 1024) {
                private final /* synthetic */ AnonymousClass2 this$1;

                {
                    this.this$1 = r1;
                }

                /* access modifiers changed from: protected */
                public void writeBuffer(byte[] pBytes, int pOffset, int pLen) throws IOException {
                    pStream.write(pBytes, pOffset, pLen);
                }
            };

            public void close() throws IOException {
                flush();
            }

            public void flush() throws IOException {
                this.decoder.flush();
                pStream.flush();
            }

            public void write(char[] cbuf, int off, int len) throws IOException {
                this.decoder.write(cbuf, off, len);
            }
        };
    }

    public static byte[] decode(char[] pBuffer, int pOffset, int pLength) throws DecodingException {
        final ByteArrayOutputStream baos = new ByteArrayOutputStream();
        Decoder d = new Decoder(1024) {
            /* access modifiers changed from: protected */
            public void writeBuffer(byte[] pBuf, int pOff, int pLen) throws IOException {
                baos.write(pBuf, pOff, pLen);
            }
        };
        try {
            d.write(pBuffer, pOffset, pLength);
            d.flush();
            return baos.toByteArray();
        } catch (DecodingException e) {
            throw e;
        } catch (IOException e2) {
            throw new UndeclaredThrowableException(e2);
        }
    }

    public static byte[] decode(char[] pBuffer) throws DecodingException {
        return decode(pBuffer, 0, pBuffer.length);
    }

    public static byte[] decode(String pBuffer) throws DecodingException {
        return decode(pBuffer.toCharArray());
    }
}
