package com.google.common.io;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import com.google.common.hash.HashCode;
import com.google.common.hash.HashFunction;
import com.google.common.hash.Hasher;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInput;
import java.io.DataInputStream;
import java.io.DataOutput;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.channels.ReadableByteChannel;
import java.nio.channels.WritableByteChannel;
import java.security.MessageDigest;
import java.util.Arrays;
import java.util.zip.Checksum;
import org.xbill.DNS.TTL;

@Beta
public final class ByteStreams {
    private static final int BUF_SIZE = 4096;

    private ByteStreams() {
    }

    public static InputSupplier<ByteArrayInputStream> newInputStreamSupplier(byte[] b) {
        return newInputStreamSupplier(b, 0, b.length);
    }

    public static InputSupplier<ByteArrayInputStream> newInputStreamSupplier(final byte[] b, final int off, final int len) {
        return new InputSupplier<ByteArrayInputStream>() {
            public ByteArrayInputStream getInput() {
                return new ByteArrayInputStream(b, off, len);
            }
        };
    }

    public static void write(byte[] from, OutputSupplier<? extends OutputStream> to) throws IOException {
        Preconditions.checkNotNull(from);
        boolean threw = true;
        OutputStream out = (OutputStream) to.getOutput();
        try {
            out.write(from);
            threw = false;
        } finally {
            Closeables.close(out, threw);
        }
    }

    /* JADX WARNING: Unknown top exception splitter block from list: {B:9:0x001c=Splitter:B:9:0x001c, B:20:0x002e=Splitter:B:20:0x002e} */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static long copy(com.google.common.io.InputSupplier<? extends java.io.InputStream> r9, com.google.common.io.OutputSupplier<? extends java.io.OutputStream> r10) throws java.io.IOException {
        /*
            r0 = 0
            java.lang.Object r1 = r9.getInput()
            java.io.InputStream r1 = (java.io.InputStream) r1
            r2 = 2
            r3 = 0
            r4 = 1
            java.lang.Object r5 = r10.getOutput()     // Catch:{ all -> 0x0034 }
            java.io.OutputStream r5 = (java.io.OutputStream) r5     // Catch:{ all -> 0x0034 }
            long r6 = copy((java.io.InputStream) r1, (java.io.OutputStream) r5)     // Catch:{ all -> 0x0028 }
            int r0 = r0 + 1
            if (r0 >= r4) goto L_0x001b
            r8 = 1
            goto L_0x001c
        L_0x001b:
            r8 = 0
        L_0x001c:
            com.google.common.io.Closeables.close(r5, r8)     // Catch:{ all -> 0x0034 }
            int r0 = r0 + r4
            if (r0 >= r2) goto L_0x0024
            r3 = 1
        L_0x0024:
            com.google.common.io.Closeables.close(r1, r3)
            return r6
        L_0x0028:
            r6 = move-exception
            if (r0 >= r4) goto L_0x002d
            r7 = 1
            goto L_0x002e
        L_0x002d:
            r7 = 0
        L_0x002e:
            com.google.common.io.Closeables.close(r5, r7)     // Catch:{ all -> 0x0034 }
            int r0 = r0 + 1
            throw r6     // Catch:{ all -> 0x0034 }
        L_0x0034:
            r5 = move-exception
            if (r0 >= r2) goto L_0x0039
            r3 = 1
        L_0x0039:
            com.google.common.io.Closeables.close(r1, r3)
            throw r5
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.io.ByteStreams.copy(com.google.common.io.InputSupplier, com.google.common.io.OutputSupplier):long");
    }

    public static long copy(InputSupplier<? extends InputStream> from, OutputStream to) throws IOException {
        boolean threw = true;
        InputStream in = (InputStream) from.getInput();
        try {
            threw = false;
            return copy(in, to);
        } finally {
            Closeables.close(in, threw);
        }
    }

    public static long copy(InputStream from, OutputSupplier<? extends OutputStream> to) throws IOException {
        boolean threw = true;
        OutputStream out = (OutputStream) to.getOutput();
        try {
            threw = false;
            return copy(from, out);
        } finally {
            Closeables.close(out, threw);
        }
    }

    public static long copy(InputStream from, OutputStream to) throws IOException {
        byte[] buf = new byte[4096];
        long total = 0;
        while (true) {
            int r = from.read(buf);
            if (r == -1) {
                return total;
            }
            to.write(buf, 0, r);
            total += (long) r;
        }
    }

    public static long copy(ReadableByteChannel from, WritableByteChannel to) throws IOException {
        ByteBuffer buf = ByteBuffer.allocate(4096);
        long total = 0;
        while (from.read(buf) != -1) {
            buf.flip();
            while (buf.hasRemaining()) {
                total += (long) to.write(buf);
            }
            buf.clear();
        }
        return total;
    }

    public static byte[] toByteArray(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        copy(in, (OutputStream) out);
        return out.toByteArray();
    }

    public static byte[] toByteArray(InputSupplier<? extends InputStream> supplier) throws IOException {
        boolean threw = true;
        InputStream in = (InputStream) supplier.getInput();
        try {
            threw = false;
            return toByteArray(in);
        } finally {
            Closeables.close(in, threw);
        }
    }

    public static ByteArrayDataInput newDataInput(byte[] bytes) {
        return new ByteArrayDataInputStream(bytes);
    }

    public static ByteArrayDataInput newDataInput(byte[] bytes, int start) {
        Preconditions.checkPositionIndex(start, bytes.length);
        return new ByteArrayDataInputStream(bytes, start);
    }

    private static class ByteArrayDataInputStream implements ByteArrayDataInput {
        final DataInput input;

        ByteArrayDataInputStream(byte[] bytes) {
            this.input = new DataInputStream(new ByteArrayInputStream(bytes));
        }

        ByteArrayDataInputStream(byte[] bytes, int start) {
            this.input = new DataInputStream(new ByteArrayInputStream(bytes, start, bytes.length - start));
        }

        public void readFully(byte[] b) {
            try {
                this.input.readFully(b);
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public void readFully(byte[] b, int off, int len) {
            try {
                this.input.readFully(b, off, len);
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public int skipBytes(int n) {
            try {
                return this.input.skipBytes(n);
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public boolean readBoolean() {
            try {
                return this.input.readBoolean();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public byte readByte() {
            try {
                return this.input.readByte();
            } catch (EOFException e) {
                throw new IllegalStateException(e);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public int readUnsignedByte() {
            try {
                return this.input.readUnsignedByte();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public short readShort() {
            try {
                return this.input.readShort();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public int readUnsignedShort() {
            try {
                return this.input.readUnsignedShort();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public char readChar() {
            try {
                return this.input.readChar();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public int readInt() {
            try {
                return this.input.readInt();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public long readLong() {
            try {
                return this.input.readLong();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public float readFloat() {
            try {
                return this.input.readFloat();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public double readDouble() {
            try {
                return this.input.readDouble();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public String readLine() {
            try {
                return this.input.readLine();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }

        public String readUTF() {
            try {
                return this.input.readUTF();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        }
    }

    public static ByteArrayDataOutput newDataOutput() {
        return new ByteArrayDataOutputStream();
    }

    public static ByteArrayDataOutput newDataOutput(int size) {
        Preconditions.checkArgument(size >= 0, "Invalid size: %s", Integer.valueOf(size));
        return new ByteArrayDataOutputStream(size);
    }

    private static class ByteArrayDataOutputStream implements ByteArrayDataOutput {
        final ByteArrayOutputStream byteArrayOutputSteam;
        final DataOutput output;

        ByteArrayDataOutputStream() {
            this(new ByteArrayOutputStream());
        }

        ByteArrayDataOutputStream(int size) {
            this(new ByteArrayOutputStream(size));
        }

        ByteArrayDataOutputStream(ByteArrayOutputStream byteArrayOutputSteam2) {
            this.byteArrayOutputSteam = byteArrayOutputSteam2;
            this.output = new DataOutputStream(byteArrayOutputSteam2);
        }

        public void write(int b) {
            try {
                this.output.write(b);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void write(byte[] b) {
            try {
                this.output.write(b);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void write(byte[] b, int off, int len) {
            try {
                this.output.write(b, off, len);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeBoolean(boolean v) {
            try {
                this.output.writeBoolean(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeByte(int v) {
            try {
                this.output.writeByte(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeBytes(String s) {
            try {
                this.output.writeBytes(s);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeChar(int v) {
            try {
                this.output.writeChar(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeChars(String s) {
            try {
                this.output.writeChars(s);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeDouble(double v) {
            try {
                this.output.writeDouble(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeFloat(float v) {
            try {
                this.output.writeFloat(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeInt(int v) {
            try {
                this.output.writeInt(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeLong(long v) {
            try {
                this.output.writeLong(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeShort(int v) {
            try {
                this.output.writeShort(v);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public void writeUTF(String s) {
            try {
                this.output.writeUTF(s);
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        public byte[] toByteArray() {
            return this.byteArrayOutputSteam.toByteArray();
        }
    }

    public static long length(InputSupplier<? extends InputStream> supplier) throws IOException {
        long count = 0;
        boolean threw = true;
        InputStream in = (InputStream) supplier.getInput();
        while (true) {
            try {
                long amt = in.skip(TTL.MAX_VALUE);
                if (amt != 0) {
                    count += amt;
                } else if (in.read() == -1) {
                    threw = false;
                    return count;
                } else {
                    count++;
                }
            } finally {
                Closeables.close(in, threw);
            }
        }
    }

    public static boolean equal(InputSupplier<? extends InputStream> supplier1, InputSupplier<? extends InputStream> supplier2) throws IOException {
        int read1;
        byte[] buf1 = new byte[4096];
        byte[] buf2 = new byte[4096];
        boolean threw = true;
        InputStream in2 = (InputStream) supplier1.getInput();
        try {
            in2 = (InputStream) supplier2.getInput();
            do {
                read1 = read(in2, buf1, 0, 4096);
                if (read1 != read(in2, buf2, 0, 4096) || !Arrays.equals(buf1, buf2)) {
                    threw = false;
                    Closeables.close(in2, threw);
                    Closeables.close(in2, threw);
                    return false;
                }
            } while (read1 == 4096);
            threw = false;
            return true;
        } catch (Throwable th) {
            throw th;
        } finally {
            Closeables.close(in2, threw);
        }
    }

    public static void readFully(InputStream in, byte[] b) throws IOException {
        readFully(in, b, 0, b.length);
    }

    public static void readFully(InputStream in, byte[] b, int off, int len) throws IOException {
        if (read(in, b, off, len) != len) {
            throw new EOFException();
        }
    }

    public static void skipFully(InputStream in, long n) throws IOException {
        while (n > 0) {
            long amt = in.skip(n);
            if (amt != 0) {
                n -= amt;
            } else if (in.read() != -1) {
                n--;
            } else {
                throw new EOFException();
            }
        }
    }

    public static <T> T readBytes(InputSupplier<? extends InputStream> supplier, ByteProcessor<T> processor) throws IOException {
        byte[] buf = new byte[4096];
        boolean threw = true;
        InputStream in = (InputStream) supplier.getInput();
        while (true) {
            try {
                int amt = in.read(buf);
                if (amt != -1) {
                    if (!processor.processBytes(buf, 0, amt)) {
                        break;
                    }
                } else {
                    threw = false;
                    break;
                }
            } finally {
                Closeables.close(in, threw);
            }
        }
        return processor.getResult();
    }

    public static long getChecksum(InputSupplier<? extends InputStream> supplier, final Checksum checksum) throws IOException {
        return ((Long) readBytes(supplier, new ByteProcessor<Long>() {
            public boolean processBytes(byte[] buf, int off, int len) {
                checksum.update(buf, off, len);
                return true;
            }

            public Long getResult() {
                long result = checksum.getValue();
                checksum.reset();
                return Long.valueOf(result);
            }
        })).longValue();
    }

    @Deprecated
    public static byte[] getDigest(InputSupplier<? extends InputStream> supplier, final MessageDigest md) throws IOException {
        return (byte[]) readBytes(supplier, new ByteProcessor<byte[]>() {
            public boolean processBytes(byte[] buf, int off, int len) {
                md.update(buf, off, len);
                return true;
            }

            public byte[] getResult() {
                return md.digest();
            }
        });
    }

    public static HashCode hash(InputSupplier<? extends InputStream> supplier, HashFunction hashFunction) throws IOException {
        final Hasher hasher = hashFunction.newHasher();
        return (HashCode) readBytes(supplier, new ByteProcessor<HashCode>() {
            public boolean processBytes(byte[] buf, int off, int len) {
                hasher.putBytes(buf, off, len);
                return true;
            }

            public HashCode getResult() {
                return hasher.hash();
            }
        });
    }

    public static int read(InputStream in, byte[] b, int off, int len) throws IOException {
        if (len >= 0) {
            int total = 0;
            while (total < len) {
                int result = in.read(b, off + total, len - total);
                if (result == -1) {
                    break;
                }
                total += result;
            }
            return total;
        }
        throw new IndexOutOfBoundsException("len is negative");
    }

    public static InputSupplier<InputStream> slice(InputSupplier<? extends InputStream> supplier, long offset, long length) {
        Preconditions.checkNotNull(supplier);
        boolean z = false;
        Preconditions.checkArgument(offset >= 0, "offset is negative");
        if (length >= 0) {
            z = true;
        }
        Preconditions.checkArgument(z, "length is negative");
        final InputSupplier<? extends InputStream> inputSupplier = supplier;
        final long j = offset;
        final long j2 = length;
        return new InputSupplier<InputStream>() {
            public InputStream getInput() throws IOException {
                InputStream in = (InputStream) inputSupplier.getInput();
                if (j > 0) {
                    try {
                        ByteStreams.skipFully(in, j);
                    } catch (IOException e) {
                        Closeables.closeQuietly(in);
                        throw e;
                    }
                }
                return new LimitInputStream(in, j2);
            }
        };
    }

    public static InputSupplier<InputStream> join(final Iterable<? extends InputSupplier<? extends InputStream>> suppliers) {
        return new InputSupplier<InputStream>() {
            public InputStream getInput() throws IOException {
                return new MultiInputStream(suppliers.iterator());
            }
        };
    }

    public static InputSupplier<InputStream> join(InputSupplier<? extends InputStream>... suppliers) {
        return join((Iterable<? extends InputSupplier<? extends InputStream>>) Arrays.asList(suppliers));
    }
}
