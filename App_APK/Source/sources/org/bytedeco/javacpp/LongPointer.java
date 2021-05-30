package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.LongBuffer;

public class LongPointer extends Pointer {
    private native void allocateArray(long j);

    public native long get(long j);

    public native LongPointer get(long[] jArr, int i, int i2);

    public native LongPointer put(long j, long j2);

    public native LongPointer put(long[] jArr, int i, int i2);

    public LongPointer(long... array) {
        this((long) array.length);
        put(array);
    }

    public LongPointer(LongBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            long[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public LongPointer(long size) {
        try {
            allocateArray(size);
            if (size <= 0) {
                return;
            }
            if (this.address == 0) {
                throw new OutOfMemoryError("Native allocator returned address == 0");
            }
        } catch (UnsatisfiedLinkError e) {
            throw new RuntimeException("No native JavaCPP library in memory. (Has Loader.load() been called?)", e);
        } catch (OutOfMemoryError e2) {
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new LongPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public LongPointer() {
    }

    public LongPointer(Pointer p) {
        super(p);
    }

    public LongPointer position(long position) {
        return (LongPointer) super.position(position);
    }

    public LongPointer limit(long limit) {
        return (LongPointer) super.limit(limit);
    }

    public LongPointer capacity(long capacity) {
        return (LongPointer) super.capacity(capacity);
    }

    public long get() {
        return get(0);
    }

    public LongPointer put(long l) {
        return put(0, l);
    }

    public LongPointer get(long[] array) {
        return get(array, 0, array.length);
    }

    public LongPointer put(long... array) {
        return put(array, 0, array.length);
    }

    public final LongBuffer asBuffer() {
        return asByteBuffer().asLongBuffer();
    }
}
