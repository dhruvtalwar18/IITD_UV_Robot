package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.DoubleBuffer;

public class DoublePointer extends Pointer {
    private native void allocateArray(long j);

    public native double get(long j);

    public native DoublePointer get(double[] dArr, int i, int i2);

    public native DoublePointer put(long j, double d);

    public native DoublePointer put(double[] dArr, int i, int i2);

    public DoublePointer(double... array) {
        this((long) array.length);
        put(array);
    }

    public DoublePointer(DoubleBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            double[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public DoublePointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new DoublePointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public DoublePointer() {
    }

    public DoublePointer(Pointer p) {
        super(p);
    }

    public DoublePointer position(long position) {
        return (DoublePointer) super.position(position);
    }

    public DoublePointer limit(long limit) {
        return (DoublePointer) super.limit(limit);
    }

    public DoublePointer capacity(long capacity) {
        return (DoublePointer) super.capacity(capacity);
    }

    public double get() {
        return get(0);
    }

    public DoublePointer put(double d) {
        return put(0, d);
    }

    public DoublePointer get(double[] array) {
        return get(array, 0, array.length);
    }

    public DoublePointer put(double... array) {
        return put(array, 0, array.length);
    }

    public final DoubleBuffer asBuffer() {
        return asByteBuffer().asDoubleBuffer();
    }
}
