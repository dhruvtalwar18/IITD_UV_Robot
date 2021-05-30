package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.ShortBuffer;

public class ShortPointer extends Pointer {
    private native void allocateArray(long j);

    public native ShortPointer get(short[] sArr, int i, int i2);

    public native short get(long j);

    public native ShortPointer put(long j, short s);

    public native ShortPointer put(short[] sArr, int i, int i2);

    public ShortPointer(short... array) {
        this((long) array.length);
        put(array);
    }

    public ShortPointer(ShortBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            short[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public ShortPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new ShortPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public ShortPointer() {
    }

    public ShortPointer(Pointer p) {
        super(p);
    }

    public ShortPointer position(long position) {
        return (ShortPointer) super.position(position);
    }

    public ShortPointer limit(long limit) {
        return (ShortPointer) super.limit(limit);
    }

    public ShortPointer capacity(long capacity) {
        return (ShortPointer) super.capacity(capacity);
    }

    public short get() {
        return get(0);
    }

    public ShortPointer put(short s) {
        return put(0, s);
    }

    public ShortPointer get(short[] array) {
        return get(array, 0, array.length);
    }

    public ShortPointer put(short... array) {
        return put(array, 0, array.length);
    }

    public final ShortBuffer asBuffer() {
        return asByteBuffer().asShortBuffer();
    }
}
