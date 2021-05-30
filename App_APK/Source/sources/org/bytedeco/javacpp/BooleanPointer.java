package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.ByteBuffer;

public class BooleanPointer extends Pointer {
    private native void allocateArray(long j);

    public native BooleanPointer get(boolean[] zArr, int i, int i2);

    public native boolean get(long j);

    public native BooleanPointer put(long j, boolean z);

    public native BooleanPointer put(boolean[] zArr, int i, int i2);

    public BooleanPointer(boolean... array) {
        this((long) array.length);
        put(array);
    }

    public BooleanPointer(ByteBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            byte[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            for (int i = buffer.arrayOffset(); i < array.length; i++) {
                put((long) (i - buffer.arrayOffset()), array[i] != 0);
            }
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public BooleanPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new BooleanPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public BooleanPointer() {
    }

    public BooleanPointer(Pointer p) {
        super(p);
    }

    public BooleanPointer position(long position) {
        return (BooleanPointer) super.position(position);
    }

    public BooleanPointer limit(long limit) {
        return (BooleanPointer) super.limit(limit);
    }

    public BooleanPointer capacity(long capacity) {
        return (BooleanPointer) super.capacity(capacity);
    }

    public boolean get() {
        return get(0);
    }

    public BooleanPointer put(boolean b) {
        return put(0, b);
    }

    public BooleanPointer get(boolean[] array) {
        return get(array, 0, array.length);
    }

    public BooleanPointer put(boolean... array) {
        return put(array, 0, array.length);
    }
}
