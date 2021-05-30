package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.FloatBuffer;

public class FloatPointer extends Pointer {
    private native void allocateArray(long j);

    public native float get(long j);

    public native FloatPointer get(float[] fArr, int i, int i2);

    public native FloatPointer put(long j, float f);

    public native FloatPointer put(float[] fArr, int i, int i2);

    public FloatPointer(float... array) {
        this((long) array.length);
        put(array);
    }

    public FloatPointer(FloatBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            float[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public FloatPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new FloatPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public FloatPointer() {
    }

    public FloatPointer(Pointer p) {
        super(p);
    }

    public FloatPointer position(long position) {
        return (FloatPointer) super.position(position);
    }

    public FloatPointer limit(long limit) {
        return (FloatPointer) super.limit(limit);
    }

    public FloatPointer capacity(long capacity) {
        return (FloatPointer) super.capacity(capacity);
    }

    public float get() {
        return get(0);
    }

    public FloatPointer put(float f) {
        return put(0, f);
    }

    public FloatPointer get(float[] array) {
        return get(array, 0, array.length);
    }

    public FloatPointer put(float... array) {
        return put(array, 0, array.length);
    }

    public final FloatBuffer asBuffer() {
        return asByteBuffer().asFloatBuffer();
    }
}
