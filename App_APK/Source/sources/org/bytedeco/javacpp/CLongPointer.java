package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Name;

@Name({"long"})
public class CLongPointer extends Pointer {
    private native void allocateArray(long j);

    @Cast({"long"})
    public native long get(long j);

    public native CLongPointer put(long j, long j2);

    public CLongPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new CLongPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public CLongPointer() {
    }

    public CLongPointer(Pointer p) {
        super(p);
    }

    public CLongPointer position(long position) {
        return (CLongPointer) super.position(position);
    }

    public CLongPointer limit(long limit) {
        return (CLongPointer) super.limit(limit);
    }

    public CLongPointer capacity(long capacity) {
        return (CLongPointer) super.capacity(capacity);
    }

    public long get() {
        return get(0);
    }

    public CLongPointer put(long l) {
        return put(0, l);
    }
}
