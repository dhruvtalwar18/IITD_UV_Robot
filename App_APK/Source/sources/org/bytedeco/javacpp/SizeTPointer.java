package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Name;

@Name({"size_t"})
public class SizeTPointer extends Pointer {
    private native void allocateArray(long j);

    @Cast({"size_t"})
    public native long get(long j);

    public native SizeTPointer put(long j, long j2);

    public SizeTPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new SizeTPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public SizeTPointer() {
    }

    public SizeTPointer(Pointer p) {
        super(p);
    }

    public SizeTPointer position(long position) {
        return (SizeTPointer) super.position(position);
    }

    public SizeTPointer limit(long limit) {
        return (SizeTPointer) super.limit(limit);
    }

    public SizeTPointer capacity(long capacity) {
        return (SizeTPointer) super.capacity(capacity);
    }

    public long get() {
        return get(0);
    }

    public SizeTPointer put(long s) {
        return put(0, s);
    }
}
