package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Name;

@Name({"bool"})
public class BoolPointer extends Pointer {
    private native void allocateArray(long j);

    @Cast({"bool"})
    public native boolean get(long j);

    public native BoolPointer put(long j, boolean z);

    public BoolPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new BoolPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public BoolPointer() {
    }

    public BoolPointer(Pointer p) {
        super(p);
    }

    public BoolPointer position(long position) {
        return (BoolPointer) super.position(position);
    }

    public BoolPointer limit(long limit) {
        return (BoolPointer) super.limit(limit);
    }

    public BoolPointer capacity(long capacity) {
        return (BoolPointer) super.capacity(capacity);
    }

    public boolean get() {
        return get(0);
    }

    public BoolPointer put(boolean b) {
        return put(0, b);
    }
}
