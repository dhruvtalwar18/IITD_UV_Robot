package org.bytedeco.javacpp;

import java.io.UnsupportedEncodingException;
import org.bytedeco.javacpp.Pointer;

public class PointerPointer<P extends Pointer> extends Pointer {
    private P[] pointerArray;

    private native void allocateArray(long j);

    public native P get(Class<P> cls, long j);

    public native PointerPointer<P> put(long j, Pointer pointer);

    public PointerPointer(String... array) {
        this((long) array.length);
        putString(array);
    }

    public PointerPointer(String[] array, String charsetName) throws UnsupportedEncodingException {
        this((long) array.length);
        putString(array, charsetName);
    }

    public PointerPointer(P... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(byte[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(short[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(int[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(long[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(float[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(double[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(char[]... array) {
        this((long) array.length);
        put(array);
    }

    public PointerPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new PointerPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public PointerPointer() {
    }

    public PointerPointer(Pointer p) {
        super(p);
    }

    public PointerPointer<P> position(long position) {
        return (PointerPointer) super.position(position);
    }

    public PointerPointer<P> limit(long limit) {
        return (PointerPointer) super.limit(limit);
    }

    public PointerPointer<P> capacity(long capacity) {
        return (PointerPointer) super.capacity(capacity);
    }

    public String getString(long i) {
        BytePointer p = (BytePointer) get(BytePointer.class, i);
        if (p != null) {
            return p.getString();
        }
        return null;
    }

    public String getString(long i, String charsetName) throws UnsupportedEncodingException {
        BytePointer p = (BytePointer) get(BytePointer.class, i);
        if (p != null) {
            return p.getString(charsetName);
        }
        return null;
    }

    public PointerPointer<P> putString(String... array) {
        this.pointerArray = (Pointer[]) new BytePointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new BytePointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> putString(String[] array, String charsetName) throws UnsupportedEncodingException {
        this.pointerArray = (Pointer[]) new BytePointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new BytePointer(array[i], charsetName) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(P... array) {
        this.pointerArray = array;
        for (int i = 0; i < array.length; i++) {
            put((long) i, array[i]);
        }
        return this;
    }

    public PointerPointer<P> put(byte[]... array) {
        this.pointerArray = (Pointer[]) new BytePointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new BytePointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(short[]... array) {
        this.pointerArray = (Pointer[]) new ShortPointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new ShortPointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(int[]... array) {
        this.pointerArray = (Pointer[]) new IntPointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new IntPointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(long[]... array) {
        this.pointerArray = (Pointer[]) new LongPointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new LongPointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(float[]... array) {
        this.pointerArray = (Pointer[]) new FloatPointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new FloatPointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(double[]... array) {
        this.pointerArray = (Pointer[]) new DoublePointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new DoublePointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public PointerPointer<P> put(char[]... array) {
        this.pointerArray = (Pointer[]) new CharPointer[array.length];
        for (int i = 0; i < array.length; i++) {
            this.pointerArray[i] = array[i] != null ? new CharPointer(array[i]) : null;
        }
        return put(this.pointerArray);
    }

    public Pointer get() {
        return get(0);
    }

    public P get(Class<P> cls) {
        return get(cls, 0);
    }

    public Pointer get(long i) {
        return get(Pointer.class, i);
    }

    public PointerPointer<P> put(Pointer p) {
        return put(0, p);
    }
}
