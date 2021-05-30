package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.IntBuffer;
import org.xbill.DNS.TTL;

public class IntPointer extends Pointer {
    private native void allocateArray(long j);

    public native int get(long j);

    public native IntPointer get(int[] iArr, int i, int i2);

    public native IntPointer put(long j, int i);

    public native IntPointer put(int[] iArr, int i, int i2);

    public IntPointer(String s) {
        this((long) (s.length() + 1));
        putString(s);
    }

    public IntPointer(int... array) {
        this((long) array.length);
        put(array);
    }

    public IntPointer(IntBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            int[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public IntPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new IntPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public IntPointer() {
    }

    public IntPointer(Pointer p) {
        super(p);
    }

    public IntPointer position(long position) {
        return (IntPointer) super.position(position);
    }

    public IntPointer limit(long limit) {
        return (IntPointer) super.limit(limit);
    }

    public IntPointer capacity(long capacity) {
        return (IntPointer) super.capacity(capacity);
    }

    public int[] getStringCodePoints() {
        if (this.limit > this.position) {
            int[] array = new int[((int) Math.min(this.limit - this.position, TTL.MAX_VALUE))];
            get(array);
            return array;
        }
        int[] buffer = new int[16];
        int i = 0;
        while (true) {
            int i2 = get((long) i);
            buffer[i] = i2;
            if (i2 != 0) {
                i++;
                if (i >= buffer.length) {
                    int[] newbuffer = new int[(buffer.length * 2)];
                    System.arraycopy(buffer, 0, newbuffer, 0, buffer.length);
                    buffer = newbuffer;
                }
            } else {
                int[] newbuffer2 = new int[i];
                System.arraycopy(buffer, 0, newbuffer2, 0, i);
                return newbuffer2;
            }
        }
    }

    public String getString() {
        int[] codePoints = getStringCodePoints();
        return new String(codePoints, 0, codePoints.length);
    }

    public IntPointer putString(String s) {
        int[] codePoints = new int[s.length()];
        for (int i = 0; i < codePoints.length; i++) {
            codePoints[i] = s.codePointAt(i);
        }
        return put(codePoints).put((long) codePoints.length, 0).limit((long) codePoints.length);
    }

    public int get() {
        return get(0);
    }

    public IntPointer put(int j) {
        return put(0, j);
    }

    public IntPointer get(int[] array) {
        return get(array, 0, array.length);
    }

    public IntPointer put(int... array) {
        return put(array, 0, array.length);
    }

    public final IntBuffer asBuffer() {
        return asByteBuffer().asIntBuffer();
    }
}
