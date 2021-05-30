package org.bytedeco.javacpp;

import java.nio.Buffer;
import java.nio.CharBuffer;
import org.xbill.DNS.TTL;

public class CharPointer extends Pointer {
    private native void allocateArray(long j);

    public native char get(long j);

    public native CharPointer get(char[] cArr, int i, int i2);

    public native CharPointer put(long j, char c);

    public native CharPointer put(char[] cArr, int i, int i2);

    public CharPointer(String s) {
        this((long) (s.toCharArray().length + 1));
        putString(s);
    }

    public CharPointer(char... array) {
        this((long) array.length);
        put(array);
    }

    public CharPointer(CharBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            char[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public CharPointer(long size) {
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
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new CharPointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public CharPointer() {
    }

    public CharPointer(Pointer p) {
        super(p);
    }

    public CharPointer position(long position) {
        return (CharPointer) super.position(position);
    }

    public CharPointer limit(long limit) {
        return (CharPointer) super.limit(limit);
    }

    public CharPointer capacity(long capacity) {
        return (CharPointer) super.capacity(capacity);
    }

    public char[] getStringChars() {
        if (this.limit > this.position) {
            char[] array = new char[((int) Math.min(this.limit - this.position, TTL.MAX_VALUE))];
            get(array);
            return array;
        }
        char[] buffer = new char[16];
        int i = 0;
        while (true) {
            char c = get((long) i);
            buffer[i] = c;
            if (c != 0) {
                i++;
                if (i >= buffer.length) {
                    char[] newbuffer = new char[(buffer.length * 2)];
                    System.arraycopy(buffer, 0, newbuffer, 0, buffer.length);
                    buffer = newbuffer;
                }
            } else {
                char[] newbuffer2 = new char[i];
                System.arraycopy(buffer, 0, newbuffer2, 0, i);
                return newbuffer2;
            }
        }
    }

    public String getString() {
        return new String(getStringChars());
    }

    public CharPointer putString(String s) {
        char[] chars = s.toCharArray();
        return put(chars).put((long) chars.length, 0).limit((long) chars.length);
    }

    public char get() {
        return get(0);
    }

    public CharPointer put(char c) {
        return put(0, c);
    }

    public CharPointer get(char[] array) {
        return get(array, 0, array.length);
    }

    public CharPointer put(char... array) {
        return put(array, 0, array.length);
    }

    public final CharBuffer asBuffer() {
        return asByteBuffer().asCharBuffer();
    }
}
