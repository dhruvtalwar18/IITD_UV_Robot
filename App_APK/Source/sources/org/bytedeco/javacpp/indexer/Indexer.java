package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import org.bytedeco.javacpp.Pointer;

public abstract class Indexer implements AutoCloseable {
    protected static final long[] ONE_STRIDE = {1};
    protected Indexable indexable;
    protected long[] sizes;
    protected long[] strides;

    public abstract double getDouble(long... jArr);

    public abstract Indexer putDouble(long[] jArr, double d);

    public abstract void release();

    public void close() throws Exception {
        release();
    }

    protected Indexer(long[] sizes2, long[] strides2) {
        this.sizes = sizes2;
        this.strides = strides2;
    }

    public long[] sizes() {
        return this.sizes;
    }

    public long[] strides() {
        return this.strides;
    }

    public long size(int i) {
        return this.sizes[i];
    }

    public long stride(int i) {
        return this.strides[i];
    }

    public long rows() {
        if (this.sizes.length == 3) {
            return this.sizes[0];
        }
        return -1;
    }

    public long cols() {
        if (this.sizes.length == 3) {
            return this.sizes[1];
        }
        return -1;
    }

    public long width() {
        if (this.sizes.length == 3) {
            return this.sizes[1];
        }
        return -1;
    }

    public long height() {
        if (this.sizes.length == 3) {
            return this.sizes[0];
        }
        return -1;
    }

    public long channels() {
        if (this.sizes.length == 3) {
            return this.sizes[2];
        }
        return 1;
    }

    protected static final long checkIndex(long i, long size) {
        if (i >= 0 && i < size) {
            return i;
        }
        throw new IndexOutOfBoundsException(Long.toString(i));
    }

    public long index(long... indices) {
        long index = 0;
        int i = 0;
        while (i < indices.length && i < this.strides.length) {
            index += indices[i] * this.strides[i];
            i++;
        }
        return index;
    }

    public Indexable indexable() {
        return this.indexable;
    }

    public Indexer indexable(Indexable indexable2) {
        this.indexable = indexable2;
        return this;
    }

    public Object array() {
        return null;
    }

    public Buffer buffer() {
        return null;
    }

    public Pointer pointer() {
        return null;
    }

    public String toString() {
        long rows = this.sizes.length > 0 ? this.sizes[0] : 1;
        long cols = this.sizes.length > 1 ? this.sizes[1] : 1;
        long channels = this.sizes.length > 2 ? this.sizes[2] : 1;
        StringBuilder s = new StringBuilder(rows > 1 ? "\n[ " : "[ ");
        int i = 0;
        while (((long) i) < rows) {
            int j = 0;
            while (((long) j) < cols) {
                if (channels > 1) {
                    s.append("(");
                }
                int k = 0;
                while (((long) k) < channels) {
                    long channels2 = channels;
                    s.append((float) getDouble((long) i, (long) j, (long) k));
                    long rows2 = rows;
                    if (((long) k) < channels2 - 1) {
                        s.append(", ");
                    }
                    k++;
                    channels = channels2;
                    rows = rows2;
                }
                long rows3 = rows;
                long channels3 = channels;
                if (channels3 > 1) {
                    s.append(")");
                }
                if (((long) j) < cols - 1) {
                    s.append(", ");
                }
                j++;
                channels = channels3;
                rows = rows3;
            }
            long rows4 = rows;
            long channels4 = channels;
            if (((long) i) < rows4 - 1) {
                s.append("\n  ");
            }
            i++;
            channels = channels4;
            rows = rows4;
        }
        long j2 = channels;
        s.append(" ]");
        return s.toString();
    }
}
