package org.bytedeco.javacpp.helper;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.annotation.ValueGetter;
import org.bytedeco.javacpp.indexer.ByteIndexer;
import org.bytedeco.javacpp.indexer.DoubleIndexer;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.javacpp.indexer.Indexable;
import org.bytedeco.javacpp.indexer.Indexer;
import org.bytedeco.javacpp.indexer.IntIndexer;
import org.bytedeco.javacpp.indexer.ShortIndexer;
import org.bytedeco.javacpp.indexer.UByteIndexer;
import org.bytedeco.javacpp.indexer.UShortIndexer;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_stitching;
import sensor_msgs.NavSatStatus;

public class opencv_core extends org.bytedeco.javacpp.presets.opencv_core {

    public static abstract class AbstractArray extends Pointer implements Indexable {
        static final /* synthetic */ boolean $assertionsDisabled = false;

        public abstract int arrayChannels();

        public abstract BytePointer arrayData();

        public abstract int arrayDepth();

        public abstract int arrayHeight();

        public abstract int arrayOrigin();

        public abstract void arrayOrigin(int i);

        public abstract opencv_core.IplROI arrayROI();

        public abstract int arraySize();

        public abstract int arrayStep();

        public abstract int arrayWidth();

        static {
            Class<opencv_core> cls = opencv_core.class;
            Loader.load();
        }

        public AbstractArray(Pointer p) {
            super(p);
        }

        public <B extends Buffer> B createBuffer() {
            return createBuffer(0);
        }

        public <B extends Buffer> B createBuffer(int index) {
            BytePointer ptr = arrayData();
            int size = arraySize();
            int arrayDepth = arrayDepth();
            if (arrayDepth != -2147483640) {
                if (arrayDepth != -2147483632) {
                    if (arrayDepth == -2147483616) {
                        return new IntPointer((Pointer) ptr).position((long) index).capacity((long) (size / 4)).asBuffer();
                    }
                    if (arrayDepth != 8) {
                        if (arrayDepth != 16) {
                            if (arrayDepth == 32) {
                                return new FloatPointer((Pointer) ptr).position((long) index).capacity((long) (size / 4)).asBuffer();
                            }
                            if (arrayDepth != 64) {
                                return null;
                            }
                            return new DoublePointer((Pointer) ptr).position((long) index).capacity((long) (size / 8)).asBuffer();
                        }
                    }
                }
                return new ShortPointer((Pointer) ptr).position((long) index).capacity((long) (size / 2)).asBuffer();
            }
            return ptr.position((long) index).capacity((long) size).asBuffer();
        }

        public <I extends Indexer> I createIndexer() {
            return createIndexer(true);
        }

        public <I extends Indexer> I createIndexer(boolean direct) {
            BytePointer ptr = arrayData();
            int size = arraySize();
            long[] sizes = {(long) arrayHeight(), (long) arrayWidth(), (long) arrayChannels()};
            long[] strides = {(long) arrayStep(), (long) arrayChannels(), 1};
            int arrayDepth = arrayDepth();
            if (arrayDepth == -2147483640) {
                return ByteIndexer.create(ptr.capacity((long) size), sizes, strides, direct).indexable(this);
            }
            if (arrayDepth == -2147483632) {
                strides[0] = strides[0] / 2;
                return ShortIndexer.create(new ShortPointer((Pointer) ptr).capacity((long) (size / 2)), sizes, strides, direct).indexable(this);
            } else if (arrayDepth == -2147483616) {
                strides[0] = strides[0] / 4;
                return IntIndexer.create(new IntPointer((Pointer) ptr).capacity((long) (size / 4)), sizes, strides, direct).indexable(this);
            } else if (arrayDepth == 8) {
                return UByteIndexer.create(ptr.capacity((long) size), sizes, strides, direct).indexable(this);
            } else {
                if (arrayDepth == 16) {
                    strides[0] = strides[0] / 2;
                    return UShortIndexer.create(new ShortPointer((Pointer) ptr).capacity((long) (size / 2)), sizes, strides, direct).indexable(this);
                } else if (arrayDepth == 32) {
                    strides[0] = strides[0] / 4;
                    return FloatIndexer.create(new FloatPointer((Pointer) ptr).capacity((long) (size / 4)), sizes, strides, direct).indexable(this);
                } else if (arrayDepth != 64) {
                    return null;
                } else {
                    strides[0] = strides[0] / 8;
                    return DoubleIndexer.create(new DoublePointer((Pointer) ptr).capacity((long) (size / 8)), sizes, strides, direct).indexable(this);
                }
            }
        }

        public double highValue() {
            int arrayDepth = arrayDepth();
            if (arrayDepth == -2147483640) {
                return 127.0d;
            }
            if (arrayDepth == -2147483632) {
                return 32767.0d;
            }
            if (arrayDepth == -2147483616) {
                return 2.147483647E9d;
            }
            if (arrayDepth != 1) {
                if (arrayDepth == 8) {
                    return 255.0d;
                }
                if (arrayDepth == 16) {
                    return 65535.0d;
                }
                if (arrayDepth == 32 || arrayDepth == 64) {
                    return 1.0d;
                }
                return opencv_stitching.Stitcher.ORIG_RESOL;
            }
            return 1.0d;
        }

        public opencv_core.CvSize cvSize() {
            return org.bytedeco.javacpp.opencv_core.cvSize(arrayWidth(), arrayHeight());
        }

        @Deprecated
        public ByteBuffer getByteBuffer(int index) {
            return arrayData().position((long) index).capacity((long) arraySize()).asByteBuffer();
        }

        @Deprecated
        public ShortBuffer getShortBuffer(int index) {
            return getByteBuffer(index * 2).asShortBuffer();
        }

        @Deprecated
        public IntBuffer getIntBuffer(int index) {
            return getByteBuffer(index * 4).asIntBuffer();
        }

        @Deprecated
        public FloatBuffer getFloatBuffer(int index) {
            return getByteBuffer(index * 4).asFloatBuffer();
        }

        @Deprecated
        public DoubleBuffer getDoubleBuffer(int index) {
            return getByteBuffer(index * 8).asDoubleBuffer();
        }

        @Deprecated
        public ByteBuffer getByteBuffer() {
            return getByteBuffer(0);
        }

        @Deprecated
        public ShortBuffer getShortBuffer() {
            return getShortBuffer(0);
        }

        @Deprecated
        public IntBuffer getIntBuffer() {
            return getIntBuffer(0);
        }

        @Deprecated
        public FloatBuffer getFloatBuffer() {
            return getFloatBuffer(0);
        }

        @Deprecated
        public DoubleBuffer getDoubleBuffer() {
            return getDoubleBuffer(0);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            try {
                return getClass().getName() + "[width=" + arrayWidth() + ",height=" + arrayHeight() + ",depth=" + arrayDepth() + ",channels=" + arrayChannels() + "]";
            } catch (Exception e) {
                return super.toString();
            }
        }
    }

    @Opaque
    public static class CvArr extends AbstractArray {
        public CvArr(Pointer p) {
            super(p);
        }

        public int arrayChannels() {
            throw new UnsupportedOperationException();
        }

        public int arrayDepth() {
            throw new UnsupportedOperationException();
        }

        public int arrayOrigin() {
            throw new UnsupportedOperationException();
        }

        public void arrayOrigin(int origin) {
            throw new UnsupportedOperationException();
        }

        public int arrayWidth() {
            throw new UnsupportedOperationException();
        }

        public int arrayHeight() {
            throw new UnsupportedOperationException();
        }

        public opencv_core.IplROI arrayROI() {
            throw new UnsupportedOperationException();
        }

        public int arraySize() {
            throw new UnsupportedOperationException();
        }

        public BytePointer arrayData() {
            throw new UnsupportedOperationException();
        }

        public int arrayStep() {
            throw new UnsupportedOperationException();
        }
    }

    @Name({"CvArr*"})
    public static class CvArrArray extends PointerPointer<CvArr> {
        private native void allocateArray(long j);

        public native CvArr get();

        public native CvArrArray put(CvArr cvArr);

        static {
            Loader.load();
        }

        public CvArrArray(CvArr... array) {
            this((long) array.length);
            put(array);
            position(0);
        }

        public CvArrArray(long size) {
            super(size);
            allocateArray(size);
        }

        public CvArrArray(Pointer p) {
            super(p);
        }

        public CvArrArray position(long position) {
            return (CvArrArray) super.position(position);
        }

        public CvArrArray put(CvArr... array) {
            for (int i = 0; i < array.length; i++) {
                position((long) i).put(array[i]);
            }
            return this;
        }
    }

    @Name({"CvMat*"})
    public static class CvMatArray extends CvArrArray {
        private native void allocateArray(long j);

        @ValueGetter
        public native opencv_core.CvMat get();

        public CvMatArray(opencv_core.CvMat... array) {
            this((long) array.length);
            put((CvArr[]) array);
            position(0);
        }

        public CvMatArray(long size) {
            super(new CvArr[0]);
            allocateArray(size);
        }

        public CvMatArray(Pointer p) {
            super(p);
        }

        public CvMatArray position(long position) {
            return (CvMatArray) super.position(position);
        }

        public CvMatArray put(CvArr... array) {
            return (CvMatArray) super.put(array);
        }

        public CvMatArray put(CvArr p) {
            if (p instanceof opencv_core.CvMat) {
                return (CvMatArray) super.put(p);
            }
            throw new ArrayStoreException(p.getClass().getName());
        }
    }

    @Name({"CvMatND*"})
    public static class CvMatNDArray extends CvArrArray {
        private native void allocateArray(long j);

        @ValueGetter
        public native opencv_core.CvMatND get();

        public CvMatNDArray(opencv_core.CvMatND... array) {
            this((long) array.length);
            put((CvArr[]) array);
            position(0);
        }

        public CvMatNDArray(long size) {
            super(new CvArr[0]);
            allocateArray(size);
        }

        public CvMatNDArray(Pointer p) {
            super(p);
        }

        public CvMatNDArray position(long position) {
            return (CvMatNDArray) super.position(position);
        }

        public CvMatNDArray put(CvArr... array) {
            return (CvMatNDArray) super.put(array);
        }

        public CvMatNDArray put(CvArr p) {
            if (p instanceof opencv_core.CvMatND) {
                return (CvMatNDArray) super.put(p);
            }
            throw new ArrayStoreException(p.getClass().getName());
        }
    }

    @Name({"IplImage*"})
    public static class IplImageArray extends CvArrArray {
        private native void allocateArray(long j);

        @ValueGetter
        public native opencv_core.IplImage get();

        public IplImageArray(opencv_core.IplImage... array) {
            this((long) array.length);
            put((CvArr[]) array);
            position(0);
        }

        public IplImageArray(long size) {
            super(new CvArr[0]);
            allocateArray(size);
        }

        public IplImageArray(Pointer p) {
            super(p);
        }

        public IplImageArray position(long position) {
            return (IplImageArray) super.position(position);
        }

        public IplImageArray put(CvArr... array) {
            return (IplImageArray) super.put(array);
        }

        public IplImageArray put(CvArr p) {
            if (p instanceof opencv_core.IplImage) {
                return (IplImageArray) super.put(p);
            }
            throw new ArrayStoreException(p.getClass().getName());
        }
    }

    public static abstract class AbstractIplImage extends CvArr {
        protected BytePointer pointer;

        public abstract int depth();

        public abstract int height();

        public abstract BytePointer imageData();

        public abstract int imageSize();

        public abstract int nChannels();

        public abstract int origin();

        public abstract opencv_core.IplImage origin(int i);

        public abstract opencv_core.IplROI roi();

        public abstract int width();

        public abstract int widthStep();

        public AbstractIplImage(Pointer p) {
            super(p);
        }

        public static opencv_core.IplImage create(opencv_core.CvSize size, int depth, int channels) {
            opencv_core.IplImage i = org.bytedeco.javacpp.opencv_core.cvCreateImage(size, depth, channels);
            if (i != null) {
                i.deallocator(new ReleaseDeallocator(i));
            }
            return i;
        }

        public static opencv_core.IplImage create(int width, int height, int depth, int channels) {
            return create(org.bytedeco.javacpp.opencv_core.cvSize(width, height), depth, channels);
        }

        public static opencv_core.IplImage create(opencv_core.CvSize size, int depth, int channels, int origin) {
            opencv_core.IplImage i = create(size, depth, channels);
            if (i != null) {
                i.origin(origin);
            }
            return i;
        }

        public static opencv_core.IplImage create(int width, int height, int depth, int channels, int origin) {
            opencv_core.IplImage i = create(width, height, depth, channels);
            if (i != null) {
                i.origin(origin);
            }
            return i;
        }

        public static opencv_core.IplImage createHeader(opencv_core.CvSize size, int depth, int channels) {
            opencv_core.IplImage i = org.bytedeco.javacpp.opencv_core.cvCreateImageHeader(size, depth, channels);
            if (i != null) {
                i.deallocator(new HeaderReleaseDeallocator(i));
            }
            return i;
        }

        public static opencv_core.IplImage createHeader(int width, int height, int depth, int channels) {
            return createHeader(org.bytedeco.javacpp.opencv_core.cvSize(width, height), depth, channels);
        }

        public static opencv_core.IplImage createHeader(opencv_core.CvSize size, int depth, int channels, int origin) {
            opencv_core.IplImage i = createHeader(size, depth, channels);
            if (i != null) {
                i.origin(origin);
            }
            return i;
        }

        public static opencv_core.IplImage createHeader(int width, int height, int depth, int channels, int origin) {
            opencv_core.IplImage i = createHeader(width, height, depth, channels);
            if (i != null) {
                i.origin(origin);
            }
            return i;
        }

        public static opencv_core.IplImage create(int width, int height, int depth, int channels, Pointer data) {
            opencv_core.IplImage i = createHeader(width, height, depth, channels);
            BytePointer bytePointer = new BytePointer(data);
            i.pointer = bytePointer;
            i.imageData(bytePointer);
            return i;
        }

        public static opencv_core.IplImage createCompatible(opencv_core.IplImage template) {
            return createIfNotCompatible((opencv_core.IplImage) null, template);
        }

        public static opencv_core.IplImage createIfNotCompatible(opencv_core.IplImage image, opencv_core.IplImage template) {
            if (!(image != null && image.width() == template.width() && image.height() == template.height() && image.depth() == template.depth() && image.nChannels() == template.nChannels())) {
                image = create(template.width(), template.height(), template.depth(), template.nChannels(), template.origin());
            }
            image.origin(template.origin());
            return image;
        }

        public opencv_core.IplImage clone() {
            opencv_core.IplImage i = org.bytedeco.javacpp.opencv_core.cvCloneImage((opencv_core.IplImage) this);
            if (i != null) {
                i.deallocator(new ReleaseDeallocator(i));
            }
            return i;
        }

        public void release() {
            deallocate();
        }

        protected static class ReleaseDeallocator extends opencv_core.IplImage implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.IplImage p) {
                super((Pointer) p);
            }

            public void deallocate() {
                if (!isNull()) {
                    org.bytedeco.javacpp.opencv_core.cvReleaseImage((opencv_core.IplImage) this);
                    setNull();
                }
            }
        }

        protected static class HeaderReleaseDeallocator extends opencv_core.IplImage implements Pointer.Deallocator {
            HeaderReleaseDeallocator(opencv_core.IplImage p) {
                super((Pointer) p);
            }

            public void deallocate() {
                if (!isNull()) {
                    org.bytedeco.javacpp.opencv_core.cvReleaseImageHeader((opencv_core.IplImage) this);
                    setNull();
                }
            }
        }

        public int arrayChannels() {
            return nChannels();
        }

        public int arrayDepth() {
            return depth();
        }

        public int arrayOrigin() {
            return origin();
        }

        public void arrayOrigin(int origin) {
            origin(origin);
        }

        public int arrayWidth() {
            return width();
        }

        public int arrayHeight() {
            return height();
        }

        public opencv_core.IplROI arrayROI() {
            return roi();
        }

        public int arraySize() {
            return imageSize();
        }

        public BytePointer arrayData() {
            return imageData();
        }

        public int arrayStep() {
            return widthStep();
        }

        public opencv_core.CvMat asCvMat() {
            opencv_core.CvMat mat = new opencv_core.CvMat();
            org.bytedeco.javacpp.opencv_core.cvGetMat((CvArr) this, mat, (IntPointer) null, 0);
            return mat;
        }
    }

    public static abstract class AbstractCvMat extends CvArr {
        static final /* synthetic */ boolean $assertionsDisabled = false;
        private ByteBuffer byteBuffer = null;
        private DoubleBuffer doubleBuffer = null;
        private FloatBuffer floatBuffer = null;
        private int fullSize = 0;
        private IntBuffer intBuffer = null;
        protected BytePointer pointer;
        private ShortBuffer shortBuffer = null;

        public abstract int cols();

        public abstract DoublePointer data_db();

        public abstract FloatPointer data_fl();

        public abstract IntPointer data_i();

        public abstract BytePointer data_ptr();

        public abstract ShortPointer data_s();

        public abstract int rows();

        public abstract int step();

        public abstract int type();

        public abstract opencv_core.CvMat type(int i);

        static {
            Class<opencv_core> cls = opencv_core.class;
        }

        public AbstractCvMat(Pointer p) {
            super(p);
        }

        public static opencv_core.CvMat create(int rows, int cols, int type) {
            opencv_core.CvMat m = org.bytedeco.javacpp.opencv_core.cvCreateMat(rows, cols, type);
            if (m != null) {
                m.fullSize = m.size();
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public static opencv_core.CvMat create(int rows, int cols, int depth, int channels) {
            return create(rows, cols, org.bytedeco.javacpp.opencv_core.CV_MAKETYPE(depth, channels));
        }

        public static opencv_core.CvMat create(int rows, int cols) {
            return create(rows, cols, 6, 1);
        }

        public static opencv_core.CvMat createHeader(int rows, int cols, int type) {
            opencv_core.CvMat m = org.bytedeco.javacpp.opencv_core.cvCreateMatHeader(rows, cols, type);
            if (m != null) {
                m.fullSize = m.size();
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public static opencv_core.CvMat createHeader(int rows, int cols, int depth, int channels) {
            return createHeader(rows, cols, org.bytedeco.javacpp.opencv_core.CV_MAKETYPE(depth, channels));
        }

        public static opencv_core.CvMat createHeader(int rows, int cols) {
            return createHeader(rows, cols, 6, 1);
        }

        public static opencv_core.CvMat create(int rows, int cols, int depth, int channels, Pointer data) {
            opencv_core.CvMat m = createHeader(rows, cols, depth, channels);
            BytePointer bytePointer = new BytePointer(data);
            m.pointer = bytePointer;
            m.data_ptr(bytePointer);
            return m;
        }

        public static ThreadLocal<opencv_core.CvMat> createThreadLocal(final int rows, final int cols, final int type) {
            return new ThreadLocal<opencv_core.CvMat>() {
                /* access modifiers changed from: protected */
                public opencv_core.CvMat initialValue() {
                    return AbstractCvMat.create(rows, cols, type);
                }
            };
        }

        public static ThreadLocal<opencv_core.CvMat> createThreadLocal(int rows, int cols, int depth, int channels) {
            return createThreadLocal(rows, cols, org.bytedeco.javacpp.opencv_core.CV_MAKETYPE(depth, channels));
        }

        public static ThreadLocal<opencv_core.CvMat> createThreadLocal(int rows, int cols) {
            return createThreadLocal(rows, cols, 6, 1);
        }

        public static ThreadLocal<opencv_core.CvMat> createHeaderThreadLocal(final int rows, final int cols, final int type) {
            return new ThreadLocal<opencv_core.CvMat>() {
                /* access modifiers changed from: protected */
                public opencv_core.CvMat initialValue() {
                    return AbstractCvMat.createHeader(rows, cols, type);
                }
            };
        }

        public static ThreadLocal<opencv_core.CvMat> createHeaderThreadLocal(int rows, int cols, int depth, int channels) {
            return createHeaderThreadLocal(rows, cols, org.bytedeco.javacpp.opencv_core.CV_MAKETYPE(depth, channels));
        }

        public static ThreadLocal<opencv_core.CvMat> createHeaderThreadLocal(int rows, int cols) {
            return createHeaderThreadLocal(rows, cols, 6, 1);
        }

        public opencv_core.CvMat clone() {
            opencv_core.CvMat m = org.bytedeco.javacpp.opencv_core.cvCloneMat((opencv_core.CvMat) this);
            if (m != null) {
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public void release() {
            deallocate();
        }

        protected static class ReleaseDeallocator extends opencv_core.CvMat implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.CvMat m) {
                super((Pointer) m);
            }

            public void deallocate() {
                if (!isNull()) {
                    org.bytedeco.javacpp.opencv_core.cvReleaseMat((opencv_core.CvMat) this);
                    setNull();
                }
            }
        }

        public int matType() {
            return org.bytedeco.javacpp.opencv_core.CV_MAT_TYPE(type());
        }

        public void type(int depth, int cn) {
            type(org.bytedeco.javacpp.opencv_core.CV_MAKETYPE(depth, cn) | org.bytedeco.javacpp.opencv_core.CV_MAT_MAGIC_VAL);
        }

        public int depth() {
            return org.bytedeco.javacpp.opencv_core.CV_MAT_DEPTH(type());
        }

        public int channels() {
            return org.bytedeco.javacpp.opencv_core.CV_MAT_CN(type());
        }

        public int nChannels() {
            return org.bytedeco.javacpp.opencv_core.CV_MAT_CN(type());
        }

        public boolean isContinuous() {
            return org.bytedeco.javacpp.opencv_core.CV_IS_MAT_CONT(type()) != 0;
        }

        public int elemSize() {
            switch (depth()) {
                case 0:
                case 1:
                    return 1;
                case 2:
                case 3:
                    return 2;
                case 4:
                case 5:
                    return 4;
                case 6:
                    return 8;
                default:
                    return 0;
            }
        }

        public int length() {
            return rows() * cols();
        }

        public int total() {
            return rows() * cols();
        }

        public boolean empty() {
            return length() == 0;
        }

        public int size() {
            int rows = rows();
            return (cols() * elemSize() * channels()) + (rows > 1 ? step() * (rows - 1) : 0);
        }

        public int arrayChannels() {
            return channels();
        }

        public int arrayDepth() {
            switch (depth()) {
                case 0:
                    return 8;
                case 1:
                    return org.bytedeco.javacpp.opencv_core.IPL_DEPTH_8S;
                case 2:
                    return 16;
                case 3:
                    return org.bytedeco.javacpp.opencv_core.IPL_DEPTH_16S;
                case 4:
                    return org.bytedeco.javacpp.opencv_core.IPL_DEPTH_32S;
                case 5:
                    return 32;
                case 6:
                    return 64;
                default:
                    return -1;
            }
        }

        public int arrayOrigin() {
            return 0;
        }

        public void arrayOrigin(int origin) {
        }

        public int arrayWidth() {
            return cols();
        }

        public int arrayHeight() {
            return rows();
        }

        public opencv_core.IplROI arrayROI() {
            return null;
        }

        public int arraySize() {
            return size();
        }

        public BytePointer arrayData() {
            return data_ptr();
        }

        public int arrayStep() {
            return step();
        }

        @Deprecated
        public void reset() {
            this.fullSize = 0;
            this.byteBuffer = null;
            this.shortBuffer = null;
            this.intBuffer = null;
            this.floatBuffer = null;
            this.doubleBuffer = null;
        }

        private int fullSize() {
            if (this.fullSize > 0) {
                return this.fullSize;
            }
            int size = size();
            this.fullSize = size;
            return size;
        }

        @Deprecated
        public ByteBuffer getByteBuffer() {
            if (this.byteBuffer == null) {
                this.byteBuffer = data_ptr().capacity((long) fullSize()).asBuffer();
            }
            this.byteBuffer.position(0);
            return this.byteBuffer;
        }

        @Deprecated
        public ShortBuffer getShortBuffer() {
            if (this.shortBuffer == null) {
                this.shortBuffer = data_s().capacity((long) (fullSize() / 2)).asBuffer();
            }
            this.shortBuffer.position(0);
            return this.shortBuffer;
        }

        @Deprecated
        public IntBuffer getIntBuffer() {
            if (this.intBuffer == null) {
                this.intBuffer = data_i().capacity((long) (fullSize() / 4)).asBuffer();
            }
            this.intBuffer.position(0);
            return this.intBuffer;
        }

        @Deprecated
        public FloatBuffer getFloatBuffer() {
            if (this.floatBuffer == null) {
                this.floatBuffer = data_fl().capacity((long) (fullSize() / 4)).asBuffer();
            }
            this.floatBuffer.position(0);
            return this.floatBuffer;
        }

        @Deprecated
        public DoubleBuffer getDoubleBuffer() {
            if (this.doubleBuffer == null) {
                this.doubleBuffer = data_db().capacity((long) (fullSize() / 8)).asBuffer();
            }
            this.doubleBuffer.position(0);
            return this.doubleBuffer;
        }

        @Deprecated
        public double get(int i) {
            switch (depth()) {
                case 0:
                    return (double) (getByteBuffer().get(i) & NavSatStatus.STATUS_NO_FIX);
                case 1:
                    return (double) getByteBuffer().get(i);
                case 2:
                    return (double) (getShortBuffer().get(i) & 65535);
                case 3:
                    return (double) getShortBuffer().get(i);
                case 4:
                    return (double) getIntBuffer().get(i);
                case 5:
                    return (double) getFloatBuffer().get(i);
                case 6:
                    return getDoubleBuffer().get(i);
                default:
                    return Double.NaN;
            }
        }

        @Deprecated
        public double get(int i, int j) {
            return get(((step() * i) / elemSize()) + (channels() * j));
        }

        @Deprecated
        public double get(int i, int j, int k) {
            return get(((step() * i) / elemSize()) + (channels() * j) + k);
        }

        @Deprecated
        public synchronized opencv_core.CvMat get(int index, double[] vv, int offset, int length) {
            int d = depth();
            int i = 0;
            switch (d) {
                case 0:
                case 1:
                    ByteBuffer bb = getByteBuffer();
                    bb.position(index);
                    while (i < length) {
                        if (d == 0) {
                            vv[i + offset] = (double) (bb.get(i) & NavSatStatus.STATUS_NO_FIX);
                        } else {
                            vv[i + offset] = (double) bb.get(i);
                        }
                        i++;
                    }
                    break;
                case 2:
                case 3:
                    ShortBuffer sb = getShortBuffer();
                    sb.position(index);
                    while (i < length) {
                        if (d == 2) {
                            vv[i + offset] = (double) (sb.get() & 65535);
                        } else {
                            vv[i + offset] = (double) sb.get();
                        }
                        i++;
                    }
                    break;
                case 4:
                    IntBuffer ib = getIntBuffer();
                    ib.position(index);
                    while (i < length) {
                        vv[i + offset] = (double) ib.get();
                        i++;
                    }
                    break;
                case 5:
                    FloatBuffer fb = getFloatBuffer();
                    fb.position(index);
                    while (i < length) {
                        vv[i + offset] = (double) fb.get();
                        i++;
                    }
                    break;
                case 6:
                    getDoubleBuffer().position(index);
                    getDoubleBuffer().get(vv, offset, length);
                    break;
            }
            return (opencv_core.CvMat) this;
        }

        @Deprecated
        public opencv_core.CvMat get(int index, double[] vv) {
            return get(index, vv, 0, vv.length);
        }

        @Deprecated
        public opencv_core.CvMat get(double[] vv) {
            return get(0, vv);
        }

        @Deprecated
        public double[] get() {
            double[] vv = new double[(fullSize() / elemSize())];
            get(vv);
            return vv;
        }

        @Deprecated
        public opencv_core.CvMat put(int i, double v) {
            switch (depth()) {
                case 0:
                case 1:
                    getByteBuffer().put(i, (byte) ((int) v));
                    break;
                case 2:
                case 3:
                    getShortBuffer().put(i, (short) ((int) v));
                    break;
                case 4:
                    getIntBuffer().put(i, (int) v);
                    break;
                case 5:
                    getFloatBuffer().put(i, (float) v);
                    break;
                case 6:
                    getDoubleBuffer().put(i, v);
                    break;
            }
            return (opencv_core.CvMat) this;
        }

        @Deprecated
        public opencv_core.CvMat put(int i, int j, double v) {
            return put(((step() * i) / elemSize()) + (channels() * j), v);
        }

        @Deprecated
        public opencv_core.CvMat put(int i, int j, int k, double v) {
            return put(((step() * i) / elemSize()) + (channels() * j) + k, v);
        }

        @Deprecated
        public synchronized opencv_core.CvMat put(int index, double[] vv, int offset, int length) {
            int i = 0;
            switch (depth()) {
                case 0:
                case 1:
                    ByteBuffer bb = getByteBuffer();
                    bb.position(index);
                    while (i < length) {
                        bb.put((byte) ((int) vv[i + offset]));
                        i++;
                    }
                    break;
                case 2:
                case 3:
                    ShortBuffer sb = getShortBuffer();
                    sb.position(index);
                    while (i < length) {
                        sb.put((short) ((int) vv[i + offset]));
                        i++;
                    }
                    break;
                case 4:
                    IntBuffer ib = getIntBuffer();
                    ib.position(index);
                    while (i < length) {
                        ib.put((int) vv[i + offset]);
                        i++;
                    }
                    break;
                case 5:
                    FloatBuffer fb = getFloatBuffer();
                    fb.position(index);
                    while (i < length) {
                        fb.put((float) vv[i + offset]);
                        i++;
                    }
                    break;
                case 6:
                    DoubleBuffer db = getDoubleBuffer();
                    db.position(index);
                    db.put(vv, offset, length);
                    break;
            }
            return (opencv_core.CvMat) this;
        }

        @Deprecated
        public opencv_core.CvMat put(int index, double... vv) {
            return put(index, vv, 0, vv.length);
        }

        @Deprecated
        public opencv_core.CvMat put(double... vv) {
            return put(0, vv);
        }

        public opencv_core.CvMat put(opencv_core.CvMat mat) {
            return put(0, 0, 0, mat, 0, 0, 0);
        }

        public synchronized opencv_core.CvMat put(int dsti, int dstj, int dstk, opencv_core.CvMat mat, int srci, int srcj, int srck) {
            if (rows() != mat.rows() || cols() != mat.cols() || step() != mat.step() || type() != mat.type() || dsti != 0 || dstj != 0 || dstk != 0 || srci != 0 || srcj != 0 || srck != 0) {
                int w = Math.min(rows() - dsti, mat.rows() - srci);
                int h = Math.min(cols() - dstj, mat.cols() - srcj);
                int d = Math.min(channels() - dstk, mat.channels() - srck);
                int i = 0;
                while (true) {
                    int i2 = i;
                    if (i2 >= w) {
                        break;
                    }
                    int j = 0;
                    while (true) {
                        int j2 = j;
                        if (j2 >= h) {
                            break;
                        }
                        int k = 0;
                        while (true) {
                            int k2 = k;
                            if (k2 >= d) {
                                break;
                            }
                            double d2 = mat.get(i2 + srci, j2 + srcj, k2 + srck);
                            int k3 = k2;
                            put(i2 + dsti, j2 + dstj, k2 + dstk, d2);
                            k = k3 + 1;
                            i2 = i2;
                            j2 = j2;
                        }
                        opencv_core.CvMat cvMat = mat;
                        int i3 = i2;
                        j = j2 + 1;
                    }
                    opencv_core.CvMat cvMat2 = mat;
                    i = i2 + 1;
                }
            } else {
                getByteBuffer().clear();
                mat.getByteBuffer().clear();
                getByteBuffer().put(mat.getByteBuffer());
            }
            opencv_core.CvMat cvMat3 = mat;
            return (opencv_core.CvMat) this;
        }

        public opencv_core.IplImage asIplImage() {
            opencv_core.IplImage image = new opencv_core.IplImage();
            org.bytedeco.javacpp.opencv_core.cvGetImage(this, image);
            return image;
        }

        public String toString() {
            return toString(0);
        }

        public String toString(int indent) {
            StringBuilder s = new StringBuilder("[ ");
            int channels = channels();
            for (int i = 0; i < rows(); i++) {
                for (int j = 0; j < cols(); j++) {
                    opencv_core.CvScalar v = org.bytedeco.javacpp.opencv_core.cvGet2D(this, i, j);
                    if (channels > 1) {
                        s.append("(");
                    }
                    for (int k = 0; k < channels; k++) {
                        s.append((float) v.val(k));
                        if (k < channels - 1) {
                            s.append(", ");
                        }
                    }
                    if (channels > 1) {
                        s.append(")");
                    }
                    if (j < cols() - 1) {
                        s.append(", ");
                    }
                }
                if (i < rows() - 1) {
                    s.append("\n  ");
                    for (int j2 = 0; j2 < indent; j2++) {
                        s.append(' ');
                    }
                }
            }
            s.append(" ]");
            return s.toString();
        }
    }

    public static abstract class AbstractCvMatND extends CvArr {
        public AbstractCvMatND(Pointer p) {
            super(p);
        }

        public static opencv_core.CvMatND create(int dims, int[] sizes, int type) {
            opencv_core.CvMatND m = org.bytedeco.javacpp.opencv_core.cvCreateMatND(dims, sizes, type);
            if (m != null) {
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public opencv_core.CvMatND clone() {
            opencv_core.CvMatND m = org.bytedeco.javacpp.opencv_core.cvCloneMatND((opencv_core.CvMatND) this);
            if (m != null) {
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public void release() {
            deallocate();
        }

        protected static class ReleaseDeallocator extends opencv_core.CvMatND implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.CvMatND p) {
                super((Pointer) p);
            }

            public void deallocate() {
                if (!isNull()) {
                    org.bytedeco.javacpp.opencv_core.cvReleaseMatND((opencv_core.CvMatND) this);
                    setNull();
                }
            }
        }
    }

    public static abstract class AbstractCvSparseMat extends CvArr {
        public AbstractCvSparseMat(Pointer p) {
            super(p);
        }

        public static opencv_core.CvSparseMat create(int dims, int[] sizes, int type) {
            opencv_core.CvSparseMat m = org.bytedeco.javacpp.opencv_core.cvCreateSparseMat(dims, sizes, type);
            if (m != null) {
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public opencv_core.CvSparseMat clone() {
            opencv_core.CvSparseMat m = org.bytedeco.javacpp.opencv_core.cvCloneSparseMat((opencv_core.CvSparseMat) this);
            if (m != null) {
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public void release() {
            deallocate();
        }

        protected static class ReleaseDeallocator extends opencv_core.CvSparseMat implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.CvSparseMat p) {
                super((Pointer) p);
            }

            public void deallocate() {
                if (!isNull()) {
                    org.bytedeco.javacpp.opencv_core.cvReleaseSparseMat((opencv_core.CvSparseMat) this);
                    setNull();
                }
            }
        }
    }

    public static abstract class AbstractCvRect extends IntPointer {
        public abstract int height();

        public abstract int width();

        public abstract int x();

        public abstract int y();

        static {
            Loader.load();
        }

        public AbstractCvRect(Pointer p) {
            super(p);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + x() + ", " + y() + "; " + width() + ", " + height() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(x());
                sb.append(", ");
                sb.append(y());
                sb.append("; ");
                sb.append(width());
                sb.append(", ");
                sb.append(height());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvPoint extends IntPointer {
        public static final opencv_core.CvPoint ZERO = new opencv_core.CvPoint().x(0).y(0);

        public abstract int x();

        public abstract opencv_core.CvPoint x(int i);

        public abstract int y();

        public abstract opencv_core.CvPoint y(int i);

        static {
            Loader.load();
        }

        public AbstractCvPoint(Pointer p) {
            super(p);
        }

        public opencv_core.CvPoint get(int[] pts) {
            return get(pts, 0, pts.length);
        }

        public opencv_core.CvPoint get(int[] pts, int offset, int length) {
            for (int i = 0; i < length / 2; i++) {
                position((long) i);
                pts[(i * 2) + offset] = x();
                pts[(i * 2) + offset + 1] = y();
            }
            return (opencv_core.CvPoint) position(0);
        }

        public final opencv_core.CvPoint put(int[] pts, int offset, int length) {
            for (int i = 0; i < length / 2; i++) {
                position((long) i);
                put(pts[(i * 2) + offset], pts[(i * 2) + offset + 1]);
            }
            return (opencv_core.CvPoint) position(0);
        }

        public final opencv_core.CvPoint put(int... pts) {
            return put(pts, 0, pts.length);
        }

        public final opencv_core.CvPoint put(byte shift, double[] pts, int offset, int length) {
            int[] a = new int[length];
            for (int i = 0; i < length; i++) {
                double d = pts[offset + i];
                double d2 = (double) (1 << shift);
                Double.isNaN(d2);
                a[i] = (int) Math.round(d * d2);
            }
            return put(a, 0, length);
        }

        public final opencv_core.CvPoint put(byte shift, double... pts) {
            return put(shift, pts, 0, pts.length);
        }

        public opencv_core.CvPoint put(int x, int y) {
            return x(x).y(y);
        }

        public opencv_core.CvPoint put(opencv_core.CvPoint o) {
            return x(o.x()).y(o.y());
        }

        public opencv_core.CvPoint put(byte shift, opencv_core.CvPoint2D32f o) {
            x(Math.round(o.x() * ((float) (1 << shift))));
            y(Math.round(o.y() * ((float) (1 << shift))));
            return (opencv_core.CvPoint) this;
        }

        public opencv_core.CvPoint put(byte shift, opencv_core.CvPoint2D64f o) {
            double x = o.x();
            double d = (double) (1 << shift);
            Double.isNaN(d);
            x((int) Math.round(x * d));
            double y = o.y();
            double d2 = (double) (1 << shift);
            Double.isNaN(d2);
            y((int) Math.round(y * d2));
            return (opencv_core.CvPoint) this;
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + x() + ", " + y() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(x());
                sb.append(", ");
                sb.append(y());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvPoint2D32f extends FloatPointer {
        public abstract float x();

        public abstract opencv_core.CvPoint2D32f x(float f);

        public abstract float y();

        public abstract opencv_core.CvPoint2D32f y(float f);

        static {
            Loader.load();
        }

        public AbstractCvPoint2D32f(Pointer p) {
            super(p);
        }

        public opencv_core.CvPoint2D32f get(double[] pts) {
            return get(pts, 0, pts.length);
        }

        public opencv_core.CvPoint2D32f get(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 2; i++) {
                position((long) i);
                pts[(i * 2) + offset] = (double) x();
                pts[(i * 2) + offset + 1] = (double) y();
            }
            return (opencv_core.CvPoint2D32f) position(0);
        }

        public final opencv_core.CvPoint2D32f put(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 2; i++) {
                position((long) i);
                put(pts[(i * 2) + offset], pts[(i * 2) + offset + 1]);
            }
            return (opencv_core.CvPoint2D32f) position(0);
        }

        public final opencv_core.CvPoint2D32f put(double... pts) {
            return put(pts, 0, pts.length);
        }

        public opencv_core.CvPoint2D32f put(double x, double y) {
            return x((float) x).y((float) y);
        }

        public opencv_core.CvPoint2D32f put(opencv_core.CvPoint o) {
            return x((float) o.x()).y((float) o.y());
        }

        public opencv_core.CvPoint2D32f put(opencv_core.CvPoint2D32f o) {
            return x(o.x()).y(o.y());
        }

        public opencv_core.CvPoint2D32f put(opencv_core.CvPoint2D64f o) {
            return x((float) o.x()).y((float) o.y());
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + x() + ", " + y() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(x());
                sb.append(", ");
                sb.append(y());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvPoint3D32f extends FloatPointer {
        public abstract float x();

        public abstract opencv_core.CvPoint3D32f x(float f);

        public abstract float y();

        public abstract opencv_core.CvPoint3D32f y(float f);

        public abstract float z();

        public abstract opencv_core.CvPoint3D32f z(float f);

        static {
            Loader.load();
        }

        public AbstractCvPoint3D32f(Pointer p) {
            super(p);
        }

        public opencv_core.CvPoint3D32f get(double[] pts) {
            return get(pts, 0, pts.length);
        }

        public opencv_core.CvPoint3D32f get(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 3; i++) {
                position((long) i);
                pts[(i * 3) + offset] = (double) x();
                pts[(i * 3) + offset + 1] = (double) y();
                pts[(i * 3) + offset + 2] = (double) z();
            }
            return (opencv_core.CvPoint3D32f) position(0);
        }

        public final opencv_core.CvPoint3D32f put(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 3; i++) {
                position((long) i);
                put(pts[(i * 3) + offset], pts[(i * 3) + offset + 1], pts[(i * 3) + offset + 2]);
            }
            return (opencv_core.CvPoint3D32f) position(0);
        }

        public final opencv_core.CvPoint3D32f put(double... pts) {
            return put(pts, 0, pts.length);
        }

        public opencv_core.CvPoint3D32f put(double x, double y, double z) {
            return x((float) x).y((float) y).z((float) z);
        }

        public opencv_core.CvPoint3D32f put(opencv_core.CvPoint o) {
            return x((float) o.x()).y((float) o.y()).z(0.0f);
        }

        public opencv_core.CvPoint3D32f put(opencv_core.CvPoint2D32f o) {
            return x(o.x()).y(o.y()).z(0.0f);
        }

        public opencv_core.CvPoint3D32f put(opencv_core.CvPoint2D64f o) {
            return x((float) o.x()).y((float) o.y()).z(0.0f);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + x() + ", " + y() + ", " + z() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(x());
                sb.append(", ");
                sb.append(y());
                sb.append(", ");
                sb.append(z());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvPoint2D64f extends DoublePointer {
        public abstract double x();

        public abstract opencv_core.CvPoint2D64f x(double d);

        public abstract double y();

        public abstract opencv_core.CvPoint2D64f y(double d);

        static {
            Loader.load();
        }

        public AbstractCvPoint2D64f(Pointer p) {
            super(p);
        }

        public opencv_core.CvPoint2D64f get(double[] pts) {
            return get(pts, 0, pts.length);
        }

        public opencv_core.CvPoint2D64f get(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 2; i++) {
                position((long) i);
                pts[(i * 2) + offset] = x();
                pts[(i * 2) + offset + 1] = y();
            }
            return (opencv_core.CvPoint2D64f) position(0);
        }

        public final opencv_core.CvPoint2D64f put(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 2; i++) {
                position((long) i);
                put(pts[(i * 2) + offset], pts[(i * 2) + offset + 1]);
            }
            return (opencv_core.CvPoint2D64f) position(0);
        }

        public final opencv_core.CvPoint2D64f put(double... pts) {
            return put(pts, 0, pts.length);
        }

        public opencv_core.CvPoint2D64f put(double x, double y) {
            return x(x).y(y);
        }

        public opencv_core.CvPoint2D64f put(opencv_core.CvPoint o) {
            return x((double) o.x()).y((double) o.y());
        }

        public opencv_core.CvPoint2D64f put(opencv_core.CvPoint2D32f o) {
            return x((double) o.x()).y((double) o.y());
        }

        public opencv_core.CvPoint2D64f put(opencv_core.CvPoint2D64f o) {
            return x(o.x()).y(o.y());
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + ((float) x()) + ", " + ((float) y()) + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append((float) x());
                sb.append(", ");
                sb.append((float) y());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvPoint3D64f extends DoublePointer {
        public abstract double x();

        public abstract opencv_core.CvPoint3D64f x(double d);

        public abstract double y();

        public abstract opencv_core.CvPoint3D64f y(double d);

        public abstract double z();

        public abstract opencv_core.CvPoint3D64f z(double d);

        static {
            Loader.load();
        }

        public AbstractCvPoint3D64f(Pointer p) {
            super(p);
        }

        public opencv_core.CvPoint3D64f get(double[] pts) {
            return get(pts, 0, pts.length);
        }

        public opencv_core.CvPoint3D64f get(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 3; i++) {
                position((long) i);
                pts[(i * 3) + offset] = x();
                pts[(i * 3) + offset + 1] = y();
                pts[(i * 3) + offset + 2] = z();
            }
            return (opencv_core.CvPoint3D64f) position(0);
        }

        public final opencv_core.CvPoint3D64f put(double[] pts, int offset, int length) {
            for (int i = 0; i < length / 3; i++) {
                position((long) i);
                put(pts[(i * 3) + offset], pts[(i * 3) + offset + 1], pts[(i * 3) + offset + 2]);
            }
            return (opencv_core.CvPoint3D64f) position(0);
        }

        public final opencv_core.CvPoint3D64f put(double... pts) {
            return put(pts, 0, pts.length);
        }

        public opencv_core.CvPoint3D64f put(double x, double y, double z) {
            return x(x()).y(y()).z(z());
        }

        public opencv_core.CvPoint3D64f put(opencv_core.CvPoint o) {
            return x((double) o.x()).y((double) o.y()).z(opencv_stitching.Stitcher.ORIG_RESOL);
        }

        public opencv_core.CvPoint3D64f put(opencv_core.CvPoint2D32f o) {
            return x((double) o.x()).y((double) o.y()).z(opencv_stitching.Stitcher.ORIG_RESOL);
        }

        public opencv_core.CvPoint3D64f put(opencv_core.CvPoint2D64f o) {
            return x(o.x()).y(o.y()).z(opencv_stitching.Stitcher.ORIG_RESOL);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + ((float) x()) + ", " + ((float) y()) + ", " + ((float) z()) + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append((float) x());
                sb.append(", ");
                sb.append((float) y());
                sb.append(", ");
                sb.append((float) z());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvSize extends IntPointer {
        public static final opencv_core.CvSize ZERO = new opencv_core.CvSize().width(0).height(0);

        public abstract int height();

        public abstract opencv_core.CvSize height(int i);

        public abstract int width();

        public abstract opencv_core.CvSize width(int i);

        static {
            Loader.load();
        }

        public AbstractCvSize(Pointer p) {
            super(p);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + width() + ", " + height() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(width());
                sb.append(", ");
                sb.append(height());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvSize2D32f extends FloatPointer {
        public abstract float height();

        public abstract opencv_core.CvSize2D32f height(float f);

        public abstract float width();

        public abstract opencv_core.CvSize2D32f width(float f);

        static {
            Loader.load();
        }

        public AbstractCvSize2D32f(Pointer p) {
            super(p);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + width() + ", " + height() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(width());
                sb.append(", ");
                sb.append(height());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvBox2D extends FloatPointer {
        public abstract float angle();

        public abstract opencv_core.CvBox2D angle(float f);

        public abstract opencv_core.CvBox2D center(opencv_core.CvPoint2D32f cvPoint2D32f);

        public abstract opencv_core.CvPoint2D32f center();

        public abstract opencv_core.CvBox2D size(opencv_core.CvSize2D32f cvSize2D32f);

        public abstract opencv_core.CvSize2D32f size();

        static {
            Loader.load();
        }

        public AbstractCvBox2D(Pointer p) {
            super(p);
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + center() + ", " + size() + ", " + angle() + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append(center());
                sb.append(", ");
                sb.append(size());
                sb.append(", ");
                sb.append(angle());
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static abstract class AbstractCvScalar extends DoublePointer {
        public static final opencv_core.CvScalar ALPHA1 = new opencv_core.CvScalar().val(0, opencv_stitching.Stitcher.ORIG_RESOL).val(1, opencv_stitching.Stitcher.ORIG_RESOL).val(2, opencv_stitching.Stitcher.ORIG_RESOL).val(3, 1.0d);
        public static final opencv_core.CvScalar ALPHA255 = new opencv_core.CvScalar().val(0, opencv_stitching.Stitcher.ORIG_RESOL).val(1, opencv_stitching.Stitcher.ORIG_RESOL).val(2, opencv_stitching.Stitcher.ORIG_RESOL).val(3, 255.0d);
        public static final opencv_core.CvScalar BLACK = opencv_core.CV_RGB(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.CvScalar BLUE = opencv_core.CV_RGB(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 255.0d);
        public static final opencv_core.CvScalar CYAN = opencv_core.CV_RGB(opencv_stitching.Stitcher.ORIG_RESOL, 255.0d, 255.0d);
        public static final opencv_core.CvScalar GRAY = opencv_core.CV_RGB(128.0d, 128.0d, 128.0d);
        public static final opencv_core.CvScalar GREEN = opencv_core.CV_RGB(opencv_stitching.Stitcher.ORIG_RESOL, 255.0d, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.CvScalar MAGENTA = opencv_core.CV_RGB(255.0d, opencv_stitching.Stitcher.ORIG_RESOL, 255.0d);
        public static final opencv_core.CvScalar ONE = new opencv_core.CvScalar().val(0, 1.0d).val(1, 1.0d).val(2, 1.0d).val(3, 1.0d);
        public static final opencv_core.CvScalar ONEHALF = new opencv_core.CvScalar().val(0, 0.5d).val(1, 0.5d).val(2, 0.5d).val(3, 0.5d);
        public static final opencv_core.CvScalar RED = opencv_core.CV_RGB(255.0d, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.CvScalar WHITE = opencv_core.CV_RGB(255.0d, 255.0d, 255.0d);
        public static final opencv_core.CvScalar YELLOW = opencv_core.CV_RGB(255.0d, 255.0d, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.CvScalar ZERO = new opencv_core.CvScalar().val(0, opencv_stitching.Stitcher.ORIG_RESOL).val(1, opencv_stitching.Stitcher.ORIG_RESOL).val(2, opencv_stitching.Stitcher.ORIG_RESOL).val(3, opencv_stitching.Stitcher.ORIG_RESOL);

        public abstract double val(int i);

        public abstract DoublePointer val();

        public abstract opencv_core.CvScalar val(int i, double d);

        static {
            Loader.load();
        }

        public AbstractCvScalar(Pointer p) {
            super(p);
        }

        public double getVal(int i) {
            return val(i);
        }

        public opencv_core.CvScalar setVal(int i, double val) {
            return val(i, val);
        }

        public DoublePointer getDoublePointerVal() {
            return val();
        }

        public LongPointer getLongPointerVal() {
            return new LongPointer((Pointer) val());
        }

        public void scale(double s) {
            for (int i = 0; i < 4; i++) {
                val(i, val(i) * s);
            }
        }

        public double red() {
            return val(2);
        }

        public double green() {
            return val(1);
        }

        public double blue() {
            return val(0);
        }

        public opencv_core.CvScalar red(double r) {
            val(2, r);
            return (opencv_core.CvScalar) this;
        }

        public opencv_core.CvScalar green(double g) {
            val(1, g);
            return (opencv_core.CvScalar) this;
        }

        public opencv_core.CvScalar blue(double b) {
            val(0, b);
            return (opencv_core.CvScalar) this;
        }

        public double magnitude() {
            return Math.sqrt((val(0) * val(0)) + (val(1) * val(1)) + (val(2) * val(2)) + (val(3) * val(3)));
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            if (capacity() == 0) {
                return "(" + ((float) val(0)) + ", " + ((float) val(1)) + ", " + ((float) val(2)) + ", " + ((float) val(3)) + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append((float) val(0));
                sb.append(", ");
                sb.append((float) val(1));
                sb.append(", ");
                sb.append((float) val(2));
                sb.append(", ");
                sb.append((float) val(3));
                sb.append(")");
                s = sb.toString();
                i++;
            }
            position(p);
            return s;
        }
    }

    public static opencv_core.CvScalar CV_RGB(double r, double g, double b) {
        return org.bytedeco.javacpp.opencv_core.cvScalar(b, g, r, opencv_stitching.Stitcher.ORIG_RESOL);
    }

    public static abstract class AbstractCvMemStorage extends Pointer {
        static {
            Loader.load();
        }

        public AbstractCvMemStorage(Pointer p) {
            super(p);
        }

        public static opencv_core.CvMemStorage create(int block_size) {
            opencv_core.CvMemStorage m = org.bytedeco.javacpp.opencv_core.cvCreateMemStorage(block_size);
            if (m != null) {
                m.deallocator(new ReleaseDeallocator(m));
            }
            return m;
        }

        public static opencv_core.CvMemStorage create() {
            return create(0);
        }

        public void release() {
            deallocate();
        }

        protected static class ReleaseDeallocator extends opencv_core.CvMemStorage implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.CvMemStorage p) {
                super((Pointer) p);
            }

            public void deallocate() {
                org.bytedeco.javacpp.opencv_core.cvReleaseMemStorage((opencv_core.CvMemStorage) this);
            }
        }
    }

    public static abstract class AbstractCvSeq extends CvArr {
        public AbstractCvSeq(Pointer p) {
            super(p);
        }

        public static opencv_core.CvSeq create(int seq_flags, int header_size, int elem_size, opencv_core.CvMemStorage storage) {
            return org.bytedeco.javacpp.opencv_core.cvCreateSeq(seq_flags, (long) header_size, (long) elem_size, storage);
        }
    }

    public static abstract class AbstractCvSet extends opencv_core.CvSeq {
        public AbstractCvSet(Pointer p) {
            super(p);
        }

        public static opencv_core.CvSet create(int set_flags, int header_size, int elem_size, opencv_core.CvMemStorage storage) {
            return org.bytedeco.javacpp.opencv_core.cvCreateSet(set_flags, header_size, elem_size, storage);
        }
    }

    public static abstract class AbstractCvGraph extends opencv_core.CvSet {
        public AbstractCvGraph(Pointer p) {
            super(p);
        }

        public static opencv_core.CvGraph create(int graph_flags, int header_size, int vtx_size, int edge_size, opencv_core.CvMemStorage storage) {
            return org.bytedeco.javacpp.opencv_core.cvCreateGraph(graph_flags, header_size, vtx_size, edge_size, storage);
        }
    }

    public static abstract class AbstractCvGraphScanner extends Pointer {
        public AbstractCvGraphScanner(Pointer p) {
            super(p);
        }

        public static opencv_core.CvGraphScanner create(opencv_core.CvGraph graph, opencv_core.CvGraphVtx vtx, int mask) {
            opencv_core.CvGraphScanner g = org.bytedeco.javacpp.opencv_core.cvCreateGraphScanner(graph, vtx, mask);
            if (g != null) {
                g.deallocator(new ReleaseDeallocator(g));
            }
            return g;
        }

        public void release() {
            deallocate();
        }

        protected static class ReleaseDeallocator extends opencv_core.CvGraphScanner implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.CvGraphScanner p) {
                super((Pointer) p);
            }

            public void deallocate() {
                org.bytedeco.javacpp.opencv_core.cvReleaseGraphScanner((opencv_core.CvGraphScanner) this);
            }
        }
    }

    public static int cvInitNArrayIterator(int count, CvArr[] arrs, CvArr mask, opencv_core.CvMatND stubs, opencv_core.CvNArrayIterator array_iterator, int flags) {
        return org.bytedeco.javacpp.opencv_core.cvInitNArrayIterator(count, (PointerPointer) new CvArrArray(arrs), mask, stubs, array_iterator, flags);
    }

    public static void cvMixChannels(CvArr[] src, int src_count, CvArr[] dst, int dst_count, int[] from_to, int pair_count) {
        org.bytedeco.javacpp.opencv_core.cvMixChannels((PointerPointer) new CvArrArray(src), src_count, (PointerPointer) new CvArrArray(dst), dst_count, new IntPointer(from_to), pair_count);
    }

    public static void cvCalcCovarMatrix(CvArr[] vects, int count, CvArr cov_mat, CvArr avg, int flags) {
        org.bytedeco.javacpp.opencv_core.cvCalcCovarMatrix((PointerPointer) new CvArrArray(vects), count, cov_mat, avg, flags);
    }

    public static double cvNorm(CvArr arr1, CvArr arr2) {
        return org.bytedeco.javacpp.opencv_core.cvNorm(arr1, arr2, 4, (CvArr) null);
    }

    public static abstract class AbstractCvFont extends Pointer {
        public AbstractCvFont(Pointer p) {
            super(p);
        }
    }

    public static abstract class AbstractMat extends AbstractArray {
        static final /* synthetic */ boolean $assertionsDisabled = false;
        public static final opencv_core.Mat EMPTY = null;

        public abstract int channels();

        public abstract int cols();

        public abstract void create(int i, int i2, int i3);

        public abstract BytePointer data();

        public abstract int depth();

        public abstract int dims();

        public abstract long elemSize1();

        public abstract void release();

        public abstract int rows();

        public abstract int size(int i);

        public abstract int step(int i);

        public abstract int type();

        static {
            Class<opencv_core> cls = opencv_core.class;
        }

        public AbstractMat(Pointer p) {
            super(p);
        }

        public int arrayChannels() {
            return channels();
        }

        public int arrayDepth() {
            switch (depth()) {
                case 0:
                    return 8;
                case 1:
                    return org.bytedeco.javacpp.opencv_core.IPL_DEPTH_8S;
                case 2:
                    return 16;
                case 3:
                    return org.bytedeco.javacpp.opencv_core.IPL_DEPTH_16S;
                case 4:
                    return org.bytedeco.javacpp.opencv_core.IPL_DEPTH_32S;
                case 5:
                    return 32;
                case 6:
                    return 64;
                default:
                    return -1;
            }
        }

        public int arrayOrigin() {
            return 0;
        }

        public void arrayOrigin(int origin) {
        }

        public int arrayWidth() {
            return cols();
        }

        public int arrayHeight() {
            return rows();
        }

        public opencv_core.IplROI arrayROI() {
            return null;
        }

        public int arraySize() {
            return step(0) * size(0);
        }

        public BytePointer arrayData() {
            return data();
        }

        public int arrayStep() {
            return step(0);
        }

        public <I extends Indexer> I createIndexer(boolean direct) {
            boolean z = direct;
            BytePointer ptr = arrayData();
            int size = arraySize();
            int dims = dims();
            int depth = depth();
            long elemSize = elemSize1();
            long[] sizes = new long[(dims + 1)];
            long[] strides = new long[(dims + 1)];
            int i = 0;
            while (i < dims) {
                sizes[i] = (long) size(i);
                int step = step(i);
                if (((long) step) % elemSize == 0) {
                    strides[i] = ((long) step) / elemSize;
                    i++;
                } else {
                    throw new UnsupportedOperationException("Step is not a multiple of element size");
                }
            }
            sizes[dims] = (long) arrayChannels();
            strides[dims] = 1;
            switch (depth) {
                case 0:
                    return UByteIndexer.create(ptr.capacity((long) size), sizes, strides, z).indexable(this);
                case 1:
                    return ByteIndexer.create(ptr.capacity((long) size), sizes, strides, z).indexable(this);
                case 2:
                    return UShortIndexer.create(new ShortPointer((Pointer) ptr).capacity((long) (size / 2)), sizes, strides, z).indexable(this);
                case 3:
                    return ShortIndexer.create(new ShortPointer((Pointer) ptr).capacity((long) (size / 2)), sizes, strides, z).indexable(this);
                case 4:
                    return IntIndexer.create(new IntPointer((Pointer) ptr).capacity((long) (size / 4)), sizes, strides, z).indexable(this);
                case 5:
                    return FloatIndexer.create(new FloatPointer((Pointer) ptr).capacity((long) (size / 4)), sizes, strides, z).indexable(this);
                case 6:
                    return DoubleIndexer.create(new DoublePointer((Pointer) ptr).capacity((long) (size / 8)), sizes, strides, z).indexable(this);
                default:
                    return null;
            }
        }
    }

    public static abstract class AbstractScalar extends DoublePointer {
        public static final opencv_core.Scalar ALPHA1 = new opencv_core.Scalar(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 1.0d);
        public static final opencv_core.Scalar ALPHA255 = new opencv_core.Scalar(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 255.0d);
        public static final opencv_core.Scalar BLACK = opencv_core.RGB(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.Scalar BLUE = opencv_core.RGB(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 255.0d);
        public static final opencv_core.Scalar CYAN = opencv_core.RGB(opencv_stitching.Stitcher.ORIG_RESOL, 255.0d, 255.0d);
        public static final opencv_core.Scalar GRAY = opencv_core.RGB(128.0d, 128.0d, 128.0d);
        public static final opencv_core.Scalar GREEN = opencv_core.RGB(opencv_stitching.Stitcher.ORIG_RESOL, 255.0d, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.Scalar MAGENTA = opencv_core.RGB(255.0d, opencv_stitching.Stitcher.ORIG_RESOL, 255.0d);
        public static final opencv_core.Scalar ONE = new opencv_core.Scalar(1.0d, 1.0d, 1.0d, 1.0d);
        public static final opencv_core.Scalar ONEHALF = new opencv_core.Scalar(0.5d, 0.5d, 0.5d, 0.5d);
        public static final opencv_core.Scalar RED = opencv_core.RGB(255.0d, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.Scalar WHITE = opencv_core.RGB(255.0d, 255.0d, 255.0d);
        public static final opencv_core.Scalar YELLOW = opencv_core.RGB(255.0d, 255.0d, opencv_stitching.Stitcher.ORIG_RESOL);
        public static final opencv_core.Scalar ZERO = new opencv_core.Scalar(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);

        static {
            Loader.load();
        }

        public AbstractScalar(Pointer p) {
            super(p);
        }

        public void scale(double s) {
            for (int i = 0; i < 4; i++) {
                put((long) i, get((long) i) * s);
            }
        }

        public double red() {
            return get(2);
        }

        public double green() {
            return get(1);
        }

        public double blue() {
            return get(0);
        }

        public opencv_core.Scalar red(double r) {
            put(2, r);
            return (opencv_core.Scalar) this;
        }

        public opencv_core.Scalar green(double g) {
            put(1, g);
            return (opencv_core.Scalar) this;
        }

        public opencv_core.Scalar blue(double b) {
            put(0, b);
            return (opencv_core.Scalar) this;
        }

        public double magnitude() {
            return Math.sqrt((get(0) * get(0)) + (get(1) * get(1)) + (get(2) * get(2)) + (get(3) * get(3)));
        }

        public String toString() {
            if (isNull()) {
                return super.toString();
            }
            long j = 2;
            if (capacity() == 0) {
                return "(" + ((float) get(0)) + ", " + ((float) get(1)) + ", " + ((float) get(2)) + ", " + ((float) get(3)) + ")";
            }
            long p = position();
            String s = "";
            long i = 0;
            while (i < capacity()) {
                position(i);
                StringBuilder sb = new StringBuilder();
                sb.append(s);
                sb.append(i == 0 ? "(" : " (");
                sb.append((float) get(0));
                sb.append(", ");
                sb.append((float) get(1));
                sb.append(", ");
                sb.append((float) get(j));
                sb.append(", ");
                sb.append((float) get(3));
                sb.append(")");
                s = sb.toString();
                i++;
                j = 2;
            }
            position(p);
            return s;
        }
    }

    public static opencv_core.Scalar RGB(double r, double g, double b) {
        return new opencv_core.Scalar(b, g, r, opencv_stitching.Stitcher.ORIG_RESOL);
    }
}
