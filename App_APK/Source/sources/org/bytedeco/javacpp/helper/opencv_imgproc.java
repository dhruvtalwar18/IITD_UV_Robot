package org.bytedeco.javacpp.helper;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.javacpp.helper.opencv_core;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_imgproc;

public class opencv_imgproc extends org.bytedeco.javacpp.presets.opencv_imgproc {

    public static abstract class AbstractCvMoments extends Pointer {
        public AbstractCvMoments(Pointer p) {
            super(p);
        }

        public static ThreadLocal<opencv_imgproc.CvMoments> createThreadLocal() {
            return new ThreadLocal<opencv_imgproc.CvMoments>() {
                /* access modifiers changed from: protected */
                public opencv_imgproc.CvMoments initialValue() {
                    return new opencv_imgproc.CvMoments();
                }
            };
        }
    }

    public static int cvFindContours(opencv_core.CvArr image, opencv_core.CvMemStorage storage, opencv_core.CvSeq first_contour, int header_size, int mode, int method) {
        return org.bytedeco.javacpp.opencv_imgproc.cvFindContours(image, storage, first_contour, header_size, mode, method, opencv_core.CvPoint.ZERO);
    }

    public static opencv_imgproc.CvContourScanner cvStartFindContours(opencv_core.CvArr image, opencv_core.CvMemStorage storage, int header_size, int mode, int method) {
        return org.bytedeco.javacpp.opencv_imgproc.cvStartFindContours(image, storage, header_size, mode, method, opencv_core.CvPoint.ZERO);
    }

    public static abstract class AbstractIplConvKernel extends Pointer {
        public AbstractIplConvKernel(Pointer p) {
            super(p);
        }

        public static opencv_core.IplConvKernel create(int cols, int rows, int anchor_x, int anchor_y, int shape, int[] values) {
            opencv_core.IplConvKernel p = org.bytedeco.javacpp.opencv_imgproc.cvCreateStructuringElementEx(cols, rows, anchor_x, anchor_y, shape, values);
            if (p != null) {
                p.deallocator(new ReleaseDeallocator(p));
            }
            return p;
        }

        public void release() {
            deallocate();
        }

        static class ReleaseDeallocator extends opencv_core.IplConvKernel implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.IplConvKernel p) {
                super((Pointer) p);
            }

            public void deallocate() {
                org.bytedeco.javacpp.opencv_imgproc.cvReleaseStructuringElement((opencv_core.IplConvKernel) this);
            }
        }
    }

    public static abstract class AbstractCvHistogram extends Pointer {
        public AbstractCvHistogram(Pointer p) {
            super(p);
        }

        public static opencv_core.CvHistogram create(int dims, int[] sizes, int type, float[][] ranges, int uniform) {
            opencv_core.CvHistogram h = opencv_imgproc.cvCreateHist(dims, sizes, type, ranges, uniform);
            if (h != null) {
                h.deallocator(new ReleaseDeallocator(h));
            }
            return h;
        }

        public void release() {
            deallocate();
        }

        static class ReleaseDeallocator extends opencv_core.CvHistogram implements Pointer.Deallocator {
            ReleaseDeallocator(opencv_core.CvHistogram p) {
                super((Pointer) p);
            }

            public void deallocate() {
                org.bytedeco.javacpp.opencv_imgproc.cvReleaseHist((opencv_core.CvHistogram) this);
            }
        }
    }

    public static opencv_core.CvHistogram cvCreateHist(int dims, int[] sizes, int type, float[][] ranges, int uniform) {
        return org.bytedeco.javacpp.opencv_imgproc.cvCreateHist(dims, new IntPointer(sizes), type, ranges == null ? null : new PointerPointer(ranges), uniform);
    }

    public static void cvSetHistBinRanges(opencv_core.CvHistogram hist, float[][] ranges, int uniform) {
        org.bytedeco.javacpp.opencv_imgproc.cvSetHistBinRanges(hist, ranges == null ? null : new PointerPointer(ranges), uniform);
    }

    public static opencv_core.CvHistogram cvMakeHistHeaderForArray(int dims, int[] sizes, opencv_core.CvHistogram hist, float[] data, float[][] ranges, int uniform) {
        return org.bytedeco.javacpp.opencv_imgproc.cvMakeHistHeaderForArray(dims, new IntPointer(sizes), hist, new FloatPointer(data), ranges == null ? null : new PointerPointer(ranges), uniform);
    }

    public static opencv_core.CvHistogram cvMakeHistHeaderForArray(int dims, int[] sizes, opencv_core.CvHistogram hist, FloatPointer data, float[][] ranges, int uniform) {
        return org.bytedeco.javacpp.opencv_imgproc.cvMakeHistHeaderForArray(dims, new IntPointer(sizes), hist, new FloatPointer((Pointer) data), ranges == null ? null : new PointerPointer(ranges), uniform);
    }

    public static void cvCalcArrHist(opencv_core.CvArr[] arr, opencv_core.CvHistogram hist, int accumulate, opencv_core.CvArr mask) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcArrHist((PointerPointer) new opencv_core.CvArrArray(arr), hist, accumulate, mask);
    }

    public static void cvCalcHist(opencv_core.IplImage[] arr, opencv_core.CvHistogram hist, int accumulate, opencv_core.CvArr mask) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcHist(new opencv_core.IplImageArray(arr), hist, accumulate, mask);
    }

    public static void cvCalcHist(opencv_core.IplImageArray arr, opencv_core.CvHistogram hist, int accumulate, opencv_core.CvArr mask) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcArrHist((PointerPointer) arr, hist, accumulate, mask);
    }

    public static void cvCalcArrBackProject(opencv_core.CvArr[] image, opencv_core.CvArr dst, opencv_core.CvHistogram hist) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcArrBackProject((PointerPointer) new opencv_core.CvArrArray(image), dst, hist);
    }

    public static void cvCalcBackProject(opencv_core.IplImage[] image, opencv_core.CvArr dst, opencv_core.CvHistogram hist) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcBackProject(new opencv_core.IplImageArray(image), dst, hist);
    }

    public static void cvCalcBackProject(opencv_core.IplImageArray image, opencv_core.CvArr dst, opencv_core.CvHistogram hist) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcArrBackProject((PointerPointer) image, dst, hist);
    }

    public static void cvCalcArrBackProjectPatch(opencv_core.CvArr[] image, opencv_core.CvArr dst, opencv_core.CvSize range, opencv_core.CvHistogram hist, int method, double factor) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcArrBackProjectPatch((PointerPointer) new opencv_core.CvArrArray(image), dst, range, hist, method, factor);
    }

    public static void cvCalcBackProjectPatch(opencv_core.IplImage[] image, opencv_core.CvArr dst, opencv_core.CvSize range, opencv_core.CvHistogram hist, int method, double factor) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcBackProjectPatch(new opencv_core.IplImageArray(image), dst, range, hist, method, factor);
    }

    public static void cvCalcBackProjectPatch(opencv_core.IplImageArray image, opencv_core.CvArr dst, opencv_core.CvSize range, opencv_core.CvHistogram hist, int method, double factor) {
        org.bytedeco.javacpp.opencv_imgproc.cvCalcArrBackProjectPatch((PointerPointer) image, dst, range, hist, method, factor);
    }

    public static void cvFillPoly(opencv_core.CvArr img, opencv_core.CvPoint[] pts, int[] npts, int contours, opencv_core.CvScalar color, int line_type, int shift) {
        org.bytedeco.javacpp.opencv_imgproc.cvFillPoly(img, new PointerPointer((P[]) pts), new IntPointer(npts), contours, color, line_type, shift);
    }

    public static void cvPolyLine(opencv_core.CvArr img, opencv_core.CvPoint[] pts, int[] npts, int contours, int is_closed, opencv_core.CvScalar color, int thickness, int line_type, int shift) {
        opencv_core.CvPoint[] cvPointArr = pts;
        int[] iArr = npts;
        org.bytedeco.javacpp.opencv_imgproc.cvPolyLine(img, new PointerPointer((P[]) pts), new IntPointer(npts), contours, is_closed, color, thickness, line_type, shift);
    }

    public static void cvDrawPolyLine(opencv_core.CvArr img, opencv_core.CvPoint[] pts, int[] npts, int contours, int is_closed, opencv_core.CvScalar color, int thickness, int line_type, int shift) {
        cvPolyLine(img, pts, npts, contours, is_closed, color, thickness, line_type, shift);
    }

    public static void cvDrawContours(opencv_core.CvArr img, opencv_core.CvSeq contour, opencv_core.CvScalar external_color, opencv_core.CvScalar hole_color, int max_level, int thickness, int line_type) {
        org.bytedeco.javacpp.opencv_imgproc.cvDrawContours(img, contour, external_color, hole_color, max_level, thickness, line_type, opencv_core.CvPoint.ZERO);
    }
}
