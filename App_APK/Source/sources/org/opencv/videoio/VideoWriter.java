package org.opencv.videoio;

import org.opencv.core.Mat;
import org.opencv.core.Size;

public class VideoWriter {
    protected final long nativeObj;

    private static native long VideoWriter_0(String str, int i, int i2, double d, double d2, double d3, boolean z);

    private static native long VideoWriter_1(String str, int i, int i2, double d, double d2, double d3);

    private static native long VideoWriter_2(String str, int i, double d, double d2, double d3, boolean z);

    private static native long VideoWriter_3(String str, int i, double d, double d2, double d3);

    private static native long VideoWriter_4();

    private static native void delete(long j);

    private static native int fourcc_0(char c, char c2, char c3, char c4);

    private static native String getBackendName_0(long j);

    private static native double get_0(long j, int i);

    private static native boolean isOpened_0(long j);

    private static native boolean open_0(long j, String str, int i, int i2, double d, double d2, double d3, boolean z);

    private static native boolean open_1(long j, String str, int i, int i2, double d, double d2, double d3);

    private static native boolean open_2(long j, String str, int i, double d, double d2, double d3, boolean z);

    private static native boolean open_3(long j, String str, int i, double d, double d2, double d3);

    private static native void release_0(long j);

    private static native boolean set_0(long j, int i, double d);

    private static native void write_0(long j, long j2);

    protected VideoWriter(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static VideoWriter __fromPtr__(long addr) {
        return new VideoWriter(addr);
    }

    public VideoWriter(String filename, int apiPreference, int fourcc, double fps, Size frameSize, boolean isColor) {
        Size size = frameSize;
        this.nativeObj = VideoWriter_0(filename, apiPreference, fourcc, fps, size.width, size.height, isColor);
    }

    public VideoWriter(String filename, int apiPreference, int fourcc, double fps, Size frameSize) {
        this.nativeObj = VideoWriter_1(filename, apiPreference, fourcc, fps, frameSize.width, frameSize.height);
    }

    public VideoWriter(String filename, int fourcc, double fps, Size frameSize, boolean isColor) {
        this.nativeObj = VideoWriter_2(filename, fourcc, fps, frameSize.width, frameSize.height, isColor);
    }

    public VideoWriter(String filename, int fourcc, double fps, Size frameSize) {
        this.nativeObj = VideoWriter_3(filename, fourcc, fps, frameSize.width, frameSize.height);
    }

    public VideoWriter() {
        this.nativeObj = VideoWriter_4();
    }

    public String getBackendName() {
        return getBackendName_0(this.nativeObj);
    }

    public boolean isOpened() {
        return isOpened_0(this.nativeObj);
    }

    public boolean open(String filename, int apiPreference, int fourcc, double fps, Size frameSize, boolean isColor) {
        Size size = frameSize;
        return open_0(this.nativeObj, filename, apiPreference, fourcc, fps, size.width, size.height, isColor);
    }

    public boolean open(String filename, int apiPreference, int fourcc, double fps, Size frameSize) {
        Size size = frameSize;
        return open_1(this.nativeObj, filename, apiPreference, fourcc, fps, size.width, size.height);
    }

    public boolean open(String filename, int fourcc, double fps, Size frameSize, boolean isColor) {
        Size size = frameSize;
        return open_2(this.nativeObj, filename, fourcc, fps, size.width, size.height, isColor);
    }

    public boolean open(String filename, int fourcc, double fps, Size frameSize) {
        return open_3(this.nativeObj, filename, fourcc, fps, frameSize.width, frameSize.height);
    }

    public boolean set(int propId, double value) {
        return set_0(this.nativeObj, propId, value);
    }

    public double get(int propId) {
        return get_0(this.nativeObj, propId);
    }

    public static int fourcc(char c1, char c2, char c3, char c4) {
        return fourcc_0(c1, c2, c3, c4);
    }

    public void release() {
        release_0(this.nativeObj);
    }

    public void write(Mat image) {
        write_0(this.nativeObj, image.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
