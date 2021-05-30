package org.opencv.plot;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public class Plot2d extends Algorithm {
    private static native long create_0(long j);

    private static native long create_1(long j, long j2);

    private static native void delete(long j);

    private static native void render_0(long j, long j2);

    private static native void setGridLinesNumber_0(long j, int i);

    private static native void setInvertOrientation_0(long j, boolean z);

    private static native void setMaxX_0(long j, double d);

    private static native void setMaxY_0(long j, double d);

    private static native void setMinX_0(long j, double d);

    private static native void setMinY_0(long j, double d);

    private static native void setNeedPlotLine_0(long j, boolean z);

    private static native void setPlotAxisColor_0(long j, double d, double d2, double d3, double d4);

    private static native void setPlotBackgroundColor_0(long j, double d, double d2, double d3, double d4);

    private static native void setPlotGridColor_0(long j, double d, double d2, double d3, double d4);

    private static native void setPlotLineColor_0(long j, double d, double d2, double d3, double d4);

    private static native void setPlotLineWidth_0(long j, int i);

    private static native void setPlotSize_0(long j, int i, int i2);

    private static native void setPlotTextColor_0(long j, double d, double d2, double d3, double d4);

    private static native void setPointIdxToPrint_0(long j, int i);

    private static native void setShowGrid_0(long j, boolean z);

    private static native void setShowText_0(long j, boolean z);

    protected Plot2d(long addr) {
        super(addr);
    }

    public static Plot2d __fromPtr__(long addr) {
        return new Plot2d(addr);
    }

    public static Plot2d create(Mat data) {
        return __fromPtr__(create_0(data.nativeObj));
    }

    public static Plot2d create(Mat dataX, Mat dataY) {
        return __fromPtr__(create_1(dataX.nativeObj, dataY.nativeObj));
    }

    public void render(Mat _plotResult) {
        render_0(this.nativeObj, _plotResult.nativeObj);
    }

    public void setGridLinesNumber(int gridLinesNumber) {
        setGridLinesNumber_0(this.nativeObj, gridLinesNumber);
    }

    public void setInvertOrientation(boolean _invertOrientation) {
        setInvertOrientation_0(this.nativeObj, _invertOrientation);
    }

    public void setMaxX(double _plotMaxX) {
        setMaxX_0(this.nativeObj, _plotMaxX);
    }

    public void setMaxY(double _plotMaxY) {
        setMaxY_0(this.nativeObj, _plotMaxY);
    }

    public void setMinX(double _plotMinX) {
        setMinX_0(this.nativeObj, _plotMinX);
    }

    public void setMinY(double _plotMinY) {
        setMinY_0(this.nativeObj, _plotMinY);
    }

    public void setNeedPlotLine(boolean _needPlotLine) {
        setNeedPlotLine_0(this.nativeObj, _needPlotLine);
    }

    public void setPlotAxisColor(Scalar _plotAxisColor) {
        setPlotAxisColor_0(this.nativeObj, _plotAxisColor.val[0], _plotAxisColor.val[1], _plotAxisColor.val[2], _plotAxisColor.val[3]);
    }

    public void setPlotBackgroundColor(Scalar _plotBackgroundColor) {
        setPlotBackgroundColor_0(this.nativeObj, _plotBackgroundColor.val[0], _plotBackgroundColor.val[1], _plotBackgroundColor.val[2], _plotBackgroundColor.val[3]);
    }

    public void setPlotGridColor(Scalar _plotGridColor) {
        setPlotGridColor_0(this.nativeObj, _plotGridColor.val[0], _plotGridColor.val[1], _plotGridColor.val[2], _plotGridColor.val[3]);
    }

    public void setPlotLineColor(Scalar _plotLineColor) {
        setPlotLineColor_0(this.nativeObj, _plotLineColor.val[0], _plotLineColor.val[1], _plotLineColor.val[2], _plotLineColor.val[3]);
    }

    public void setPlotLineWidth(int _plotLineWidth) {
        setPlotLineWidth_0(this.nativeObj, _plotLineWidth);
    }

    public void setPlotSize(int _plotSizeWidth, int _plotSizeHeight) {
        setPlotSize_0(this.nativeObj, _plotSizeWidth, _plotSizeHeight);
    }

    public void setPlotTextColor(Scalar _plotTextColor) {
        setPlotTextColor_0(this.nativeObj, _plotTextColor.val[0], _plotTextColor.val[1], _plotTextColor.val[2], _plotTextColor.val[3]);
    }

    public void setPointIdxToPrint(int pointIdx) {
        setPointIdxToPrint_0(this.nativeObj, pointIdx);
    }

    public void setShowGrid(boolean needShowGrid) {
        setShowGrid_0(this.nativeObj, needShowGrid);
    }

    public void setShowText(boolean needShowText) {
        setShowText_0(this.nativeObj, needShowText);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
