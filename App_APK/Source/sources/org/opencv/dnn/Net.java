package org.opencv.dnn;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.Scalar;
import org.opencv.utils.Converters;

public class Net {
    protected final long nativeObj;

    private static native long Net_0();

    private static native void connect_0(long j, String str, String str2);

    private static native void delete(long j);

    private static native boolean empty_0(long j);

    private static native void enableFusion_0(long j, boolean z);

    private static native long forward_0(long j, String str);

    private static native long forward_1(long j);

    private static native void forward_2(long j, long j2, String str);

    private static native void forward_3(long j, long j2);

    private static native void forward_4(long j, long j2, List<String> list);

    private static native long getFLOPS_0(long j, long j2);

    private static native long getFLOPS_1(long j, int i, long j2);

    private static native long getFLOPS_2(long j, int i, List<MatOfInt> list);

    private static native long getFLOPS_3(long j, List<MatOfInt> list);

    private static native int getLayerId_0(long j, String str);

    private static native List<String> getLayerNames_0(long j);

    private static native void getLayerTypes_0(long j, List<String> list);

    private static native long getLayer_0(long j, long j2);

    private static native int getLayersCount_0(long j, String str);

    private static native void getMemoryConsumption_0(long j, long j2, double[] dArr, double[] dArr2);

    private static native void getMemoryConsumption_1(long j, int i, long j2, double[] dArr, double[] dArr2);

    private static native void getMemoryConsumption_2(long j, int i, List<MatOfInt> list, double[] dArr, double[] dArr2);

    private static native long getParam_0(long j, long j2, int i);

    private static native long getParam_1(long j, long j2);

    private static native long getPerfProfile_0(long j, long j2);

    private static native List<String> getUnconnectedOutLayersNames_0(long j);

    private static native long getUnconnectedOutLayers_0(long j);

    private static native long readFromModelOptimizer_0(String str, String str2);

    private static native void setHalideScheduler_0(long j, String str);

    private static native void setInput_0(long j, long j2, String str, double d, double d2, double d3, double d4, double d5);

    private static native void setInput_1(long j, long j2, String str, double d);

    private static native void setInput_2(long j, long j2, String str);

    private static native void setInput_3(long j, long j2);

    private static native void setInputsNames_0(long j, List<String> list);

    private static native void setParam_0(long j, long j2, int i, long j3);

    private static native void setPreferableBackend_0(long j, int i);

    private static native void setPreferableTarget_0(long j, int i);

    protected Net(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static Net __fromPtr__(long addr) {
        return new Net(addr);
    }

    public Net() {
        this.nativeObj = Net_0();
    }

    public Mat forward(String outputName) {
        return new Mat(forward_0(this.nativeObj, outputName));
    }

    public Mat forward() {
        return new Mat(forward_1(this.nativeObj));
    }

    public Mat getParam(DictValue layer, int numParam) {
        return new Mat(getParam_0(this.nativeObj, layer.getNativeObjAddr(), numParam));
    }

    public Mat getParam(DictValue layer) {
        return new Mat(getParam_1(this.nativeObj, layer.getNativeObjAddr()));
    }

    public static Net readFromModelOptimizer(String xml, String bin) {
        return new Net(readFromModelOptimizer_0(xml, bin));
    }

    public Layer getLayer(DictValue layerId) {
        return Layer.__fromPtr__(getLayer_0(this.nativeObj, layerId.getNativeObjAddr()));
    }

    public boolean empty() {
        return empty_0(this.nativeObj);
    }

    public int getLayerId(String layer) {
        return getLayerId_0(this.nativeObj, layer);
    }

    public int getLayersCount(String layerType) {
        return getLayersCount_0(this.nativeObj, layerType);
    }

    public long getFLOPS(MatOfInt netInputShape) {
        return getFLOPS_0(this.nativeObj, netInputShape.nativeObj);
    }

    public long getFLOPS(int layerId, MatOfInt netInputShape) {
        return getFLOPS_1(this.nativeObj, layerId, netInputShape.nativeObj);
    }

    public long getFLOPS(int layerId, List<MatOfInt> netInputShapes) {
        return getFLOPS_2(this.nativeObj, layerId, netInputShapes);
    }

    public long getFLOPS(List<MatOfInt> netInputShapes) {
        return getFLOPS_3(this.nativeObj, netInputShapes);
    }

    public long getPerfProfile(MatOfDouble timings) {
        return getPerfProfile_0(this.nativeObj, timings.nativeObj);
    }

    public List<String> getLayerNames() {
        return getLayerNames_0(this.nativeObj);
    }

    public List<String> getUnconnectedOutLayersNames() {
        return getUnconnectedOutLayersNames_0(this.nativeObj);
    }

    public MatOfInt getUnconnectedOutLayers() {
        return MatOfInt.fromNativeAddr(getUnconnectedOutLayers_0(this.nativeObj));
    }

    public void connect(String outPin, String inpPin) {
        connect_0(this.nativeObj, outPin, inpPin);
    }

    public void enableFusion(boolean fusion) {
        enableFusion_0(this.nativeObj, fusion);
    }

    public void forward(List<Mat> outputBlobs, String outputName) {
        Mat outputBlobs_mat = new Mat();
        forward_2(this.nativeObj, outputBlobs_mat.nativeObj, outputName);
        Converters.Mat_to_vector_Mat(outputBlobs_mat, outputBlobs);
        outputBlobs_mat.release();
    }

    public void forward(List<Mat> outputBlobs) {
        Mat outputBlobs_mat = new Mat();
        forward_3(this.nativeObj, outputBlobs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(outputBlobs_mat, outputBlobs);
        outputBlobs_mat.release();
    }

    public void forward(List<Mat> outputBlobs, List<String> outBlobNames) {
        Mat outputBlobs_mat = new Mat();
        forward_4(this.nativeObj, outputBlobs_mat.nativeObj, outBlobNames);
        Converters.Mat_to_vector_Mat(outputBlobs_mat, outputBlobs);
        outputBlobs_mat.release();
    }

    public void getLayerTypes(List<String> layersTypes) {
        getLayerTypes_0(this.nativeObj, layersTypes);
    }

    public void getMemoryConsumption(MatOfInt netInputShape, long[] weights, long[] blobs) {
        double[] weights_out = new double[1];
        double[] blobs_out = new double[1];
        getMemoryConsumption_0(this.nativeObj, netInputShape.nativeObj, weights_out, blobs_out);
        if (weights != null) {
            weights[0] = (long) weights_out[0];
        }
        if (blobs != null) {
            blobs[0] = (long) blobs_out[0];
        }
    }

    public void getMemoryConsumption(int layerId, MatOfInt netInputShape, long[] weights, long[] blobs) {
        double[] weights_out = new double[1];
        double[] blobs_out = new double[1];
        getMemoryConsumption_1(this.nativeObj, layerId, netInputShape.nativeObj, weights_out, blobs_out);
        if (weights != null) {
            weights[0] = (long) weights_out[0];
        }
        if (blobs != null) {
            blobs[0] = (long) blobs_out[0];
        }
    }

    public void getMemoryConsumption(int layerId, List<MatOfInt> netInputShapes, long[] weights, long[] blobs) {
        double[] weights_out = new double[1];
        double[] blobs_out = new double[1];
        getMemoryConsumption_2(this.nativeObj, layerId, netInputShapes, weights_out, blobs_out);
        if (weights != null) {
            weights[0] = (long) weights_out[0];
        }
        if (blobs != null) {
            blobs[0] = (long) blobs_out[0];
        }
    }

    public void setHalideScheduler(String scheduler) {
        setHalideScheduler_0(this.nativeObj, scheduler);
    }

    public void setInput(Mat blob, String name, double scalefactor, Scalar mean) {
        Scalar scalar = mean;
        setInput_0(this.nativeObj, blob.nativeObj, name, scalefactor, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public void setInput(Mat blob, String name, double scalefactor) {
        setInput_1(this.nativeObj, blob.nativeObj, name, scalefactor);
    }

    public void setInput(Mat blob, String name) {
        setInput_2(this.nativeObj, blob.nativeObj, name);
    }

    public void setInput(Mat blob) {
        setInput_3(this.nativeObj, blob.nativeObj);
    }

    public void setInputsNames(List<String> inputBlobNames) {
        setInputsNames_0(this.nativeObj, inputBlobNames);
    }

    public void setParam(DictValue layer, int numParam, Mat blob) {
        setParam_0(this.nativeObj, layer.getNativeObjAddr(), numParam, blob.nativeObj);
    }

    public void setPreferableBackend(int backendId) {
        setPreferableBackend_0(this.nativeObj, backendId);
    }

    public void setPreferableTarget(int targetId) {
        setPreferableTarget_0(this.nativeObj, targetId);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
