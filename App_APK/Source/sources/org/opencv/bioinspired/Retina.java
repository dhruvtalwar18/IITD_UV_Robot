package org.opencv.bioinspired;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.Size;

public class Retina extends Algorithm {
    private static native void activateContoursProcessing_0(long j, boolean z);

    private static native void activateMovingContoursProcessing_0(long j, boolean z);

    private static native void applyFastToneMapping_0(long j, long j2, long j3);

    private static native void clearBuffers_0(long j);

    private static native long create_0(double d, double d2, boolean z, int i, boolean z2, float f, float f2);

    private static native long create_1(double d, double d2, boolean z, int i, boolean z2, float f);

    private static native long create_2(double d, double d2, boolean z, int i, boolean z2);

    private static native long create_3(double d, double d2, boolean z, int i);

    private static native long create_4(double d, double d2, boolean z);

    private static native long create_5(double d, double d2);

    private static native void delete(long j);

    private static native double[] getInputSize_0(long j);

    private static native long getMagnoRAW_0(long j);

    private static native void getMagnoRAW_1(long j, long j2);

    private static native void getMagno_0(long j, long j2);

    private static native double[] getOutputSize_0(long j);

    private static native long getParvoRAW_0(long j);

    private static native void getParvoRAW_1(long j, long j2);

    private static native void getParvo_0(long j, long j2);

    private static native String printSetup_0(long j);

    private static native void run_0(long j, long j2);

    private static native void setColorSaturation_0(long j, boolean z, float f);

    private static native void setColorSaturation_1(long j, boolean z);

    private static native void setColorSaturation_2(long j);

    private static native void setupIPLMagnoChannel_0(long j, boolean z, float f, float f2, float f3, float f4, float f5, float f6, float f7);

    private static native void setupIPLMagnoChannel_1(long j, boolean z, float f, float f2, float f3, float f4, float f5, float f6);

    private static native void setupIPLMagnoChannel_2(long j, boolean z, float f, float f2, float f3, float f4, float f5);

    private static native void setupIPLMagnoChannel_3(long j, boolean z, float f, float f2, float f3, float f4);

    private static native void setupIPLMagnoChannel_4(long j, boolean z, float f, float f2, float f3);

    private static native void setupIPLMagnoChannel_5(long j, boolean z, float f, float f2);

    private static native void setupIPLMagnoChannel_6(long j, boolean z, float f);

    private static native void setupIPLMagnoChannel_7(long j, boolean z);

    private static native void setupIPLMagnoChannel_8(long j);

    private static native void setupOPLandIPLParvoChannel_0(long j, boolean z, boolean z2, float f, float f2, float f3, float f4, float f5, float f6, float f7);

    private static native void setupOPLandIPLParvoChannel_1(long j, boolean z, boolean z2, float f, float f2, float f3, float f4, float f5, float f6);

    private static native void setupOPLandIPLParvoChannel_2(long j, boolean z, boolean z2, float f, float f2, float f3, float f4, float f5);

    private static native void setupOPLandIPLParvoChannel_3(long j, boolean z, boolean z2, float f, float f2, float f3, float f4);

    private static native void setupOPLandIPLParvoChannel_4(long j, boolean z, boolean z2, float f, float f2, float f3);

    private static native void setupOPLandIPLParvoChannel_5(long j, boolean z, boolean z2, float f, float f2);

    private static native void setupOPLandIPLParvoChannel_6(long j, boolean z, boolean z2, float f);

    private static native void setupOPLandIPLParvoChannel_7(long j, boolean z, boolean z2);

    private static native void setupOPLandIPLParvoChannel_8(long j, boolean z);

    private static native void setupOPLandIPLParvoChannel_9(long j);

    private static native void setup_0(long j, String str, boolean z);

    private static native void setup_1(long j, String str);

    private static native void setup_2(long j);

    private static native void write_0(long j, String str);

    protected Retina(long addr) {
        super(addr);
    }

    public static Retina __fromPtr__(long addr) {
        return new Retina(addr);
    }

    public Mat getMagnoRAW() {
        return new Mat(getMagnoRAW_0(this.nativeObj));
    }

    public Mat getParvoRAW() {
        return new Mat(getParvoRAW_0(this.nativeObj));
    }

    public static Retina create(Size inputSize, boolean colorMode, int colorSamplingMethod, boolean useRetinaLogSampling, float reductionFactor, float samplingStrenght) {
        return __fromPtr__(create_0(inputSize.width, inputSize.height, colorMode, colorSamplingMethod, useRetinaLogSampling, reductionFactor, samplingStrenght));
    }

    public static Retina create(Size inputSize, boolean colorMode, int colorSamplingMethod, boolean useRetinaLogSampling, float reductionFactor) {
        return __fromPtr__(create_1(inputSize.width, inputSize.height, colorMode, colorSamplingMethod, useRetinaLogSampling, reductionFactor));
    }

    public static Retina create(Size inputSize, boolean colorMode, int colorSamplingMethod, boolean useRetinaLogSampling) {
        return __fromPtr__(create_2(inputSize.width, inputSize.height, colorMode, colorSamplingMethod, useRetinaLogSampling));
    }

    public static Retina create(Size inputSize, boolean colorMode, int colorSamplingMethod) {
        return __fromPtr__(create_3(inputSize.width, inputSize.height, colorMode, colorSamplingMethod));
    }

    public static Retina create(Size inputSize, boolean colorMode) {
        return __fromPtr__(create_4(inputSize.width, inputSize.height, colorMode));
    }

    public static Retina create(Size inputSize) {
        return __fromPtr__(create_5(inputSize.width, inputSize.height));
    }

    public Size getInputSize() {
        return new Size(getInputSize_0(this.nativeObj));
    }

    public Size getOutputSize() {
        return new Size(getOutputSize_0(this.nativeObj));
    }

    public String printSetup() {
        return printSetup_0(this.nativeObj);
    }

    public void activateContoursProcessing(boolean activate) {
        activateContoursProcessing_0(this.nativeObj, activate);
    }

    public void activateMovingContoursProcessing(boolean activate) {
        activateMovingContoursProcessing_0(this.nativeObj, activate);
    }

    public void applyFastToneMapping(Mat inputImage, Mat outputToneMappedImage) {
        applyFastToneMapping_0(this.nativeObj, inputImage.nativeObj, outputToneMappedImage.nativeObj);
    }

    public void clearBuffers() {
        clearBuffers_0(this.nativeObj);
    }

    public void getMagno(Mat retinaOutput_magno) {
        getMagno_0(this.nativeObj, retinaOutput_magno.nativeObj);
    }

    public void getMagnoRAW(Mat retinaOutput_magno) {
        getMagnoRAW_1(this.nativeObj, retinaOutput_magno.nativeObj);
    }

    public void getParvo(Mat retinaOutput_parvo) {
        getParvo_0(this.nativeObj, retinaOutput_parvo.nativeObj);
    }

    public void getParvoRAW(Mat retinaOutput_parvo) {
        getParvoRAW_1(this.nativeObj, retinaOutput_parvo.nativeObj);
    }

    public void run(Mat inputImage) {
        run_0(this.nativeObj, inputImage.nativeObj);
    }

    public void setColorSaturation(boolean saturateColors, float colorSaturationValue) {
        setColorSaturation_0(this.nativeObj, saturateColors, colorSaturationValue);
    }

    public void setColorSaturation(boolean saturateColors) {
        setColorSaturation_1(this.nativeObj, saturateColors);
    }

    public void setColorSaturation() {
        setColorSaturation_2(this.nativeObj);
    }

    public void setup(String retinaParameterFile, boolean applyDefaultSetupOnFailure) {
        setup_0(this.nativeObj, retinaParameterFile, applyDefaultSetupOnFailure);
    }

    public void setup(String retinaParameterFile) {
        setup_1(this.nativeObj, retinaParameterFile);
    }

    public void setup() {
        setup_2(this.nativeObj);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta, float parasolCells_tau, float parasolCells_k, float amacrinCellsTemporalCutFrequency, float V0CompressionParameter, float localAdaptintegration_tau, float localAdaptintegration_k) {
        setupIPLMagnoChannel_0(this.nativeObj, normaliseOutput, parasolCells_beta, parasolCells_tau, parasolCells_k, amacrinCellsTemporalCutFrequency, V0CompressionParameter, localAdaptintegration_tau, localAdaptintegration_k);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta, float parasolCells_tau, float parasolCells_k, float amacrinCellsTemporalCutFrequency, float V0CompressionParameter, float localAdaptintegration_tau) {
        setupIPLMagnoChannel_1(this.nativeObj, normaliseOutput, parasolCells_beta, parasolCells_tau, parasolCells_k, amacrinCellsTemporalCutFrequency, V0CompressionParameter, localAdaptintegration_tau);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta, float parasolCells_tau, float parasolCells_k, float amacrinCellsTemporalCutFrequency, float V0CompressionParameter) {
        setupIPLMagnoChannel_2(this.nativeObj, normaliseOutput, parasolCells_beta, parasolCells_tau, parasolCells_k, amacrinCellsTemporalCutFrequency, V0CompressionParameter);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta, float parasolCells_tau, float parasolCells_k, float amacrinCellsTemporalCutFrequency) {
        setupIPLMagnoChannel_3(this.nativeObj, normaliseOutput, parasolCells_beta, parasolCells_tau, parasolCells_k, amacrinCellsTemporalCutFrequency);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta, float parasolCells_tau, float parasolCells_k) {
        setupIPLMagnoChannel_4(this.nativeObj, normaliseOutput, parasolCells_beta, parasolCells_tau, parasolCells_k);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta, float parasolCells_tau) {
        setupIPLMagnoChannel_5(this.nativeObj, normaliseOutput, parasolCells_beta, parasolCells_tau);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput, float parasolCells_beta) {
        setupIPLMagnoChannel_6(this.nativeObj, normaliseOutput, parasolCells_beta);
    }

    public void setupIPLMagnoChannel(boolean normaliseOutput) {
        setupIPLMagnoChannel_7(this.nativeObj, normaliseOutput);
    }

    public void setupIPLMagnoChannel() {
        setupIPLMagnoChannel_8(this.nativeObj);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity, float photoreceptorsTemporalConstant, float photoreceptorsSpatialConstant, float horizontalCellsGain, float HcellsTemporalConstant, float HcellsSpatialConstant, float ganglionCellsSensitivity) {
        setupOPLandIPLParvoChannel_0(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity, photoreceptorsTemporalConstant, photoreceptorsSpatialConstant, horizontalCellsGain, HcellsTemporalConstant, HcellsSpatialConstant, ganglionCellsSensitivity);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity, float photoreceptorsTemporalConstant, float photoreceptorsSpatialConstant, float horizontalCellsGain, float HcellsTemporalConstant, float HcellsSpatialConstant) {
        setupOPLandIPLParvoChannel_1(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity, photoreceptorsTemporalConstant, photoreceptorsSpatialConstant, horizontalCellsGain, HcellsTemporalConstant, HcellsSpatialConstant);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity, float photoreceptorsTemporalConstant, float photoreceptorsSpatialConstant, float horizontalCellsGain, float HcellsTemporalConstant) {
        setupOPLandIPLParvoChannel_2(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity, photoreceptorsTemporalConstant, photoreceptorsSpatialConstant, horizontalCellsGain, HcellsTemporalConstant);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity, float photoreceptorsTemporalConstant, float photoreceptorsSpatialConstant, float horizontalCellsGain) {
        setupOPLandIPLParvoChannel_3(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity, photoreceptorsTemporalConstant, photoreceptorsSpatialConstant, horizontalCellsGain);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity, float photoreceptorsTemporalConstant, float photoreceptorsSpatialConstant) {
        setupOPLandIPLParvoChannel_4(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity, photoreceptorsTemporalConstant, photoreceptorsSpatialConstant);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity, float photoreceptorsTemporalConstant) {
        setupOPLandIPLParvoChannel_5(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity, photoreceptorsTemporalConstant);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput, float photoreceptorsLocalAdaptationSensitivity) {
        setupOPLandIPLParvoChannel_6(this.nativeObj, colorMode, normaliseOutput, photoreceptorsLocalAdaptationSensitivity);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode, boolean normaliseOutput) {
        setupOPLandIPLParvoChannel_7(this.nativeObj, colorMode, normaliseOutput);
    }

    public void setupOPLandIPLParvoChannel(boolean colorMode) {
        setupOPLandIPLParvoChannel_8(this.nativeObj, colorMode);
    }

    public void setupOPLandIPLParvoChannel() {
        setupOPLandIPLParvoChannel_9(this.nativeObj);
    }

    public void write(String fs) {
        write_0(this.nativeObj, fs);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
