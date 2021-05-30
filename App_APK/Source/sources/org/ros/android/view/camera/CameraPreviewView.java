package org.ros.android.view.camera;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import com.google.common.base.Preconditions;
import java.io.IOException;
import java.util.List;
import org.ros.exception.RosRuntimeException;

public class CameraPreviewView extends ViewGroup {
    private static final double ASPECT_TOLERANCE = 0.1d;
    private BufferingPreviewCallback bufferingPreviewCallback;
    /* access modifiers changed from: private */
    public Camera camera;
    /* access modifiers changed from: private */
    public byte[] previewBuffer;
    /* access modifiers changed from: private */
    public Camera.Size previewSize;
    /* access modifiers changed from: private */
    public RawImageListener rawImageListener;
    private SurfaceHolder surfaceHolder;

    private final class BufferingPreviewCallback implements Camera.PreviewCallback {
        private BufferingPreviewCallback() {
        }

        public void onPreviewFrame(byte[] data, Camera camera) {
            boolean z = false;
            Preconditions.checkArgument(camera == CameraPreviewView.this.camera);
            if (data == CameraPreviewView.this.previewBuffer) {
                z = true;
            }
            Preconditions.checkArgument(z);
            if (CameraPreviewView.this.rawImageListener != null) {
                CameraPreviewView.this.rawImageListener.onNewRawImage(data, CameraPreviewView.this.previewSize);
            }
            camera.addCallbackBuffer(CameraPreviewView.this.previewBuffer);
        }
    }

    private final class SurfaceHolderCallback implements SurfaceHolder.Callback {
        private SurfaceHolderCallback() {
        }

        public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        }

        public void surfaceCreated(SurfaceHolder holder) {
            try {
                if (CameraPreviewView.this.camera != null) {
                    CameraPreviewView.this.camera.setPreviewDisplay(holder);
                }
            } catch (IOException e) {
                throw new RosRuntimeException((Throwable) e);
            }
        }

        public void surfaceDestroyed(SurfaceHolder holder) {
            CameraPreviewView.this.releaseCamera();
        }
    }

    private void init(Context context) {
        SurfaceView surfaceView = new SurfaceView(context);
        addView(surfaceView);
        this.surfaceHolder = surfaceView.getHolder();
        this.surfaceHolder.addCallback(new SurfaceHolderCallback());
        this.surfaceHolder.setType(3);
        this.bufferingPreviewCallback = new BufferingPreviewCallback();
    }

    public CameraPreviewView(Context context) {
        super(context);
        init(context);
    }

    public CameraPreviewView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    public CameraPreviewView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(context);
    }

    public void releaseCamera() {
        if (this.camera != null) {
            this.camera.setPreviewCallbackWithBuffer((Camera.PreviewCallback) null);
            this.camera.stopPreview();
            this.camera.release();
            this.camera = null;
        }
    }

    public void setRawImageListener(RawImageListener rawImageListener2) {
        this.rawImageListener = rawImageListener2;
    }

    public Camera.Size getPreviewSize() {
        return this.previewSize;
    }

    public void setCamera(Camera camera2) {
        Preconditions.checkNotNull(camera2);
        this.camera = camera2;
        setupCameraParameters();
        setupBufferingPreviewCallback();
        camera2.startPreview();
        try {
            camera2.setPreviewDisplay(this.surfaceHolder);
        } catch (IOException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    private void setupCameraParameters() {
        Camera.Parameters parameters = this.camera.getParameters();
        this.previewSize = getOptimalPreviewSize(parameters.getSupportedPreviewSizes(), getWidth(), getHeight());
        parameters.setPreviewSize(this.previewSize.width, this.previewSize.height);
        parameters.setPreviewFormat(17);
        this.camera.setParameters(parameters);
    }

    private Camera.Size getOptimalPreviewSize(List<Camera.Size> sizes, int width, int height) {
        int i = height;
        Preconditions.checkNotNull(sizes);
        double d = (double) width;
        double d2 = (double) i;
        Double.isNaN(d);
        Double.isNaN(d2);
        double targetRatio = d / d2;
        double minimumDifference = Double.MAX_VALUE;
        Camera.Size optimalSize = null;
        for (Camera.Size size : sizes) {
            double d3 = (double) size.width;
            double d4 = (double) size.height;
            Double.isNaN(d3);
            Double.isNaN(d4);
            if (Math.abs((d3 / d4) - targetRatio) <= ASPECT_TOLERANCE && ((double) Math.abs(size.height - i)) < minimumDifference) {
                optimalSize = size;
                minimumDifference = (double) Math.abs(size.height - i);
            }
        }
        if (optimalSize == null) {
            double minimumDifference2 = Double.MAX_VALUE;
            for (Camera.Size size2 : sizes) {
                if (((double) Math.abs(size2.height - i)) < minimumDifference2) {
                    optimalSize = size2;
                    minimumDifference2 = (double) Math.abs(size2.height - i);
                }
            }
        }
        Preconditions.checkNotNull(optimalSize);
        return optimalSize;
    }

    private void setupBufferingPreviewCallback() {
        this.previewBuffer = new byte[(((this.previewSize.height * this.previewSize.width) * ImageFormat.getBitsPerPixel(this.camera.getParameters().getPreviewFormat())) / 8)];
        this.camera.addCallbackBuffer(this.previewBuffer);
        this.camera.setPreviewCallbackWithBuffer(this.bufferingPreviewCallback);
    }

    /* access modifiers changed from: protected */
    public void onLayout(boolean changed, int l, int t, int r, int b) {
        if (changed && getChildCount() > 0) {
            View child = getChildAt(0);
            int width = r - l;
            int height = b - t;
            int previewWidth = width;
            int previewHeight = height;
            if (this.previewSize != null) {
                previewWidth = this.previewSize.width;
                previewHeight = this.previewSize.height;
            }
            if (width * previewHeight > height * previewWidth) {
                int scaledChildWidth = (previewWidth * height) / previewHeight;
                child.layout((width - scaledChildWidth) / 2, 0, (width + scaledChildWidth) / 2, height);
                return;
            }
            int scaledChildHeight = (previewHeight * width) / previewWidth;
            child.layout(0, (height - scaledChildHeight) / 2, width, (height + scaledChildHeight) / 2);
        }
    }
}
