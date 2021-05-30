package org.ros.android.view.camera;

import android.hardware.Camera;

interface RawImageListener {
    void onNewRawImage(byte[] bArr, Camera.Size size);
}
