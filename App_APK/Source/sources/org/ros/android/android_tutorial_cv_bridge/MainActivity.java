package org.ros.android.android_tutorial_cv_bridge;

import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.location.Criteria;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.android.view.camera.CameraPreviewView;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity implements View.OnClickListener {
    /* access modifiers changed from: private */
    public static final String TAG = MainActivity.class.getSimpleName();
    Button applyB;
    Button autonomousB;
    public int autonomous_status = 0;
    Button backB;
    Button batteryB;
    /* access modifiers changed from: private */
    public int cameraId = 0;
    private ChannelBuffer cameraPreview;
    private CameraPreviewView cameraPreviewView;
    Button continueB;
    Button emergencyb;
    /* access modifiers changed from: private */
    public Handler handy = new Handler();
    private OnFrameIdChangeListener imuFrameIdListener;
    Button light_offB;
    Button light_onB;
    private OnFrameIdChangeListener locationFrameIdListener;
    public EditText locationFrameIdView;
    NodeConfiguration nodeConfiguration;
    Button resetB;
    /* access modifiers changed from: private */
    public RosCameraPreviewView rosCameraPreviewView;
    Runnable sizeCheckRunnable = new Runnable() {
        public void run() {
            if (MainActivity.this.rosCameraPreviewView.getHeight() == -1 || MainActivity.this.rosCameraPreviewView.getWidth() == -1) {
                MainActivity.this.handy.postDelayed(this, 100);
                return;
            }
            MainActivity.this.rosCameraPreviewView.setCamera(Camera.open(MainActivity.this.cameraId));
        }
    };
    public int status = 0;
    public EditText statusBar;
    public EditText workBar;

    public MainActivity() {
        super("RosAndroidExample", "RosAndroidExample");
    }

    /* access modifiers changed from: protected */
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_nodes);
        Log.d(TAG, "adxdcdecnkedncjksnjkcdn");
        this.locationFrameIdListener = new OnFrameIdChangeListener() {
            public void onFrameIdChanged(String newFrameId) {
                Log.w(MainActivity.TAG, "Default location OnFrameIdChangedListener called");
            }
        };
        this.imuFrameIdListener = new OnFrameIdChangeListener() {
            public void onFrameIdChanged(String newFrameId) {
                Log.w(MainActivity.TAG, "Default IMU OnFrameIdChangedListener called");
            }
        };
        this.locationFrameIdView = (EditText) findViewById(R.id.et_location_frame_id);
        SharedPreferences sharedPreferences = getSharedPreferences("SharedPreferences", 0);
        this.applyB = (Button) findViewById(R.id.b_apply);
        this.resetB = (Button) findViewById(R.id.b_reset);
        this.emergencyb = (Button) findViewById(R.id.b_emergency);
        this.batteryB = (Button) findViewById(R.id.b_battery);
        this.backB = (Button) findViewById(R.id.b_back);
        this.light_offB = (Button) findViewById(R.id.b_light_off);
        this.light_onB = (Button) findViewById(R.id.b_light_on);
        this.continueB = (Button) findViewById(R.id.b_continue);
        this.statusBar = (EditText) findViewById(R.id.status);
        this.workBar = (EditText) findViewById(R.id.work);
        this.autonomousB = (Button) findViewById(R.id.b_autonomous);
        this.backB.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivity.this.finish();
            }
        });
        this.applyB.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Toast.makeText(MainActivity.this.getApplicationContext(), "throttle", 0).show();
                MainActivity.this.startActivity(new Intent(MainActivity.this.getApplicationContext(), Main2Activity.class));
                MainActivity.this.finish();
            }
        });
        this.cameraPreviewView = (CameraPreviewView) findViewById(R.id.camera_preview_view);
        this.rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor) {
        NodeMainExecutor nodeMainExecutor2 = nodeMainExecutor;
        Log.d(TAG, "init()");
        final LocationPublisherNode locationPublisherNode = new LocationPublisherNode();
        ImuPublisherNode imuPublisherNode = new ImuPublisherNode();
        Emergency_Stop em = new Emergency_Stop(this);
        final Battery b = new Battery(this);
        work_completion w = new work_completion(this);
        final status sta = new status(this);
        CameraView c = new CameraView();
        autonomous auto = new autonomous(this);
        this.locationFrameIdListener = locationPublisherNode.getFrameIdListener();
        this.imuFrameIdListener = imuPublisherNode.getFrameIdListener();
        Criteria criteria = new Criteria();
        criteria.setAccuracy(1);
        criteria.setPowerRequirement(1);
        criteria.setAltitudeRequired(false);
        criteria.setBearingRequired(false);
        criteria.setSpeedRequired(false);
        criteria.setCostAllowed(true);
        final LocationManager locationManager = (LocationManager) getSystemService("location");
        runOnUiThread(new Runnable() {
            public void run() {
                if (Build.VERSION.SDK_INT >= 23) {
                    boolean permissionFineLocation = MainActivity.this.checkSelfPermission("android.permission.ACCESS_FINE_LOCATION") == 0;
                    boolean permissionCoarseLocation = MainActivity.this.checkSelfPermission("android.permission.ACCESS_COARSE_LOCATION") == 0;
                    String access$300 = MainActivity.TAG;
                    Log.d(access$300, "PERMISSION 1: " + String.valueOf(permissionFineLocation));
                    String access$3002 = MainActivity.TAG;
                    Log.d(access$3002, "PERMISSION 2: " + String.valueOf(permissionCoarseLocation));
                    if (!permissionFineLocation || !permissionCoarseLocation) {
                        MainActivity.this.requestPermissions(new String[]{"android.permission.ACCESS_FINE_LOCATION", "android.permission.ACCESS_COARSE_LOCATION"}, 4096);
                    } else if (locationManager != null) {
                        Log.d(MainActivity.TAG, "Requesting location");
                        locationManager.requestLocationUpdates("gps", 500, 0.1f, locationPublisherNode.getLocationListener());
                    }
                } else {
                    locationManager.requestLocationUpdates("gps", 500, 0.1f, locationPublisherNode.getLocationListener());
                }
            }
        });
        SensorManager sensorManager = (SensorManager) getSystemService("sensor");
        try {
            LocationPublisherNode locationPublisherNode2 = locationPublisherNode;
            try {
                Criteria criteria2 = criteria;
                try {
                    sensorManager.registerListener(imuPublisherNode.getAccelerometerListener(), sensorManager.getDefaultSensor(1), 0);
                    SensorManager sensorManager1 = (SensorManager) getSystemService("sensor");
                    try {
                        SensorManager sensorManager2 = sensorManager;
                        try {
                            sensorManager1.registerListener(imuPublisherNode.getGyroscopeListener(), sensorManager1.getDefaultSensor(4), 0);
                            SensorManager sensorManager22 = (SensorManager) getSystemService("sensor");
                            try {
                                SensorManager sensorManager3 = sensorManager1;
                                try {
                                    sensorManager22.registerListener(imuPublisherNode.getOrientationListener(), sensorManager22.getDefaultSensor(3), 0);
                                    this.nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
                                    this.nodeConfiguration.setMasterUri(getMasterUri());
                                    this.handy.post(this.sizeCheckRunnable);
                                    nodeMainExecutor2.execute(b, this.nodeConfiguration);
                                    nodeMainExecutor2.execute(sta, this.nodeConfiguration);
                                    nodeMainExecutor2.execute(w, this.nodeConfiguration);
                                    nodeMainExecutor2.execute(c, this.nodeConfiguration);
                                    nodeMainExecutor2.execute(em, this.nodeConfiguration);
                                    nodeMainExecutor2.execute(auto, this.nodeConfiguration);
                                    this.batteryB.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            EditText editText = MainActivity.this.locationFrameIdView;
                                            editText.setText(b.getLevel() + "%");
                                            EditText editText2 = MainActivity.this.statusBar;
                                            editText2.setText(sta.getLevel() + "");
                                        }
                                    });
                                    this.emergencyb.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            Toast.makeText(MainActivity.this.getApplicationContext(), "Emergency Stop", 1).show();
                                            MainActivity.this.status = 1;
                                        }
                                    });
                                    this.autonomousB.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            Toast.makeText(MainActivity.this.getApplicationContext(), "Start Sanitisation", 1).show();
                                            MainActivity.this.autonomous_status = 1;
                                        }
                                    });
                                    this.resetB.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            MainActivity.this.status = 0;
                                            MainActivity.this.startActivity(new Intent(MainActivity.this.getApplicationContext(), MainActivity.class));
                                            MainActivity.this.finish();
                                        }
                                    });
                                    this.light_onB.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            MainActivity.this.status = 0;
                                        }
                                    });
                                    this.light_offB.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            MainActivity.this.status = 2;
                                        }
                                    });
                                    this.continueB.setOnClickListener(new View.OnClickListener() {
                                        public void onClick(View v) {
                                            Toast.makeText(MainActivity.this.getApplicationContext(), "Resume", 1).show();
                                            MainActivity.this.status = 0;
                                        }
                                    });
                                    onClick((View) null);
                                } catch (NullPointerException e) {
                                    e = e;
                                    Log.e(TAG, e.toString());
                                }
                            } catch (NullPointerException e2) {
                                e = e2;
                                SensorManager sensorManager4 = sensorManager1;
                                Log.e(TAG, e.toString());
                            }
                        } catch (NullPointerException e3) {
                            e = e3;
                            SensorManager sensorManager5 = sensorManager1;
                            Log.e(TAG, e.toString());
                        }
                    } catch (NullPointerException e4) {
                        e = e4;
                        SensorManager sensorManager6 = sensorManager1;
                        SensorManager sensorManager7 = sensorManager;
                        Log.e(TAG, e.toString());
                    }
                } catch (NullPointerException e5) {
                    e = e5;
                    SensorManager sensorManager8 = sensorManager;
                    Log.e(TAG, e.toString());
                }
            } catch (NullPointerException e6) {
                e = e6;
                Criteria criteria3 = criteria;
                SensorManager sensorManager9 = sensorManager;
                Log.e(TAG, e.toString());
            }
        } catch (NullPointerException e7) {
            e = e7;
            LocationPublisherNode locationPublisherNode3 = locationPublisherNode;
            Criteria criteria4 = criteria;
            SensorManager sensorManager10 = sensorManager;
            Log.e(TAG, e.toString());
        }
    }

    public boolean onTouchEvent(MotionEvent event) {
        final Toast toast;
        if (event.getAction() == 1) {
            int numberOfCameras = Camera.getNumberOfCameras();
            if (numberOfCameras > 1) {
                this.cameraId = (this.cameraId + 1) % numberOfCameras;
                this.rosCameraPreviewView.releaseCamera();
                this.rosCameraPreviewView.setCamera(Camera.open(this.cameraId));
                toast = Toast.makeText(this, "Switching cameras.", 0);
            } else {
                toast = Toast.makeText(this, "No alternative cameras to switch to.", 0);
            }
            runOnUiThread(new Runnable() {
                public void run() {
                    toast.show();
                }
            });
        }
        return true;
    }

    public void onClick(View view) {
        Log.i(TAG, "Default IMU OnFrameIdChangedListener called");
        SharedPreferences.Editor spe = getSharedPreferences("SharedPreferences", 0).edit();
        String newLocationFrameId = this.locationFrameIdView.getText().toString();
        if (!newLocationFrameId.isEmpty()) {
            this.locationFrameIdListener.onFrameIdChanged(newLocationFrameId);
            spe.putString("locationFrameId", newLocationFrameId);
        }
    }

    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != 4096) {
            return;
        }
        if (grantResults[0] == 0 && grantResults[1] == 0) {
            Log.d(TAG, "Permissions granted!");
        } else {
            Log.e(TAG, "Permissions not granted.");
        }
    }

    public void apply(View v) {
    }
}
