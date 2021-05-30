package org.opencv.android;

import android.util.Log;
import java.util.StringTokenizer;

class StaticHelper {
    private static final String TAG = "OpenCV/StaticHelper";

    private static native String getLibraryList();

    StaticHelper() {
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v4, resolved type: int} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v5, resolved type: int} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v5, resolved type: boolean} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v6, resolved type: int} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v7, resolved type: int} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static boolean initOpenCV(boolean r7) {
        /*
            java.lang.String r0 = ""
            if (r7 == 0) goto L_0x0022
            java.lang.String r1 = "cudart"
            loadLibrary(r1)
            java.lang.String r1 = "nppc"
            loadLibrary(r1)
            java.lang.String r1 = "nppi"
            loadLibrary(r1)
            java.lang.String r1 = "npps"
            loadLibrary(r1)
            java.lang.String r1 = "cufft"
            loadLibrary(r1)
            java.lang.String r1 = "cublas"
            loadLibrary(r1)
        L_0x0022:
            java.lang.String r1 = "OpenCV/StaticHelper"
            java.lang.String r2 = "Trying to get library list"
            android.util.Log.d(r1, r2)
            java.lang.String r1 = "opencv_info"
            java.lang.System.loadLibrary(r1)     // Catch:{ UnsatisfiedLinkError -> 0x0034 }
            java.lang.String r1 = getLibraryList()     // Catch:{ UnsatisfiedLinkError -> 0x0034 }
            r0 = r1
            goto L_0x003c
        L_0x0034:
            r1 = move-exception
            java.lang.String r2 = "OpenCV/StaticHelper"
            java.lang.String r3 = "OpenCV error: Cannot load info library for OpenCV"
            android.util.Log.e(r2, r3)
        L_0x003c:
            java.lang.String r1 = "OpenCV/StaticHelper"
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "Library list: \""
            r2.append(r3)
            r2.append(r0)
            java.lang.String r3 = "\""
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            android.util.Log.d(r1, r2)
            java.lang.String r1 = "OpenCV/StaticHelper"
            java.lang.String r2 = "First attempt to load libs"
            android.util.Log.d(r1, r2)
            boolean r1 = initOpenCVLibs(r0)
            r2 = 0
            if (r1 == 0) goto L_0x0089
            java.lang.String r1 = "OpenCV/StaticHelper"
            java.lang.String r3 = "First attempt to load libs is OK"
            android.util.Log.d(r1, r3)
            java.lang.String r1 = "line.separator"
            java.lang.String r1 = java.lang.System.getProperty(r1)
            java.lang.String r3 = org.opencv.core.Core.getBuildInformation()
            java.lang.String[] r3 = r3.split(r1)
            int r4 = r3.length
        L_0x007b:
            if (r2 >= r4) goto L_0x0087
            r5 = r3[r2]
            java.lang.String r6 = "OpenCV/StaticHelper"
            android.util.Log.i(r6, r5)
            int r2 = r2 + 1
            goto L_0x007b
        L_0x0087:
            r2 = 1
            goto L_0x0091
        L_0x0089:
            java.lang.String r1 = "OpenCV/StaticHelper"
            java.lang.String r3 = "First attempt to load libs fails"
            android.util.Log.d(r1, r3)
        L_0x0091:
            r1 = r2
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.opencv.android.StaticHelper.initOpenCV(boolean):boolean");
    }

    private static boolean loadLibrary(String Name) {
        Log.d(TAG, "Trying to load library " + Name);
        try {
            System.loadLibrary(Name);
            Log.d(TAG, "Library " + Name + " loaded");
            return true;
        } catch (UnsatisfiedLinkError e) {
            Log.d(TAG, "Cannot load library \"" + Name + "\"");
            e.printStackTrace();
            return false;
        }
    }

    private static boolean initOpenCVLibs(String Libs) {
        Log.d(TAG, "Trying to init OpenCV libs");
        boolean result = true;
        if (Libs == null || Libs.length() == 0) {
            return loadLibrary("opencv_java4");
        }
        Log.d(TAG, "Trying to load libs by dependency list");
        StringTokenizer splitter = new StringTokenizer(Libs, ";");
        while (splitter.hasMoreTokens()) {
            result &= loadLibrary(splitter.nextToken());
        }
        return result;
    }
}
