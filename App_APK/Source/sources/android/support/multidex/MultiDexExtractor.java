package android.support.multidex;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Build;
import android.util.Log;
import java.io.BufferedOutputStream;
import java.io.Closeable;
import java.io.File;
import java.io.FileFilter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;
import java.util.zip.ZipOutputStream;

final class MultiDexExtractor {
    private static final int BUFFER_SIZE = 16384;
    private static final String DEX_PREFIX = "classes";
    private static final String DEX_SUFFIX = ".dex";
    private static final String EXTRACTED_NAME_EXT = ".classes";
    private static final String EXTRACTED_SUFFIX = ".zip";
    private static final String KEY_CRC = "crc";
    private static final String KEY_DEX_CRC = "dex.crc.";
    private static final String KEY_DEX_NUMBER = "dex.number";
    private static final String KEY_DEX_TIME = "dex.time.";
    private static final String KEY_TIME_STAMP = "timestamp";
    private static final String LOCK_FILENAME = "MultiDex.lock";
    private static final int MAX_EXTRACT_ATTEMPTS = 3;
    private static final long NO_VALUE = -1;
    private static final String PREFS_FILE = "multidex.version";
    private static final String TAG = "MultiDex";

    MultiDexExtractor() {
    }

    private static class ExtractedDex extends File {
        public long crc = -1;

        public ExtractedDex(File dexDir, String fileName) {
            super(dexDir, fileName);
        }
    }

    /* JADX WARNING: Removed duplicated region for block: B:47:0x0135 A[SYNTHETIC, Splitter:B:47:0x0135] */
    /* JADX WARNING: Removed duplicated region for block: B:52:0x0159  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    static java.util.List<? extends java.io.File> load(android.content.Context r18, java.io.File r19, java.io.File r20, java.lang.String r21, boolean r22) throws java.io.IOException {
        /*
            r8 = r21
            r9 = r22
            java.lang.String r0 = "MultiDex"
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "MultiDexExtractor.load("
            r1.append(r2)
            java.lang.String r2 = r19.getPath()
            r1.append(r2)
            java.lang.String r2 = ", "
            r1.append(r2)
            r1.append(r9)
            java.lang.String r2 = ", "
            r1.append(r2)
            r1.append(r8)
            java.lang.String r2 = ")"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            android.util.Log.i(r0, r1)
            long r10 = getZipCrc(r19)
            java.io.File r0 = new java.io.File
            java.lang.String r1 = "MultiDex.lock"
            r12 = r20
            r0.<init>(r12, r1)
            r13 = r0
            java.io.RandomAccessFile r0 = new java.io.RandomAccessFile
            java.lang.String r1 = "rw"
            r0.<init>(r13, r1)
            r14 = r0
            r1 = 0
            r2 = 0
            r3 = 0
            r15 = r3
            java.nio.channels.FileChannel r0 = r14.getChannel()     // Catch:{ all -> 0x012d }
            r16 = r0
            java.lang.String r0 = "MultiDex"
            java.lang.StringBuilder r1 = new java.lang.StringBuilder     // Catch:{ all -> 0x0128 }
            r1.<init>()     // Catch:{ all -> 0x0128 }
            java.lang.String r4 = "Blocking on lock "
            r1.append(r4)     // Catch:{ all -> 0x0128 }
            java.lang.String r4 = r13.getPath()     // Catch:{ all -> 0x0128 }
            r1.append(r4)     // Catch:{ all -> 0x0128 }
            java.lang.String r1 = r1.toString()     // Catch:{ all -> 0x0128 }
            android.util.Log.i(r0, r1)     // Catch:{ all -> 0x0128 }
            java.nio.channels.FileLock r0 = r16.lock()     // Catch:{ all -> 0x0128 }
            r17 = r0
            java.lang.String r0 = "MultiDex"
            java.lang.StringBuilder r1 = new java.lang.StringBuilder     // Catch:{ all -> 0x0126 }
            r1.<init>()     // Catch:{ all -> 0x0126 }
            java.lang.String r2 = r13.getPath()     // Catch:{ all -> 0x0126 }
            r1.append(r2)     // Catch:{ all -> 0x0126 }
            java.lang.String r2 = " locked"
            r1.append(r2)     // Catch:{ all -> 0x0126 }
            java.lang.String r1 = r1.toString()     // Catch:{ all -> 0x0126 }
            android.util.Log.i(r0, r1)     // Catch:{ all -> 0x0126 }
            if (r9 != 0) goto L_0x00bb
            r5 = r18
            r6 = r19
            boolean r0 = isModified(r5, r6, r10, r8)     // Catch:{ all -> 0x0126 }
            if (r0 != 0) goto L_0x00bb
            java.util.List r0 = loadExistingExtractions(r18, r19, r20, r21)     // Catch:{ IOException -> 0x00a0 }
            r7 = r0
            goto L_0x00d2
        L_0x00a0:
            r0 = move-exception
            r1 = r0
            r0 = r1
            java.lang.String r1 = "MultiDex"
            java.lang.String r2 = "Failed to reload existing extracted secondary dex files, falling back to fresh extraction"
            android.util.Log.w(r1, r2, r0)     // Catch:{ all -> 0x0126 }
            java.util.List r7 = performExtractions(r19, r20)     // Catch:{ all -> 0x0126 }
            long r3 = getTimeStamp(r19)     // Catch:{ all -> 0x0122 }
            r1 = r18
            r2 = r21
            r5 = r10
            putStoredApkInfo(r1, r2, r3, r5, r7)     // Catch:{ all -> 0x0122 }
            goto L_0x00d2
        L_0x00bb:
            java.lang.String r0 = "MultiDex"
            java.lang.String r1 = "Detected that extraction must be performed."
            android.util.Log.i(r0, r1)     // Catch:{ all -> 0x0126 }
            java.util.List r7 = performExtractions(r19, r20)     // Catch:{ all -> 0x0126 }
            long r3 = getTimeStamp(r19)     // Catch:{ all -> 0x0122 }
            r1 = r18
            r2 = r21
            r5 = r10
            putStoredApkInfo(r1, r2, r3, r5, r7)     // Catch:{ all -> 0x0122 }
        L_0x00d2:
            if (r17 == 0) goto L_0x00f6
            r17.release()     // Catch:{ IOException -> 0x00d8 }
            goto L_0x00f6
        L_0x00d8:
            r0 = move-exception
            r1 = r0
            r0 = r1
            java.lang.String r1 = "MultiDex"
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "Failed to release lock on "
            r2.append(r3)
            java.lang.String r3 = r13.getPath()
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            android.util.Log.e(r1, r2)
            r15 = r0
        L_0x00f6:
            if (r16 == 0) goto L_0x00fb
            closeQuietly(r16)
        L_0x00fb:
            closeQuietly(r14)
            if (r15 != 0) goto L_0x0121
            java.lang.String r0 = "MultiDex"
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "load found "
            r1.append(r2)
            int r2 = r7.size()
            r1.append(r2)
            java.lang.String r2 = " secondary dex files"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            android.util.Log.i(r0, r1)
            return r7
        L_0x0121:
            throw r15
        L_0x0122:
            r0 = move-exception
            r1 = r0
            r3 = r7
            goto L_0x0133
        L_0x0126:
            r0 = move-exception
            goto L_0x0132
        L_0x0128:
            r0 = move-exception
            r1 = r0
            r17 = r2
            goto L_0x0133
        L_0x012d:
            r0 = move-exception
            r16 = r1
            r17 = r2
        L_0x0132:
            r1 = r0
        L_0x0133:
            if (r17 == 0) goto L_0x0157
            r17.release()     // Catch:{ IOException -> 0x0139 }
            goto L_0x0157
        L_0x0139:
            r0 = move-exception
            r2 = r0
            r0 = r2
            java.lang.String r2 = "MultiDex"
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "Failed to release lock on "
            r4.append(r5)
            java.lang.String r5 = r13.getPath()
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            android.util.Log.e(r2, r4)
            r15 = r0
        L_0x0157:
            if (r16 == 0) goto L_0x015c
            closeQuietly(r16)
        L_0x015c:
            closeQuietly(r14)
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: android.support.multidex.MultiDexExtractor.load(android.content.Context, java.io.File, java.io.File, java.lang.String, boolean):java.util.List");
    }

    private static List<ExtractedDex> loadExistingExtractions(Context context, File sourceApk, File dexDir, String prefsKeyPrefix) throws IOException {
        String str = prefsKeyPrefix;
        Log.i(TAG, "loading existing secondary dex files");
        String extractedFilePrefix = sourceApk.getName() + EXTRACTED_NAME_EXT;
        SharedPreferences multiDexPreferences = getMultiDexPreferences(context);
        int totalDexNumber = multiDexPreferences.getInt(str + KEY_DEX_NUMBER, 1);
        List<ExtractedDex> files = new ArrayList<>(totalDexNumber + -1);
        int secondaryNumber = 2;
        while (secondaryNumber <= totalDexNumber) {
            String fileName = extractedFilePrefix + secondaryNumber + EXTRACTED_SUFFIX;
            ExtractedDex extractedFile = new ExtractedDex(dexDir, fileName);
            if (extractedFile.isFile()) {
                extractedFile.crc = getZipCrc(extractedFile);
                long expectedCrc = multiDexPreferences.getLong(str + KEY_DEX_CRC + secondaryNumber, -1);
                long expectedModTime = multiDexPreferences.getLong(str + KEY_DEX_TIME + secondaryNumber, -1);
                long lastModified = extractedFile.lastModified();
                if (expectedModTime == lastModified) {
                    String extractedFilePrefix2 = extractedFilePrefix;
                    SharedPreferences multiDexPreferences2 = multiDexPreferences;
                    if (expectedCrc == extractedFile.crc) {
                        files.add(extractedFile);
                        secondaryNumber++;
                        extractedFilePrefix = extractedFilePrefix2;
                        multiDexPreferences = multiDexPreferences2;
                    }
                } else {
                    SharedPreferences sharedPreferences = multiDexPreferences;
                }
                StringBuilder sb = new StringBuilder();
                sb.append("Invalid extracted dex: ");
                sb.append(extractedFile);
                sb.append(" (key \"");
                sb.append(str);
                sb.append("\"), expected modification time: ");
                sb.append(expectedModTime);
                sb.append(", modification time: ");
                sb.append(lastModified);
                sb.append(", expected crc: ");
                sb.append(expectedCrc);
                sb.append(", file crc: ");
                int i = secondaryNumber;
                String str2 = fileName;
                sb.append(extractedFile.crc);
                throw new IOException(sb.toString());
            }
            SharedPreferences sharedPreferences2 = multiDexPreferences;
            int i2 = secondaryNumber;
            String str3 = fileName;
            throw new IOException("Missing extracted secondary dex file '" + extractedFile.getPath() + "'");
        }
        File file = dexDir;
        String str4 = extractedFilePrefix;
        SharedPreferences sharedPreferences3 = multiDexPreferences;
        return files;
    }

    private static boolean isModified(Context context, File archive, long currentCrc, String prefsKeyPrefix) {
        SharedPreferences prefs = getMultiDexPreferences(context);
        if (prefs.getLong(prefsKeyPrefix + KEY_TIME_STAMP, -1) == getTimeStamp(archive)) {
            StringBuilder sb = new StringBuilder();
            sb.append(prefsKeyPrefix);
            sb.append(KEY_CRC);
            return prefs.getLong(sb.toString(), -1) != currentCrc;
        }
    }

    private static long getTimeStamp(File archive) {
        long timeStamp = archive.lastModified();
        if (timeStamp == -1) {
            return timeStamp - 1;
        }
        return timeStamp;
    }

    private static long getZipCrc(File archive) throws IOException {
        long computedValue = ZipUtil.getZipCrc(archive);
        if (computedValue == -1) {
            return computedValue - 1;
        }
        return computedValue;
    }

    private static List<ExtractedDex> performExtractions(File sourceApk, File dexDir) throws IOException {
        ExtractedDex extractedFile;
        boolean isExtractionSuccessful;
        String extractedFilePrefix = sourceApk.getName() + EXTRACTED_NAME_EXT;
        prepareDexDir(dexDir, extractedFilePrefix);
        List<ExtractedDex> files = new ArrayList<>();
        ZipFile apk = new ZipFile(sourceApk);
        int secondaryNumber = 2;
        try {
            ZipEntry dexFile = apk.getEntry(DEX_PREFIX + 2 + DEX_SUFFIX);
            while (dexFile != null) {
                extractedFile = new ExtractedDex(dexDir, extractedFilePrefix + secondaryNumber + EXTRACTED_SUFFIX);
                files.add(extractedFile);
                Log.i(TAG, "Extraction is needed for file " + extractedFile);
                int numAttempts = 0;
                isExtractionSuccessful = false;
                while (numAttempts < 3 && !isExtractionSuccessful) {
                    numAttempts++;
                    extract(apk, dexFile, extractedFile, extractedFilePrefix);
                    extractedFile.crc = getZipCrc(extractedFile);
                    isExtractionSuccessful = true;
                    StringBuilder sb = new StringBuilder();
                    sb.append("Extraction ");
                    sb.append(isExtractionSuccessful ? "succeeded" : "failed");
                    sb.append(" - length ");
                    sb.append(extractedFile.getAbsolutePath());
                    sb.append(": ");
                    sb.append(extractedFile.length());
                    sb.append(" - crc: ");
                    sb.append(extractedFile.crc);
                    Log.i(TAG, sb.toString());
                    if (!isExtractionSuccessful) {
                        extractedFile.delete();
                        if (extractedFile.exists()) {
                            Log.w(TAG, "Failed to delete corrupted secondary dex '" + extractedFile.getPath() + "'");
                        }
                    }
                }
                if (isExtractionSuccessful) {
                    secondaryNumber++;
                    dexFile = apk.getEntry(DEX_PREFIX + secondaryNumber + DEX_SUFFIX);
                } else {
                    throw new IOException("Could not create zip file " + extractedFile.getAbsolutePath() + " for secondary dex (" + secondaryNumber + ")");
                }
            }
            try {
                apk.close();
            } catch (IOException e) {
                Log.w(TAG, "Failed to close resource", e);
            }
            return files;
        } catch (IOException e2) {
            isExtractionSuccessful = false;
            Log.w(TAG, "Failed to read crc from " + extractedFile.getAbsolutePath(), e2);
        } catch (Throwable th) {
            try {
                apk.close();
            } catch (IOException e3) {
                Log.w(TAG, "Failed to close resource", e3);
            }
            throw th;
        }
    }

    private static void putStoredApkInfo(Context context, String keyPrefix, long timeStamp, long crc, List<ExtractedDex> extractedDexes) {
        SharedPreferences.Editor edit = getMultiDexPreferences(context).edit();
        edit.putLong(keyPrefix + KEY_TIME_STAMP, timeStamp);
        edit.putLong(keyPrefix + KEY_CRC, crc);
        edit.putInt(keyPrefix + KEY_DEX_NUMBER, extractedDexes.size() + 1);
        int extractedDexId = 2;
        for (ExtractedDex dex : extractedDexes) {
            edit.putLong(keyPrefix + KEY_DEX_CRC + extractedDexId, dex.crc);
            edit.putLong(keyPrefix + KEY_DEX_TIME + extractedDexId, dex.lastModified());
            extractedDexId++;
        }
        edit.commit();
    }

    private static SharedPreferences getMultiDexPreferences(Context context) {
        return context.getSharedPreferences(PREFS_FILE, Build.VERSION.SDK_INT < 11 ? 0 : 4);
    }

    private static void prepareDexDir(File dexDir, final String extractedFilePrefix) {
        File[] files = dexDir.listFiles(new FileFilter() {
            public boolean accept(File pathname) {
                String name = pathname.getName();
                return !name.startsWith(extractedFilePrefix) && !name.equals(MultiDexExtractor.LOCK_FILENAME);
            }
        });
        if (files == null) {
            Log.w(TAG, "Failed to list secondary dex dir content (" + dexDir.getPath() + ").");
            return;
        }
        for (File oldFile : files) {
            Log.i(TAG, "Trying to delete old file " + oldFile.getPath() + " of size " + oldFile.length());
            if (!oldFile.delete()) {
                Log.w(TAG, "Failed to delete old file " + oldFile.getPath());
            } else {
                Log.i(TAG, "Deleted old file " + oldFile.getPath());
            }
        }
    }

    private static void extract(ZipFile apk, ZipEntry dexFile, File extractTo, String extractedFilePrefix) throws IOException, FileNotFoundException {
        ZipOutputStream out;
        InputStream in = apk.getInputStream(dexFile);
        File tmp = File.createTempFile("tmp-" + extractedFilePrefix, EXTRACTED_SUFFIX, extractTo.getParentFile());
        Log.i(TAG, "Extracting " + tmp.getPath());
        try {
            out = new ZipOutputStream(new BufferedOutputStream(new FileOutputStream(tmp)));
            ZipEntry classesDex = new ZipEntry("classes.dex");
            classesDex.setTime(dexFile.getTime());
            out.putNextEntry(classesDex);
            byte[] buffer = new byte[16384];
            for (int length = in.read(buffer); length != -1; length = in.read(buffer)) {
                out.write(buffer, 0, length);
            }
            out.closeEntry();
            out.close();
            if (tmp.setReadOnly()) {
                Log.i(TAG, "Renaming to " + extractTo.getPath());
                if (tmp.renameTo(extractTo)) {
                    closeQuietly(in);
                    tmp.delete();
                    return;
                }
                throw new IOException("Failed to rename \"" + tmp.getAbsolutePath() + "\" to \"" + extractTo.getAbsolutePath() + "\"");
            }
            throw new IOException("Failed to mark readonly \"" + tmp.getAbsolutePath() + "\" (tmp of \"" + extractTo.getAbsolutePath() + "\")");
        } catch (Throwable th) {
            closeQuietly(in);
            tmp.delete();
            throw th;
        }
    }

    private static void closeQuietly(Closeable closeable) {
        try {
            closeable.close();
        } catch (IOException e) {
            Log.w(TAG, "Failed to close resource", e);
        }
    }
}
