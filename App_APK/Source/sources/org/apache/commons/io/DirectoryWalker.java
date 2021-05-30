package org.apache.commons.io;

import java.io.File;
import java.io.FileFilter;
import java.io.IOException;
import java.util.Collection;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.apache.commons.io.filefilter.IOFileFilter;
import org.apache.commons.io.filefilter.TrueFileFilter;

public abstract class DirectoryWalker {
    private final int depthLimit;
    private final FileFilter filter;

    protected DirectoryWalker() {
        this((FileFilter) null, -1);
    }

    protected DirectoryWalker(FileFilter filter2, int depthLimit2) {
        this.filter = filter2;
        this.depthLimit = depthLimit2;
    }

    protected DirectoryWalker(IOFileFilter directoryFilter, IOFileFilter fileFilter, int depthLimit2) {
        if (directoryFilter == null && fileFilter == null) {
            this.filter = null;
        } else {
            this.filter = FileFilterUtils.orFileFilter(FileFilterUtils.makeDirectoryOnly(directoryFilter != null ? directoryFilter : TrueFileFilter.TRUE), FileFilterUtils.makeFileOnly(fileFilter != null ? fileFilter : TrueFileFilter.TRUE));
        }
        this.depthLimit = depthLimit2;
    }

    /* access modifiers changed from: protected */
    public final void walk(File startDirectory, Collection results) throws IOException {
        if (startDirectory != null) {
            try {
                handleStart(startDirectory, results);
                walk(startDirectory, 0, results);
                handleEnd(results);
            } catch (CancelException cancel) {
                handleCancelled(startDirectory, results, cancel);
            }
        } else {
            throw new NullPointerException("Start Directory is null");
        }
    }

    private void walk(File directory, int depth, Collection results) throws IOException {
        checkIfCancelled(directory, depth, results);
        if (handleDirectory(directory, depth, results)) {
            handleDirectoryStart(directory, depth, results);
            int childDepth = depth + 1;
            if (this.depthLimit < 0 || childDepth <= this.depthLimit) {
                checkIfCancelled(directory, depth, results);
                File[] childFiles = this.filter == null ? directory.listFiles() : directory.listFiles(this.filter);
                if (childFiles == null) {
                    handleRestricted(directory, childDepth, results);
                } else {
                    for (File childFile : childFiles) {
                        if (childFile.isDirectory()) {
                            walk(childFile, childDepth, results);
                        } else {
                            checkIfCancelled(childFile, childDepth, results);
                            handleFile(childFile, childDepth, results);
                            checkIfCancelled(childFile, childDepth, results);
                        }
                    }
                }
            }
            handleDirectoryEnd(directory, depth, results);
        }
        checkIfCancelled(directory, depth, results);
    }

    /* access modifiers changed from: protected */
    public final void checkIfCancelled(File file, int depth, Collection results) throws IOException {
        if (handleIsCancelled(file, depth, results)) {
            throw new CancelException(file, depth);
        }
    }

    /* access modifiers changed from: protected */
    public boolean handleIsCancelled(File file, int depth, Collection results) throws IOException {
        return false;
    }

    /* access modifiers changed from: protected */
    public void handleCancelled(File startDirectory, Collection results, CancelException cancel) throws IOException {
        throw cancel;
    }

    /* access modifiers changed from: protected */
    public void handleStart(File startDirectory, Collection results) throws IOException {
    }

    /* access modifiers changed from: protected */
    public boolean handleDirectory(File directory, int depth, Collection results) throws IOException {
        return true;
    }

    /* access modifiers changed from: protected */
    public void handleDirectoryStart(File directory, int depth, Collection results) throws IOException {
    }

    /* access modifiers changed from: protected */
    public void handleFile(File file, int depth, Collection results) throws IOException {
    }

    /* access modifiers changed from: protected */
    public void handleRestricted(File directory, int depth, Collection results) throws IOException {
    }

    /* access modifiers changed from: protected */
    public void handleDirectoryEnd(File directory, int depth, Collection results) throws IOException {
    }

    /* access modifiers changed from: protected */
    public void handleEnd(Collection results) throws IOException {
    }

    public static class CancelException extends IOException {
        private static final long serialVersionUID = 1347339620135041008L;
        private int depth;
        private File file;

        public CancelException(File file2, int depth2) {
            this("Operation Cancelled", file2, depth2);
        }

        public CancelException(String message, File file2, int depth2) {
            super(message);
            this.depth = -1;
            this.file = file2;
            this.depth = depth2;
        }

        public File getFile() {
            return this.file;
        }

        public int getDepth() {
            return this.depth;
        }
    }
}
