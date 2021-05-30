package org.apache.commons.io;

import java.io.File;
import java.lang.ref.PhantomReference;
import java.lang.ref.ReferenceQueue;
import java.util.Collection;
import java.util.Vector;

public class FileCleaningTracker {
    volatile boolean exitWhenFinished = false;
    ReferenceQueue q = new ReferenceQueue();
    Thread reaper;
    final Collection trackers = new Vector();

    public void track(File file, Object marker) {
        track(file, marker, (FileDeleteStrategy) null);
    }

    public void track(File file, Object marker, FileDeleteStrategy deleteStrategy) {
        if (file != null) {
            addTracker(file.getPath(), marker, deleteStrategy);
            return;
        }
        throw new NullPointerException("The file must not be null");
    }

    public void track(String path, Object marker) {
        track(path, marker, (FileDeleteStrategy) null);
    }

    public void track(String path, Object marker, FileDeleteStrategy deleteStrategy) {
        if (path != null) {
            addTracker(path, marker, deleteStrategy);
            return;
        }
        throw new NullPointerException("The path must not be null");
    }

    private synchronized void addTracker(String path, Object marker, FileDeleteStrategy deleteStrategy) {
        if (!this.exitWhenFinished) {
            if (this.reaper == null) {
                this.reaper = new Reaper(this);
                this.reaper.start();
            }
            this.trackers.add(new Tracker(path, deleteStrategy, marker, this.q));
        } else {
            throw new IllegalStateException("No new trackers can be added once exitWhenFinished() is called");
        }
    }

    public int getTrackCount() {
        return this.trackers.size();
    }

    public synchronized void exitWhenFinished() {
        this.exitWhenFinished = true;
        if (this.reaper != null) {
            synchronized (this.reaper) {
                this.reaper.interrupt();
            }
        }
    }

    private final class Reaper extends Thread {
        private final /* synthetic */ FileCleaningTracker this$0;

        Reaper(FileCleaningTracker fileCleaningTracker) {
            super("File Reaper");
            this.this$0 = fileCleaningTracker;
            setPriority(10);
            setDaemon(true);
        }

        public void run() {
            while (true) {
                if (!this.this$0.exitWhenFinished || this.this$0.trackers.size() > 0) {
                    try {
                        Tracker tracker = (Tracker) this.this$0.q.remove();
                        if (tracker != null) {
                            tracker.delete();
                            tracker.clear();
                            this.this$0.trackers.remove(tracker);
                        }
                    } catch (Exception e) {
                    }
                } else {
                    return;
                }
            }
        }
    }

    private static final class Tracker extends PhantomReference {
        private final FileDeleteStrategy deleteStrategy;
        private final String path;

        Tracker(String path2, FileDeleteStrategy deleteStrategy2, Object marker, ReferenceQueue queue) {
            super(marker, queue);
            this.path = path2;
            this.deleteStrategy = deleteStrategy2 == null ? FileDeleteStrategy.NORMAL : deleteStrategy2;
        }

        public boolean delete() {
            return this.deleteStrategy.deleteQuietly(new File(this.path));
        }
    }
}
