package org.ros.internal.message;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import java.io.File;
import java.io.FileFilter;
import java.io.IOException;
import java.util.Collection;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;
import org.apache.commons.io.DirectoryWalker;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.apache.commons.io.filefilter.IOFileFilter;
import org.ros.exception.RosMessageRuntimeException;

public class StringFileProvider {
    private final Collection<File> directories = Lists.newArrayList();
    private final StringFileDirectoryWalker stringFileDirectoryWalker;
    /* access modifiers changed from: private */
    public final Map<File, String> strings = Maps.newConcurrentMap();

    private final class StringFileDirectoryWalker extends DirectoryWalker {
        private final Set<File> directories;

        private StringFileDirectoryWalker(FileFilter filter, int depthLimit) {
            super(filter, depthLimit);
            this.directories = Sets.newHashSet();
        }

        /* access modifiers changed from: protected */
        public boolean handleDirectory(File directory, int depth, Collection results) throws IOException {
            File canonicalDirectory = directory.getCanonicalFile();
            if (this.directories.contains(canonicalDirectory)) {
                return false;
            }
            this.directories.add(canonicalDirectory);
            return true;
        }

        /* access modifiers changed from: protected */
        public void handleFile(File file, int depth, Collection results) {
            try {
                StringFileProvider.this.strings.put(file, FileUtils.readFileToString(file, "US-ASCII"));
            } catch (IOException e) {
                throw new RosMessageRuntimeException((Throwable) e);
            }
        }

        public void update(File directory) {
            try {
                walk(directory, (Collection) null);
            } catch (IOException e) {
                throw new RosMessageRuntimeException((Throwable) e);
            }
        }
    }

    public StringFileProvider(IOFileFilter ioFileFilter) {
        this.stringFileDirectoryWalker = new StringFileDirectoryWalker(FileFilterUtils.orFileFilter(FileFilterUtils.directoryFileFilter(), ioFileFilter), -1);
    }

    public void update() {
        for (File directory : this.directories) {
            this.stringFileDirectoryWalker.update(directory);
        }
    }

    public void addDirectory(File directory) {
        Preconditions.checkArgument(directory.isDirectory());
        this.directories.add(directory);
    }

    public Map<File, String> getStrings() {
        return ImmutableMap.copyOf(this.strings);
    }

    public String get(File file) {
        if (has(file)) {
            return this.strings.get(file);
        }
        throw new NoSuchElementException("File does not exist: " + file);
    }

    public boolean has(File file) {
        return this.strings.containsKey(file);
    }
}
