package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.FileFilter;
import java.io.FilenameFilter;
import java.util.Date;

public class FileFilterUtils {
    private static IOFileFilter cvsFilter;
    private static IOFileFilter svnFilter;

    public static IOFileFilter prefixFileFilter(String prefix) {
        return new PrefixFileFilter(prefix);
    }

    public static IOFileFilter suffixFileFilter(String suffix) {
        return new SuffixFileFilter(suffix);
    }

    public static IOFileFilter nameFileFilter(String name) {
        return new NameFileFilter(name);
    }

    public static IOFileFilter directoryFileFilter() {
        return DirectoryFileFilter.DIRECTORY;
    }

    public static IOFileFilter fileFileFilter() {
        return FileFileFilter.FILE;
    }

    public static IOFileFilter andFileFilter(IOFileFilter filter1, IOFileFilter filter2) {
        return new AndFileFilter(filter1, filter2);
    }

    public static IOFileFilter orFileFilter(IOFileFilter filter1, IOFileFilter filter2) {
        return new OrFileFilter(filter1, filter2);
    }

    public static IOFileFilter notFileFilter(IOFileFilter filter) {
        return new NotFileFilter(filter);
    }

    public static IOFileFilter trueFileFilter() {
        return TrueFileFilter.TRUE;
    }

    public static IOFileFilter falseFileFilter() {
        return FalseFileFilter.FALSE;
    }

    public static IOFileFilter asFileFilter(FileFilter filter) {
        return new DelegateFileFilter(filter);
    }

    public static IOFileFilter asFileFilter(FilenameFilter filter) {
        return new DelegateFileFilter(filter);
    }

    public static IOFileFilter ageFileFilter(long cutoff) {
        return new AgeFileFilter(cutoff);
    }

    public static IOFileFilter ageFileFilter(long cutoff, boolean acceptOlder) {
        return new AgeFileFilter(cutoff, acceptOlder);
    }

    public static IOFileFilter ageFileFilter(Date cutoffDate) {
        return new AgeFileFilter(cutoffDate);
    }

    public static IOFileFilter ageFileFilter(Date cutoffDate, boolean acceptOlder) {
        return new AgeFileFilter(cutoffDate, acceptOlder);
    }

    public static IOFileFilter ageFileFilter(File cutoffReference) {
        return new AgeFileFilter(cutoffReference);
    }

    public static IOFileFilter ageFileFilter(File cutoffReference, boolean acceptOlder) {
        return new AgeFileFilter(cutoffReference, acceptOlder);
    }

    public static IOFileFilter sizeFileFilter(long threshold) {
        return new SizeFileFilter(threshold);
    }

    public static IOFileFilter sizeFileFilter(long threshold, boolean acceptLarger) {
        return new SizeFileFilter(threshold, acceptLarger);
    }

    public static IOFileFilter sizeRangeFileFilter(long minSizeInclusive, long maxSizeInclusive) {
        return new AndFileFilter(new SizeFileFilter(minSizeInclusive, true), new SizeFileFilter(1 + maxSizeInclusive, false));
    }

    public static IOFileFilter makeCVSAware(IOFileFilter filter) {
        if (cvsFilter == null) {
            cvsFilter = notFileFilter(andFileFilter(directoryFileFilter(), nameFileFilter("CVS")));
        }
        if (filter == null) {
            return cvsFilter;
        }
        return andFileFilter(filter, cvsFilter);
    }

    public static IOFileFilter makeSVNAware(IOFileFilter filter) {
        if (svnFilter == null) {
            svnFilter = notFileFilter(andFileFilter(directoryFileFilter(), nameFileFilter(".svn")));
        }
        if (filter == null) {
            return svnFilter;
        }
        return andFileFilter(filter, svnFilter);
    }

    public static IOFileFilter makeDirectoryOnly(IOFileFilter filter) {
        if (filter == null) {
            return DirectoryFileFilter.DIRECTORY;
        }
        return new AndFileFilter(DirectoryFileFilter.DIRECTORY, filter);
    }

    public static IOFileFilter makeFileOnly(IOFileFilter filter) {
        if (filter == null) {
            return FileFileFilter.FILE;
        }
        return new AndFileFilter(FileFileFilter.FILE, filter);
    }
}
