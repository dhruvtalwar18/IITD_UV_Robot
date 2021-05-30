package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.Serializable;
import java.util.Date;
import org.apache.commons.io.FileUtils;

public class AgeFileFilter extends AbstractFileFilter implements Serializable {
    private final boolean acceptOlder;
    private final long cutoff;

    public AgeFileFilter(long cutoff2) {
        this(cutoff2, true);
    }

    public AgeFileFilter(long cutoff2, boolean acceptOlder2) {
        this.acceptOlder = acceptOlder2;
        this.cutoff = cutoff2;
    }

    public AgeFileFilter(Date cutoffDate) {
        this(cutoffDate, true);
    }

    public AgeFileFilter(Date cutoffDate, boolean acceptOlder2) {
        this(cutoffDate.getTime(), acceptOlder2);
    }

    public AgeFileFilter(File cutoffReference) {
        this(cutoffReference, true);
    }

    public AgeFileFilter(File cutoffReference, boolean acceptOlder2) {
        this(cutoffReference.lastModified(), acceptOlder2);
    }

    public boolean accept(File file) {
        boolean newer = FileUtils.isFileNewer(file, this.cutoff);
        if (this.acceptOlder) {
            return !newer;
        }
        return newer;
    }

    public String toString() {
        String condition = this.acceptOlder ? "<=" : ">";
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(super.toString());
        stringBuffer.append("(");
        stringBuffer.append(condition);
        stringBuffer.append(this.cutoff);
        stringBuffer.append(")");
        return stringBuffer.toString();
    }
}
