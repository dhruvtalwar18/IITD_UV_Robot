package org.apache.commons.io.comparator;

import java.io.File;
import java.io.Serializable;
import java.util.Comparator;

public class LastModifiedFileComparator implements Comparator, Serializable {
    public static final Comparator LASTMODIFIED_COMPARATOR = new LastModifiedFileComparator();
    public static final Comparator LASTMODIFIED_REVERSE = new ReverseComparator(LASTMODIFIED_COMPARATOR);

    public int compare(Object obj1, Object obj2) {
        long result = ((File) obj1).lastModified() - ((File) obj2).lastModified();
        if (result < 0) {
            return -1;
        }
        if (result > 0) {
            return 1;
        }
        return 0;
    }
}
