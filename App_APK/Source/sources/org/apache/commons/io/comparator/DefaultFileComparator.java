package org.apache.commons.io.comparator;

import java.io.File;
import java.io.Serializable;
import java.util.Comparator;

public class DefaultFileComparator implements Comparator, Serializable {
    public static final Comparator DEFAULT_COMPARATOR = new DefaultFileComparator();
    public static final Comparator DEFAULT_REVERSE = new ReverseComparator(DEFAULT_COMPARATOR);

    public int compare(Object obj1, Object obj2) {
        return ((File) obj1).compareTo((File) obj2);
    }
}
