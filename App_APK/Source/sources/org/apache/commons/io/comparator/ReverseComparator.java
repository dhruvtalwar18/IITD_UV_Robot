package org.apache.commons.io.comparator;

import java.io.Serializable;
import java.util.Comparator;

class ReverseComparator implements Comparator, Serializable {
    private final Comparator delegate;

    public ReverseComparator(Comparator delegate2) {
        if (delegate2 != null) {
            this.delegate = delegate2;
            return;
        }
        throw new IllegalArgumentException("Delegate comparator is missing");
    }

    public int compare(Object obj1, Object obj2) {
        return this.delegate.compare(obj2, obj1);
    }
}
