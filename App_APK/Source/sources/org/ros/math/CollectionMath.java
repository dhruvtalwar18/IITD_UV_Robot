package org.ros.math;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class CollectionMath {
    private CollectionMath() {
    }

    public static <T extends Comparable<? super T>> T median(Collection<T> collection) {
        Preconditions.checkArgument(collection.size() > 0);
        List<T> list = Lists.newArrayList(collection);
        Collections.sort(list);
        return (Comparable) list.get(list.size() / 2);
    }
}
