package org.ros.internal.node.response;

import com.google.common.collect.Lists;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

public class StringListResultFactory implements ResultFactory<List<String>> {
    public List<String> newFromValue(Object value) {
        List<String> strings = Lists.newArrayList();
        Iterator<Object> it = Arrays.asList((Object[]) value).iterator();
        while (it.hasNext()) {
            strings.add((String) it.next());
        }
        return strings;
    }
}
