package org.ros.internal.node.response;

import com.google.common.collect.Lists;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import org.ros.exception.RosRuntimeException;

public class UriListResultFactory implements ResultFactory<List<URI>> {
    public List<URI> newFromValue(Object value) {
        List<Object> values = Arrays.asList((Object[]) value);
        List<URI> uris = Lists.newArrayList();
        Iterator<Object> it = values.iterator();
        while (it.hasNext()) {
            try {
                uris.add(new URI((String) it.next()));
            } catch (URISyntaxException e) {
                throw new RosRuntimeException((Throwable) e);
            }
        }
        return uris;
    }
}
