package org.ros.internal.message;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Maps;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.Charset;
import java.util.Map;
import java.util.NoSuchElementException;
import org.ros.exception.RosMessageRuntimeException;

public class StringResourceProvider {
    private final Map<String, String> cache = Maps.newConcurrentMap();

    public String get(String resourceName) {
        if (has(resourceName)) {
            if (!this.cache.containsKey(resourceName)) {
                InputStream in = getClass().getResourceAsStream(resourceName);
                StringBuilder out = new StringBuilder();
                Charset charset = Charset.forName("US-ASCII");
                byte[] buffer = new byte[8192];
                while (true) {
                    try {
                        int read = in.read(buffer);
                        int bytesRead = read;
                        if (read == -1) {
                            break;
                        }
                        out.append(new String(buffer, 0, bytesRead, charset));
                    } catch (IOException e) {
                        throw new RosMessageRuntimeException("Failed to read resource: " + resourceName, e);
                    }
                }
                this.cache.put(resourceName, out.toString());
            }
            return this.cache.get(resourceName);
        }
        throw new NoSuchElementException("Resource does not exist: " + resourceName);
    }

    public boolean has(String resourceName) {
        return this.cache.containsKey(resourceName) || getClass().getResource(resourceName) != null;
    }

    public Map<String, String> getCachedStrings() {
        return ImmutableMap.copyOf(this.cache);
    }

    public void addStringToCache(String resourceName, String resourceContent) {
        this.cache.put(resourceName, resourceContent);
    }
}
