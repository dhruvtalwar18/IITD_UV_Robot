package org.ros.master.uri;

import java.net.URI;
import java.util.concurrent.TimeUnit;

public class StaticMasterUriProvider implements MasterUriProvider {
    private final URI uri;

    public StaticMasterUriProvider(URI uri2) {
        this.uri = uri2;
    }

    public URI getMasterUri() {
        return this.uri;
    }

    public URI getMasterUri(long timeout, TimeUnit unit) {
        return this.uri;
    }
}
