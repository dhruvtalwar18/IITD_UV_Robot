package org.ros.master.uri;

import com.google.common.collect.Lists;
import java.net.URI;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.ros.exception.RosRuntimeException;

public class SwitchableMasterUriProvider implements MasterUriProvider {
    private final Object mutex;
    private List<ProviderRequest> pending = Lists.newArrayList();
    private MasterUriProvider provider;

    public interface MasterUriProviderSwitcher {
        MasterUriProvider switchProvider(MasterUriProvider masterUriProvider);
    }

    public SwitchableMasterUriProvider(MasterUriProvider provider2) {
        this.provider = provider2;
        this.mutex = new Object();
    }

    public URI getMasterUri() throws RosRuntimeException {
        MasterUriProvider providerToUse = null;
        ProviderRequest requestToUse = null;
        synchronized (this.mutex) {
            if (this.provider != null) {
                providerToUse = this.provider;
            } else {
                requestToUse = new ProviderRequest();
                this.pending.add(requestToUse);
            }
        }
        if (providerToUse != null) {
            return providerToUse.getMasterUri();
        }
        return requestToUse.getMasterUri();
    }

    public URI getMasterUri(long timeout, TimeUnit unit) {
        MasterUriProvider providerToUse = null;
        synchronized (this.mutex) {
            if (this.provider != null) {
                providerToUse = this.provider;
            }
        }
        if (providerToUse != null) {
            return providerToUse.getMasterUri(timeout, unit);
        }
        try {
            Thread.sleep(unit.toMillis(timeout));
            return null;
        } catch (InterruptedException e) {
            return null;
        }
    }

    public void switchProvider(MasterUriProviderSwitcher switcher) {
        synchronized (this.mutex) {
            MasterUriProvider oldProvider = this.provider;
            this.provider = switcher.switchProvider(oldProvider);
            if (oldProvider == null) {
                for (ProviderRequest request : this.pending) {
                    request.setProvider(this.provider);
                }
                this.pending.clear();
            }
        }
    }

    private static class ProviderRequest {
        private CountDownLatch latch;
        private MasterUriProvider provider;

        private ProviderRequest() {
            this.latch = new CountDownLatch(1);
        }

        public URI getMasterUri() {
            try {
                this.latch.await();
                return this.provider.getMasterUri();
            } catch (InterruptedException e) {
                throw new RosRuntimeException("URI provider interrupted", e);
            }
        }

        public void setProvider(MasterUriProvider provider2) {
            this.provider = provider2;
            this.latch.countDown();
        }
    }
}
