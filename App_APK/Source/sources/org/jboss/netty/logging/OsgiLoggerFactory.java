package org.jboss.netty.logging;

import org.osgi.framework.BundleContext;
import org.osgi.framework.ServiceReference;
import org.osgi.service.log.LogService;
import org.osgi.util.tracker.ServiceTracker;
import org.osgi.util.tracker.ServiceTrackerCustomizer;

public class OsgiLoggerFactory extends InternalLoggerFactory {
    private final InternalLoggerFactory fallback;
    volatile LogService logService;
    private final ServiceTracker logServiceTracker;

    public OsgiLoggerFactory(BundleContext ctx) {
        this(ctx, (InternalLoggerFactory) null);
    }

    public OsgiLoggerFactory(BundleContext ctx, InternalLoggerFactory fallback2) {
        if (ctx != null) {
            if (fallback2 == null) {
                fallback2 = InternalLoggerFactory.getDefaultFactory();
                if (fallback2 instanceof OsgiLoggerFactory) {
                    fallback2 = new JdkLoggerFactory();
                }
            }
            this.fallback = fallback2;
            this.logServiceTracker = new ServiceTracker(ctx, "org.osgi.service.log.LogService", (ServiceTrackerCustomizer) null) {
                public Object addingService(ServiceReference reference) {
                    LogService service = (LogService) OsgiLoggerFactory.super.addingService(reference);
                    OsgiLoggerFactory.this.logService = service;
                    return service;
                }

                public void removedService(ServiceReference reference, Object service) {
                    OsgiLoggerFactory.this.logService = null;
                }
            };
            this.logServiceTracker.open();
            return;
        }
        throw new NullPointerException("ctx");
    }

    public InternalLoggerFactory getFallback() {
        return this.fallback;
    }

    public LogService getLogService() {
        return this.logService;
    }

    public void destroy() {
        this.logService = null;
        this.logServiceTracker.close();
    }

    public InternalLogger newInstance(String name) {
        return new OsgiLogger(this, name, this.fallback.newInstance(name));
    }
}
