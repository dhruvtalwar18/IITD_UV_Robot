package org.apache.commons.httpclient.methods;

import org.apache.commons.httpclient.HttpMethodBase;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class GetMethod extends HttpMethodBase {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$GetMethod;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$GetMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.GetMethod");
            class$org$apache$commons$httpclient$methods$GetMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$GetMethod;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public GetMethod() {
        setFollowRedirects(true);
    }

    public GetMethod(String uri) {
        super(uri);
        LOG.trace("enter GetMethod(String)");
        setFollowRedirects(true);
    }

    public String getName() {
        return "GET";
    }

    public void recycle() {
        LOG.trace("enter GetMethod.recycle()");
        super.recycle();
        setFollowRedirects(true);
    }
}
