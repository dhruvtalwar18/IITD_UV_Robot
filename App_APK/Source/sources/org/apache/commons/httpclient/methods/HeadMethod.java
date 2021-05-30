package org.apache.commons.httpclient.methods;

import java.io.IOException;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.HttpMethodBase;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.ProtocolException;
import org.apache.commons.httpclient.params.HttpMethodParams;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HeadMethod extends HttpMethodBase {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$HeadMethod;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$HeadMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.HeadMethod");
            class$org$apache$commons$httpclient$methods$HeadMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$HeadMethod;
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

    public HeadMethod() {
        setFollowRedirects(true);
    }

    public HeadMethod(String uri) {
        super(uri);
        setFollowRedirects(true);
    }

    public String getName() {
        return "HEAD";
    }

    public void recycle() {
        super.recycle();
        setFollowRedirects(true);
    }

    /* access modifiers changed from: protected */
    public void readResponseBody(HttpState state, HttpConnection conn) throws HttpException, IOException {
        boolean responseAvailable;
        LOG.trace("enter HeadMethod.readResponseBody(HttpState, HttpConnection)");
        int bodyCheckTimeout = getParams().getIntParameter(HttpMethodParams.HEAD_BODY_CHECK_TIMEOUT, -1);
        if (bodyCheckTimeout < 0) {
            responseBodyConsumed();
            return;
        }
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Check for non-compliant response body. Timeout in ");
            stringBuffer.append(bodyCheckTimeout);
            stringBuffer.append(" ms");
            log.debug(stringBuffer.toString());
        }
        try {
            responseAvailable = conn.isResponseAvailable(bodyCheckTimeout);
        } catch (IOException e) {
            LOG.debug("An IOException occurred while testing if a response was available, we will assume one is not.", e);
            responseAvailable = false;
        }
        if (!responseAvailable) {
            return;
        }
        if (!getParams().isParameterTrue(HttpMethodParams.REJECT_HEAD_BODY)) {
            LOG.warn("Body content returned in response to HTTP HEAD");
            super.readResponseBody(state, conn);
            return;
        }
        throw new ProtocolException("Body content may not be sent in response to HTTP HEAD request");
    }

    public int getBodyCheckTimeout() {
        return getParams().getIntParameter(HttpMethodParams.HEAD_BODY_CHECK_TIMEOUT, -1);
    }

    public void setBodyCheckTimeout(int timeout) {
        getParams().setIntParameter(HttpMethodParams.HEAD_BODY_CHECK_TIMEOUT, timeout);
    }
}
