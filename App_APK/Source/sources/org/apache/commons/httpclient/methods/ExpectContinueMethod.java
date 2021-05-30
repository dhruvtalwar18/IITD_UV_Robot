package org.apache.commons.httpclient.methods;

import java.io.IOException;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.HttpMethodBase;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.HttpVersion;
import org.apache.commons.httpclient.params.HttpMethodParams;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public abstract class ExpectContinueMethod extends HttpMethodBase {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$ExpectContinueMethod;

    /* access modifiers changed from: protected */
    public abstract boolean hasRequestContent();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$ExpectContinueMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.ExpectContinueMethod");
            class$org$apache$commons$httpclient$methods$ExpectContinueMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$ExpectContinueMethod;
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

    public ExpectContinueMethod() {
    }

    public ExpectContinueMethod(String uri) {
        super(uri);
    }

    public boolean getUseExpectHeader() {
        return getParams().getBooleanParameter(HttpMethodParams.USE_EXPECT_CONTINUE, false);
    }

    public void setUseExpectHeader(boolean value) {
        getParams().setBooleanParameter(HttpMethodParams.USE_EXPECT_CONTINUE, value);
    }

    /* access modifiers changed from: protected */
    public void addRequestHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter ExpectContinueMethod.addRequestHeaders(HttpState, HttpConnection)");
        super.addRequestHeaders(state, conn);
        boolean headerPresent = getRequestHeader("Expect") != null;
        if (!getParams().isParameterTrue(HttpMethodParams.USE_EXPECT_CONTINUE) || !getEffectiveVersion().greaterEquals(HttpVersion.HTTP_1_1) || !hasRequestContent()) {
            if (headerPresent) {
                removeRequestHeader("Expect");
            }
        } else if (!headerPresent) {
            setRequestHeader("Expect", "100-continue");
        }
    }
}
