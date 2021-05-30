package org.jboss.netty.handler.codec.serialization;

import java.io.FilterInputStream;
import java.io.InputStream;

final class SwitchableInputStream extends FilterInputStream {
    SwitchableInputStream() {
        super((InputStream) null);
    }

    /* access modifiers changed from: package-private */
    public void switchStream(InputStream in) {
        this.in = in;
    }
}
