package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdyPingFrame implements SpdyPingFrame {
    private int id;

    public DefaultSpdyPingFrame(int id2) {
        setId(id2);
    }

    public int getID() {
        return getId();
    }

    public int getId() {
        return this.id;
    }

    public void setID(int id2) {
        setId(id2);
    }

    public void setId(int id2) {
        this.id = id2;
    }

    public String toString() {
        return getClass().getSimpleName() + StringUtil.NEWLINE + "--> ID = " + this.id;
    }
}
