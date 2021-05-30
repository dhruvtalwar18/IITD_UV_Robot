package org.jboss.netty.channel.local;

import java.net.SocketAddress;

public final class LocalAddress extends SocketAddress implements Comparable<LocalAddress> {
    public static final String EPHEMERAL = "ephemeral";
    private static final long serialVersionUID = -3601961747680808645L;
    private final boolean ephemeral;
    private final String id;

    public LocalAddress(int id2) {
        this(String.valueOf(id2));
    }

    public LocalAddress(String id2) {
        if (id2 != null) {
            String id3 = id2.trim().toLowerCase();
            if (id3.length() != 0) {
                this.id = id3;
                this.ephemeral = id3.equals(EPHEMERAL);
                return;
            }
            throw new IllegalArgumentException("empty id");
        }
        throw new NullPointerException("id");
    }

    public String getId() {
        return this.id;
    }

    public boolean isEphemeral() {
        return this.ephemeral;
    }

    public int hashCode() {
        if (this.ephemeral) {
            return System.identityHashCode(this);
        }
        return this.id.hashCode();
    }

    public boolean equals(Object o) {
        if (!(o instanceof LocalAddress)) {
            return false;
        }
        if (!this.ephemeral) {
            return getId().equals(((LocalAddress) o).getId());
        }
        if (this == o) {
            return true;
        }
        return false;
    }

    public int compareTo(LocalAddress o) {
        if (this.ephemeral) {
            if (!o.ephemeral) {
                return 1;
            }
            if (this == o) {
                return 0;
            }
            int a = System.identityHashCode(this);
            int b = System.identityHashCode(o);
            if (a < b) {
                return -1;
            }
            if (a > b) {
                return 1;
            }
            throw new Error("Two different ephemeral addresses have same identityHashCode.");
        } else if (o.ephemeral) {
            return -1;
        } else {
            return getId().compareTo(o.getId());
        }
    }

    public String toString() {
        return "local:" + getId();
    }
}
