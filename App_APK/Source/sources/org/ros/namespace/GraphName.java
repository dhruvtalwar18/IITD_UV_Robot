package org.ros.namespace;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Pattern;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.ros.exception.RosRuntimeException;

public class GraphName {
    @VisibleForTesting
    static final String ANONYMOUS_PREFIX = "anonymous_";
    private static final String ROOT = "/";
    private static final String SEPARATOR = "/";
    private static final Pattern VALID_GRAPH_NAME_PATTERN = Pattern.compile("^([\\~\\/A-Za-z][\\w_\\/]*)?$");
    private static AtomicInteger anonymousCounter = new AtomicInteger();
    private final String name;

    public static GraphName newAnonymous() {
        return new GraphName(ANONYMOUS_PREFIX + anonymousCounter.incrementAndGet());
    }

    public static GraphName root() {
        return new GraphName(CookieSpec.PATH_DELIM);
    }

    public static GraphName empty() {
        return new GraphName("");
    }

    public static GraphName of(String name2) {
        return new GraphName(canonicalize(name2));
    }

    private GraphName(String name2) {
        Preconditions.checkNotNull(name2);
        this.name = name2;
    }

    private static String canonicalize(String name2) {
        if (VALID_GRAPH_NAME_PATTERN.matcher(name2).matches()) {
            while (!name2.equals(CookieSpec.PATH_DELIM) && name2.endsWith(CookieSpec.PATH_DELIM)) {
                name2 = name2.substring(0, name2.length() - 1);
            }
            if (!name2.startsWith("~/")) {
                return name2;
            }
            return "~" + name2.substring(2);
        }
        throw new RosRuntimeException("Invalid graph name: " + name2);
    }

    public boolean isGlobal() {
        return !isEmpty() && this.name.charAt(0) == '/';
    }

    public boolean isRoot() {
        return this.name.equals(CookieSpec.PATH_DELIM);
    }

    public boolean isEmpty() {
        return this.name.isEmpty();
    }

    public boolean isPrivate() {
        return !isEmpty() && this.name.charAt(0) == '~';
    }

    public boolean isRelative() {
        return !isPrivate() && !isGlobal();
    }

    public GraphName getParent() {
        if (this.name.length() == 0) {
            return empty();
        }
        if (this.name.equals(CookieSpec.PATH_DELIM)) {
            return root();
        }
        int slashIdx = this.name.lastIndexOf(47);
        if (slashIdx > 1) {
            return new GraphName(this.name.substring(0, slashIdx));
        }
        if (isGlobal()) {
            return root();
        }
        return empty();
    }

    public GraphName getBasename() {
        int slashIdx = this.name.lastIndexOf(47);
        if (slashIdx <= -1) {
            return this;
        }
        if (slashIdx + 1 < this.name.length()) {
            return new GraphName(this.name.substring(slashIdx + 1));
        }
        return empty();
    }

    public GraphName toRelative() {
        if (isPrivate() || isGlobal()) {
            return new GraphName(this.name.substring(1));
        }
        return this;
    }

    public GraphName toGlobal() {
        if (isGlobal()) {
            return this;
        }
        if (isPrivate()) {
            return new GraphName(CookieSpec.PATH_DELIM + this.name.substring(1));
        }
        return new GraphName(CookieSpec.PATH_DELIM + this.name);
    }

    public GraphName join(GraphName other) {
        if (other.isGlobal() || isEmpty()) {
            return other;
        }
        if (isRoot()) {
            return other.toGlobal();
        }
        if (other.isEmpty()) {
            return this;
        }
        return new GraphName(toString() + CookieSpec.PATH_DELIM + other.toString());
    }

    public GraphName join(String other) {
        return join(of(other));
    }

    public String toString() {
        return this.name;
    }

    public int hashCode() {
        return this.name.hashCode();
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        GraphName other = (GraphName) obj;
        if (this.name == null) {
            if (other.name != null) {
                return false;
            }
        } else if (!this.name.equals(other.name)) {
            return false;
        }
        return true;
    }
}
