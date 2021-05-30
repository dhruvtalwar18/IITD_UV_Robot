package org.yaml.snakeyaml.resolver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.Tag;

public class Resolver {
    public static final Pattern BOOL = Pattern.compile("^(?:yes|Yes|YES|no|No|NO|true|True|TRUE|false|False|FALSE|on|On|ON|off|Off|OFF)$");
    public static final Pattern EMPTY = Pattern.compile("^$");
    public static final Pattern FLOAT = Pattern.compile("^([-+]?(\\.[0-9]+|[0-9_]+(\\.[0-9_]*)?)([eE][-+]?[0-9]+)?|[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\\.[0-9_]*|[-+]?\\.(?:inf|Inf|INF)|\\.(?:nan|NaN|NAN))$");
    public static final Pattern INT = Pattern.compile("^(?:[-+]?0b[0-1_]+|[-+]?0[0-7_]+|[-+]?(?:0|[1-9][0-9_]*)|[-+]?0x[0-9a-fA-F_]+|[-+]?[1-9][0-9_]*(?::[0-5]?[0-9])+)$");
    public static final Pattern MERGE = Pattern.compile("^(?:<<)$");
    public static final Pattern NULL = Pattern.compile("^(?:~|null|Null|NULL| )$");
    public static final Pattern TIMESTAMP = Pattern.compile("^(?:[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]|[0-9][0-9][0-9][0-9]-[0-9][0-9]?-[0-9][0-9]?(?:[Tt]|[ \t]+)[0-9][0-9]?:[0-9][0-9]:[0-9][0-9](?:\\.[0-9]*)?(?:[ \t]*(?:Z|[-+][0-9][0-9]?(?::[0-9][0-9])?))?)$");
    public static final Pattern VALUE = Pattern.compile("^(?:=)$");
    public static final Pattern YAML = Pattern.compile("^(?:!|&|\\*)$");
    protected Map<Character, List<ResolverTuple>> yamlImplicitResolvers;

    public Resolver(boolean respectDefaultImplicitScalars) {
        this.yamlImplicitResolvers = new HashMap();
        if (respectDefaultImplicitScalars) {
            addImplicitResolvers();
        }
    }

    /* access modifiers changed from: protected */
    public void addImplicitResolvers() {
        addImplicitResolver(Tag.BOOL, BOOL, "yYnNtTfFoO");
        addImplicitResolver(Tag.INT, INT, "-+0123456789");
        addImplicitResolver(Tag.FLOAT, FLOAT, "-+0123456789.");
        addImplicitResolver(Tag.MERGE, MERGE, "<");
        addImplicitResolver(Tag.NULL, NULL, "~nN\u0000");
        addImplicitResolver(Tag.NULL, EMPTY, (String) null);
        addImplicitResolver(Tag.TIMESTAMP, TIMESTAMP, "0123456789");
        addImplicitResolver(Tag.VALUE, VALUE, "=");
        addImplicitResolver(Tag.YAML, YAML, "!&*");
    }

    public Resolver() {
        this(true);
    }

    public void addImplicitResolver(Tag tag, Pattern regexp, String first) {
        if (first == null) {
            List<ResolverTuple> curr = this.yamlImplicitResolvers.get((Object) null);
            if (curr == null) {
                curr = new ArrayList<>();
                this.yamlImplicitResolvers.put((Object) null, curr);
            }
            curr.add(new ResolverTuple(tag, regexp));
            return;
        }
        for (char valueOf : first.toCharArray()) {
            Character theC = Character.valueOf(valueOf);
            if (theC.charValue() == 0) {
                theC = null;
            }
            List<ResolverTuple> curr2 = this.yamlImplicitResolvers.get(theC);
            if (curr2 == null) {
                curr2 = new ArrayList<>();
                this.yamlImplicitResolvers.put(theC, curr2);
            }
            curr2.add(new ResolverTuple(tag, regexp));
        }
    }

    public Tag resolve(NodeId kind, String value, boolean implicit) {
        List<ResolverTuple> resolvers;
        if (kind == NodeId.scalar && implicit) {
            if (value.length() == 0) {
                resolvers = this.yamlImplicitResolvers.get(0);
            } else {
                resolvers = this.yamlImplicitResolvers.get(Character.valueOf(value.charAt(0)));
            }
            if (resolvers != null) {
                for (ResolverTuple v : resolvers) {
                    Tag tag = v.getTag();
                    if (v.getRegexp().matcher(value).matches()) {
                        return tag;
                    }
                }
            }
            if (this.yamlImplicitResolvers.containsKey((Object) null)) {
                for (ResolverTuple v2 : this.yamlImplicitResolvers.get((Object) null)) {
                    Tag tag2 = v2.getTag();
                    if (v2.getRegexp().matcher(value).matches()) {
                        return tag2;
                    }
                }
            }
        }
        switch (kind) {
            case scalar:
                return Tag.STR;
            case sequence:
                return Tag.SEQ;
            default:
                return Tag.MAP;
        }
    }
}
