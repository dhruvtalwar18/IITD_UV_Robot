package org.bytedeco.javacpp.tools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

class Context {
    String cppName;
    boolean inaccessible;
    InfoMap infoMap;
    String javaName;
    String namespace;
    Map<String, String> namespaceMap;
    TemplateMap templateMap;
    List<String> usingList;
    Declarator variable;
    boolean virtualize;

    Context() {
        this.namespace = null;
        this.cppName = null;
        this.javaName = null;
        this.inaccessible = false;
        this.virtualize = false;
        this.variable = null;
        this.infoMap = null;
        this.templateMap = null;
        this.usingList = null;
        this.namespaceMap = null;
        this.usingList = new ArrayList();
        this.namespaceMap = new HashMap();
    }

    Context(Context c) {
        this.namespace = null;
        this.cppName = null;
        this.javaName = null;
        this.inaccessible = false;
        this.virtualize = false;
        this.variable = null;
        this.infoMap = null;
        this.templateMap = null;
        this.usingList = null;
        this.namespaceMap = null;
        this.namespace = c.namespace;
        this.cppName = c.cppName;
        this.javaName = c.javaName;
        this.inaccessible = c.inaccessible;
        this.variable = c.variable;
        this.infoMap = c.infoMap;
        this.templateMap = c.templateMap;
        this.usingList = c.usingList;
        this.namespaceMap = c.namespaceMap;
    }

    /* access modifiers changed from: package-private */
    public String[] qualify(String cppName2) {
        String name;
        if (cppName2 == null || cppName2.length() == 0) {
            return new String[0];
        }
        for (Map.Entry<String, String> e : this.namespaceMap.entrySet()) {
            cppName2 = cppName2.replaceAll(e.getKey() + "::", e.getValue() + "::");
        }
        if (cppName2.startsWith("::")) {
            return new String[]{cppName2.substring(2)};
        }
        List<String> names = new ArrayList<>();
        String ns = this.namespace != null ? this.namespace : "";
        while (ns != null) {
            if (ns.length() > 0) {
                name = ns + "::" + cppName2;
            } else {
                name = cppName2;
            }
            TemplateMap map = this.templateMap;
            while (true) {
                if (map == null) {
                    break;
                } else if (name.equals(map.getName())) {
                    String args = "<";
                    String separator = "";
                    for (Type t : map.values()) {
                        if (t != null) {
                            args = args + separator + t.cppName;
                            separator = ",";
                        }
                    }
                    StringBuilder sb = new StringBuilder();
                    sb.append(name);
                    sb.append(args);
                    sb.append(args.endsWith(">") ? " >" : ">");
                    names.add(sb.toString());
                } else {
                    map = map.parent;
                }
            }
            names.add(name);
            String ns2 = this.infoMap.normalize(ns, false, true);
            int i = ns2.lastIndexOf("::");
            ns = i >= 0 ? ns2.substring(0, i) : ns2.length() > 0 ? "" : null;
        }
        for (String s : this.usingList) {
            String prefix = this.infoMap.normalize(cppName2, false, true);
            int i2 = s.lastIndexOf("::") + 2;
            String ns3 = s.substring(0, i2);
            String suffix = s.substring(i2);
            if (suffix.length() == 0 || prefix.equals(suffix)) {
                names.add(ns3 + cppName2);
            }
        }
        return (String[]) names.toArray(new String[names.size()]);
    }

    /* access modifiers changed from: package-private */
    public String shorten(String javaName2) {
        if (this.javaName == null) {
            return javaName2;
        }
        int lastDot = 0;
        String s1 = javaName2;
        String s2 = this.javaName + '.';
        int i = 0;
        while (i < s1.length() && i < s2.length() && s1.charAt(i) == s2.charAt(i)) {
            if (s1.charAt(i) == '.') {
                lastDot = i;
            }
            i++;
        }
        if (lastDot > 0) {
            return javaName2.substring(lastDot + 1);
        }
        return javaName2;
    }
}
