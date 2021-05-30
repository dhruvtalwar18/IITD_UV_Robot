package org.yaml.snakeyaml;

import java.util.Map;
import java.util.TimeZone;
import org.yaml.snakeyaml.emitter.ScalarAnalysis;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.nodes.Tag;

public class DumperOptions {
    private boolean allowReadOnlyProperties = false;
    private boolean allowUnicode = true;
    private int bestWidth = 80;
    private boolean canonical = false;
    private FlowStyle defaultFlowStyle = FlowStyle.AUTO;
    private ScalarStyle defaultStyle = ScalarStyle.PLAIN;
    private boolean explicitEnd = false;
    private Tag explicitRoot = null;
    private boolean explicitStart = false;
    private int indent = 2;
    private LineBreak lineBreak = LineBreak.UNIX;
    private Boolean prettyFlow = false;
    private Map<String, String> tags = null;
    private TimeZone timeZone = null;
    private Version version = null;

    public enum ScalarStyle {
        DOUBLE_QUOTED('\"'),
        SINGLE_QUOTED('\''),
        LITERAL('|'),
        FOLDED('>'),
        PLAIN((String) null);
        
        private Character styleChar;

        private ScalarStyle(Character style) {
            this.styleChar = style;
        }

        public Character getChar() {
            return this.styleChar;
        }

        public String toString() {
            return "Scalar style: '" + this.styleChar + "'";
        }

        public static ScalarStyle createStyle(Character style) {
            if (style == null) {
                return PLAIN;
            }
            char charValue = style.charValue();
            if (charValue == '\"') {
                return DOUBLE_QUOTED;
            }
            if (charValue == '\'') {
                return SINGLE_QUOTED;
            }
            if (charValue == '>') {
                return FOLDED;
            }
            if (charValue == '|') {
                return LITERAL;
            }
            throw new YAMLException("Unknown scalar style character: " + style);
        }
    }

    public enum FlowStyle {
        FLOW(Boolean.TRUE),
        BLOCK(Boolean.FALSE),
        AUTO((String) null);
        
        private Boolean styleBoolean;

        private FlowStyle(Boolean flowStyle) {
            this.styleBoolean = flowStyle;
        }

        public Boolean getStyleBoolean() {
            return this.styleBoolean;
        }

        public String toString() {
            return "Flow style: '" + this.styleBoolean + "'";
        }
    }

    public enum LineBreak {
        WIN("\r\n"),
        MAC("\r"),
        UNIX("\n");
        
        private String lineBreak;

        private LineBreak(String lineBreak2) {
            this.lineBreak = lineBreak2;
        }

        public String getString() {
            return this.lineBreak;
        }

        public String toString() {
            return "Line break: " + name();
        }

        public static LineBreak getPlatformLineBreak() {
            String platformLineBreak = System.getProperty("line.separator");
            for (LineBreak lb : values()) {
                if (lb.lineBreak.equals(platformLineBreak)) {
                    return lb;
                }
            }
            return UNIX;
        }
    }

    public enum Version {
        V1_0(new Integer[]{1, 0}),
        V1_1(new Integer[]{1, 1});
        
        private Integer[] version;

        private Version(Integer[] version2) {
            this.version = version2;
        }

        public Integer[] getArray() {
            return this.version;
        }

        public String toString() {
            return "Version: " + this.version[0] + "." + this.version[1];
        }
    }

    public boolean isAllowUnicode() {
        return this.allowUnicode;
    }

    public void setAllowUnicode(boolean allowUnicode2) {
        this.allowUnicode = allowUnicode2;
    }

    public ScalarStyle getDefaultScalarStyle() {
        return this.defaultStyle;
    }

    public void setDefaultScalarStyle(ScalarStyle defaultStyle2) {
        if (defaultStyle2 != null) {
            this.defaultStyle = defaultStyle2;
            return;
        }
        throw new NullPointerException("Use ScalarStyle enum.");
    }

    public void setIndent(int indent2) {
        if (indent2 < 1) {
            throw new YAMLException("Indent must be at least 1");
        } else if (indent2 <= 10) {
            this.indent = indent2;
        } else {
            throw new YAMLException("Indent must be at most 10");
        }
    }

    public int getIndent() {
        return this.indent;
    }

    public void setVersion(Version version2) {
        this.version = version2;
    }

    public Version getVersion() {
        return this.version;
    }

    public void setCanonical(boolean canonical2) {
        this.canonical = canonical2;
    }

    public boolean isCanonical() {
        return this.canonical;
    }

    public void setPrettyFlow(boolean prettyFlow2) {
        this.prettyFlow = Boolean.valueOf(prettyFlow2);
    }

    public boolean isPrettyFlow() {
        return this.prettyFlow.booleanValue();
    }

    public void setWidth(int bestWidth2) {
        this.bestWidth = bestWidth2;
    }

    public int getWidth() {
        return this.bestWidth;
    }

    public LineBreak getLineBreak() {
        return this.lineBreak;
    }

    public void setDefaultFlowStyle(FlowStyle defaultFlowStyle2) {
        if (defaultFlowStyle2 != null) {
            this.defaultFlowStyle = defaultFlowStyle2;
            return;
        }
        throw new NullPointerException("Use FlowStyle enum.");
    }

    public FlowStyle getDefaultFlowStyle() {
        return this.defaultFlowStyle;
    }

    public Tag getExplicitRoot() {
        return this.explicitRoot;
    }

    public void setExplicitRoot(String expRoot) {
        setExplicitRoot(new Tag(expRoot));
    }

    public void setExplicitRoot(Tag expRoot) {
        if (expRoot != null) {
            this.explicitRoot = expRoot;
            return;
        }
        throw new NullPointerException("Root tag must be specified.");
    }

    public void setLineBreak(LineBreak lineBreak2) {
        if (lineBreak2 != null) {
            this.lineBreak = lineBreak2;
            return;
        }
        throw new NullPointerException("Specify line break.");
    }

    public boolean isExplicitStart() {
        return this.explicitStart;
    }

    public void setExplicitStart(boolean explicitStart2) {
        this.explicitStart = explicitStart2;
    }

    public boolean isExplicitEnd() {
        return this.explicitEnd;
    }

    public void setExplicitEnd(boolean explicitEnd2) {
        this.explicitEnd = explicitEnd2;
    }

    public Map<String, String> getTags() {
        return this.tags;
    }

    public void setTags(Map<String, String> tags2) {
        this.tags = tags2;
    }

    public ScalarStyle calculateScalarStyle(ScalarAnalysis analysis, ScalarStyle style) {
        return style;
    }

    public boolean isAllowReadOnlyProperties() {
        return this.allowReadOnlyProperties;
    }

    public void setAllowReadOnlyProperties(boolean allowReadOnlyProperties2) {
        this.allowReadOnlyProperties = allowReadOnlyProperties2;
    }

    public TimeZone getTimeZone() {
        return this.timeZone;
    }

    public void setTimeZone(TimeZone timeZone2) {
        this.timeZone = timeZone2;
    }
}
