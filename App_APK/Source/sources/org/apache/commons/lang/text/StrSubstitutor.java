package org.apache.commons.lang.text;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

public class StrSubstitutor {
    public static final char DEFAULT_ESCAPE = '$';
    public static final StrMatcher DEFAULT_PREFIX = StrMatcher.stringMatcher("${");
    public static final StrMatcher DEFAULT_SUFFIX = StrMatcher.stringMatcher("}");
    private char escapeChar;
    private StrMatcher prefixMatcher;
    private StrMatcher suffixMatcher;
    private StrLookup variableResolver;

    public static String replace(Object source, Map valueMap) {
        return new StrSubstitutor(valueMap).replace(source);
    }

    public static String replace(Object source, Map valueMap, String prefix, String suffix) {
        return new StrSubstitutor(valueMap, prefix, suffix).replace(source);
    }

    public static String replaceSystemProperties(Object source) {
        return new StrSubstitutor(StrLookup.systemPropertiesLookup()).replace(source);
    }

    public StrSubstitutor() {
        this((StrLookup) null, DEFAULT_PREFIX, DEFAULT_SUFFIX, '$');
    }

    public StrSubstitutor(Map valueMap) {
        this(StrLookup.mapLookup(valueMap), DEFAULT_PREFIX, DEFAULT_SUFFIX, '$');
    }

    public StrSubstitutor(Map valueMap, String prefix, String suffix) {
        this(StrLookup.mapLookup(valueMap), prefix, suffix, '$');
    }

    public StrSubstitutor(Map valueMap, String prefix, String suffix, char escape) {
        this(StrLookup.mapLookup(valueMap), prefix, suffix, escape);
    }

    public StrSubstitutor(StrLookup variableResolver2) {
        this(variableResolver2, DEFAULT_PREFIX, DEFAULT_SUFFIX, '$');
    }

    public StrSubstitutor(StrLookup variableResolver2, String prefix, String suffix, char escape) {
        setVariableResolver(variableResolver2);
        setVariablePrefix(prefix);
        setVariableSuffix(suffix);
        setEscapeChar(escape);
    }

    public StrSubstitutor(StrLookup variableResolver2, StrMatcher prefixMatcher2, StrMatcher suffixMatcher2, char escape) {
        setVariableResolver(variableResolver2);
        setVariablePrefixMatcher(prefixMatcher2);
        setVariableSuffixMatcher(suffixMatcher2);
        setEscapeChar(escape);
    }

    public String replace(String source) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(source);
        if (!substitute(buf, 0, source.length())) {
            return source;
        }
        return buf.toString();
    }

    public String replace(String source, int offset, int length) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(length).append(source, offset, length);
        if (!substitute(buf, 0, length)) {
            return source.substring(offset, offset + length);
        }
        return buf.toString();
    }

    public String replace(char[] source) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(source.length).append(source);
        substitute(buf, 0, source.length);
        return buf.toString();
    }

    public String replace(char[] source, int offset, int length) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(length).append(source, offset, length);
        substitute(buf, 0, length);
        return buf.toString();
    }

    public String replace(StringBuffer source) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(source.length()).append(source);
        substitute(buf, 0, buf.length());
        return buf.toString();
    }

    public String replace(StringBuffer source, int offset, int length) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(length).append(source, offset, length);
        substitute(buf, 0, length);
        return buf.toString();
    }

    public String replace(StrBuilder source) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(source.length()).append(source);
        substitute(buf, 0, buf.length());
        return buf.toString();
    }

    public String replace(StrBuilder source, int offset, int length) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder(length).append(source, offset, length);
        substitute(buf, 0, length);
        return buf.toString();
    }

    public String replace(Object source) {
        if (source == null) {
            return null;
        }
        StrBuilder buf = new StrBuilder().append(source);
        substitute(buf, 0, buf.length());
        return buf.toString();
    }

    public boolean replaceIn(StringBuffer source) {
        if (source == null) {
            return false;
        }
        return replaceIn(source, 0, source.length());
    }

    public boolean replaceIn(StringBuffer source, int offset, int length) {
        if (source == null) {
            return false;
        }
        StrBuilder buf = new StrBuilder(length).append(source, offset, length);
        if (!substitute(buf, 0, length)) {
            return false;
        }
        source.replace(offset, offset + length, buf.toString());
        return true;
    }

    public boolean replaceIn(StrBuilder source) {
        if (source == null) {
            return false;
        }
        return substitute(source, 0, source.length());
    }

    public boolean replaceIn(StrBuilder source, int offset, int length) {
        if (source == null) {
            return false;
        }
        return substitute(source, offset, length);
    }

    /* access modifiers changed from: protected */
    public boolean substitute(StrBuilder buf, int offset, int length) {
        return substitute(buf, offset, length, (List) null) > 0;
    }

    private int substitute(StrBuilder buf, int offset, int length, List priorVariables) {
        char c;
        StrMatcher strMatcher;
        StrMatcher prefixMatcher2;
        StrBuilder strBuilder = buf;
        int i = offset;
        int i2 = length;
        StrMatcher prefixMatcher3 = getVariablePrefixMatcher();
        StrMatcher suffixMatcher2 = getVariableSuffixMatcher();
        char escape = getEscapeChar();
        boolean top = priorVariables == null;
        boolean altered = false;
        int lengthChange = 0;
        char[] chars = strBuilder.buffer;
        int bufEnd = i + i2;
        List priorVariables2 = priorVariables;
        int pos = i;
        while (pos < bufEnd) {
            int startMatchLen = prefixMatcher3.isMatch(chars, pos, i, bufEnd);
            if (startMatchLen == 0) {
                pos++;
                prefixMatcher2 = prefixMatcher3;
                strMatcher = suffixMatcher2;
                c = escape;
            } else if (pos <= i || chars[pos - 1] != escape) {
                int startPos = pos;
                int pos2 = pos + startMatchLen;
                while (true) {
                    if (pos2 >= bufEnd) {
                        prefixMatcher2 = prefixMatcher3;
                        strMatcher = suffixMatcher2;
                        c = escape;
                        pos = pos2;
                        break;
                    }
                    int endMatchLen = suffixMatcher2.isMatch(chars, pos2, i, bufEnd);
                    if (endMatchLen == 0) {
                        pos2++;
                    } else {
                        prefixMatcher2 = prefixMatcher3;
                        strMatcher = suffixMatcher2;
                        c = escape;
                        String varName = new String(chars, startPos + startMatchLen, (pos2 - startPos) - startMatchLen);
                        int pos3 = pos2 + endMatchLen;
                        int endPos = pos3;
                        if (priorVariables2 == null) {
                            priorVariables2 = new ArrayList();
                            priorVariables2.add(new String(chars, i, i2));
                        }
                        checkCyclicSubstitution(varName, priorVariables2);
                        priorVariables2.add(varName);
                        String varValue = resolveVariable(varName, strBuilder, startPos, endPos);
                        if (varValue != null) {
                            int varLen = varValue.length();
                            strBuilder.replace(startPos, endPos, varValue);
                            altered = true;
                            int change = substitute(strBuilder, startPos, varLen, priorVariables2) + (varLen - (endPos - startPos));
                            pos3 += change;
                            bufEnd += change;
                            lengthChange += change;
                            chars = strBuilder.buffer;
                        }
                        priorVariables2.remove(priorVariables2.size() - 1);
                        pos = pos3;
                    }
                }
                prefixMatcher3 = prefixMatcher2;
                suffixMatcher2 = strMatcher;
                escape = c;
                i = offset;
            } else {
                strBuilder.deleteCharAt(pos - 1);
                lengthChange--;
                altered = true;
                bufEnd--;
                prefixMatcher2 = prefixMatcher3;
                strMatcher = suffixMatcher2;
                c = escape;
                chars = strBuilder.buffer;
            }
            prefixMatcher3 = prefixMatcher2;
            suffixMatcher2 = strMatcher;
            escape = c;
            i = offset;
        }
        StrMatcher strMatcher2 = suffixMatcher2;
        char c2 = escape;
        if (!top) {
            return lengthChange;
        }
        if (altered) {
            return 1;
        }
        return 0;
    }

    private void checkCyclicSubstitution(String varName, List priorVariables) {
        if (priorVariables.contains(varName)) {
            StrBuilder buf = new StrBuilder(256);
            buf.append("Infinite loop in property interpolation of ");
            buf.append(priorVariables.remove(0));
            buf.append(": ");
            buf.appendWithSeparators((Collection) priorVariables, "->");
            throw new IllegalStateException(buf.toString());
        }
    }

    /* access modifiers changed from: protected */
    public String resolveVariable(String variableName, StrBuilder buf, int startPos, int endPos) {
        StrLookup resolver = getVariableResolver();
        if (resolver == null) {
            return null;
        }
        return resolver.lookup(variableName);
    }

    public char getEscapeChar() {
        return this.escapeChar;
    }

    public void setEscapeChar(char escapeCharacter) {
        this.escapeChar = escapeCharacter;
    }

    public StrMatcher getVariablePrefixMatcher() {
        return this.prefixMatcher;
    }

    public StrSubstitutor setVariablePrefixMatcher(StrMatcher prefixMatcher2) {
        if (prefixMatcher2 != null) {
            this.prefixMatcher = prefixMatcher2;
            return this;
        }
        throw new IllegalArgumentException("Variable prefix matcher must not be null!");
    }

    public StrSubstitutor setVariablePrefix(char prefix) {
        return setVariablePrefixMatcher(StrMatcher.charMatcher(prefix));
    }

    public StrSubstitutor setVariablePrefix(String prefix) {
        if (prefix != null) {
            return setVariablePrefixMatcher(StrMatcher.stringMatcher(prefix));
        }
        throw new IllegalArgumentException("Variable prefix must not be null!");
    }

    public StrMatcher getVariableSuffixMatcher() {
        return this.suffixMatcher;
    }

    public StrSubstitutor setVariableSuffixMatcher(StrMatcher suffixMatcher2) {
        if (suffixMatcher2 != null) {
            this.suffixMatcher = suffixMatcher2;
            return this;
        }
        throw new IllegalArgumentException("Variable suffix matcher must not be null!");
    }

    public StrSubstitutor setVariableSuffix(char suffix) {
        return setVariableSuffixMatcher(StrMatcher.charMatcher(suffix));
    }

    public StrSubstitutor setVariableSuffix(String suffix) {
        if (suffix != null) {
            return setVariableSuffixMatcher(StrMatcher.stringMatcher(suffix));
        }
        throw new IllegalArgumentException("Variable suffix must not be null!");
    }

    public StrLookup getVariableResolver() {
        return this.variableResolver;
    }

    public void setVariableResolver(StrLookup variableResolver2) {
        this.variableResolver = variableResolver2;
    }
}
