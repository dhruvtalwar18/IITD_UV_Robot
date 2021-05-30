package org.apache.commons.lang.text;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.ListIterator;
import java.util.NoSuchElementException;

public class StrTokenizer implements ListIterator, Cloneable {
    private static final StrTokenizer CSV_TOKENIZER_PROTOTYPE = new StrTokenizer();
    private static final StrTokenizer TSV_TOKENIZER_PROTOTYPE = new StrTokenizer();
    private char[] chars;
    private StrMatcher delimMatcher;
    private boolean emptyAsNull;
    private boolean ignoreEmptyTokens;
    private StrMatcher ignoredMatcher;
    private StrMatcher quoteMatcher;
    private int tokenPos;
    private String[] tokens;
    private StrMatcher trimmerMatcher;

    static {
        CSV_TOKENIZER_PROTOTYPE.setDelimiterMatcher(StrMatcher.commaMatcher());
        CSV_TOKENIZER_PROTOTYPE.setQuoteMatcher(StrMatcher.doubleQuoteMatcher());
        CSV_TOKENIZER_PROTOTYPE.setIgnoredMatcher(StrMatcher.noneMatcher());
        CSV_TOKENIZER_PROTOTYPE.setTrimmerMatcher(StrMatcher.trimMatcher());
        CSV_TOKENIZER_PROTOTYPE.setEmptyTokenAsNull(false);
        CSV_TOKENIZER_PROTOTYPE.setIgnoreEmptyTokens(false);
        TSV_TOKENIZER_PROTOTYPE.setDelimiterMatcher(StrMatcher.tabMatcher());
        TSV_TOKENIZER_PROTOTYPE.setQuoteMatcher(StrMatcher.doubleQuoteMatcher());
        TSV_TOKENIZER_PROTOTYPE.setIgnoredMatcher(StrMatcher.noneMatcher());
        TSV_TOKENIZER_PROTOTYPE.setTrimmerMatcher(StrMatcher.trimMatcher());
        TSV_TOKENIZER_PROTOTYPE.setEmptyTokenAsNull(false);
        TSV_TOKENIZER_PROTOTYPE.setIgnoreEmptyTokens(false);
    }

    private static StrTokenizer getCSVClone() {
        return (StrTokenizer) CSV_TOKENIZER_PROTOTYPE.clone();
    }

    public static StrTokenizer getCSVInstance() {
        return getCSVClone();
    }

    public static StrTokenizer getCSVInstance(String input) {
        StrTokenizer tok = getCSVClone();
        tok.reset(input);
        return tok;
    }

    public static StrTokenizer getCSVInstance(char[] input) {
        StrTokenizer tok = getCSVClone();
        tok.reset(input);
        return tok;
    }

    private static StrTokenizer getTSVClone() {
        return (StrTokenizer) TSV_TOKENIZER_PROTOTYPE.clone();
    }

    public static StrTokenizer getTSVInstance() {
        return getTSVClone();
    }

    public static StrTokenizer getTSVInstance(String input) {
        StrTokenizer tok = getTSVClone();
        tok.reset(input);
        return tok;
    }

    public static StrTokenizer getTSVInstance(char[] input) {
        StrTokenizer tok = getTSVClone();
        tok.reset(input);
        return tok;
    }

    public StrTokenizer() {
        this.delimMatcher = StrMatcher.splitMatcher();
        this.quoteMatcher = StrMatcher.noneMatcher();
        this.ignoredMatcher = StrMatcher.noneMatcher();
        this.trimmerMatcher = StrMatcher.noneMatcher();
        this.emptyAsNull = false;
        this.ignoreEmptyTokens = true;
        this.chars = null;
    }

    public StrTokenizer(String input) {
        this.delimMatcher = StrMatcher.splitMatcher();
        this.quoteMatcher = StrMatcher.noneMatcher();
        this.ignoredMatcher = StrMatcher.noneMatcher();
        this.trimmerMatcher = StrMatcher.noneMatcher();
        this.emptyAsNull = false;
        this.ignoreEmptyTokens = true;
        if (input != null) {
            this.chars = input.toCharArray();
        } else {
            this.chars = null;
        }
    }

    public StrTokenizer(String input, char delim) {
        this(input);
        setDelimiterChar(delim);
    }

    public StrTokenizer(String input, String delim) {
        this(input);
        setDelimiterString(delim);
    }

    public StrTokenizer(String input, StrMatcher delim) {
        this(input);
        setDelimiterMatcher(delim);
    }

    public StrTokenizer(String input, char delim, char quote) {
        this(input, delim);
        setQuoteChar(quote);
    }

    public StrTokenizer(String input, StrMatcher delim, StrMatcher quote) {
        this(input, delim);
        setQuoteMatcher(quote);
    }

    public StrTokenizer(char[] input) {
        this.delimMatcher = StrMatcher.splitMatcher();
        this.quoteMatcher = StrMatcher.noneMatcher();
        this.ignoredMatcher = StrMatcher.noneMatcher();
        this.trimmerMatcher = StrMatcher.noneMatcher();
        this.emptyAsNull = false;
        this.ignoreEmptyTokens = true;
        this.chars = input;
    }

    public StrTokenizer(char[] input, char delim) {
        this(input);
        setDelimiterChar(delim);
    }

    public StrTokenizer(char[] input, String delim) {
        this(input);
        setDelimiterString(delim);
    }

    public StrTokenizer(char[] input, StrMatcher delim) {
        this(input);
        setDelimiterMatcher(delim);
    }

    public StrTokenizer(char[] input, char delim, char quote) {
        this(input, delim);
        setQuoteChar(quote);
    }

    public StrTokenizer(char[] input, StrMatcher delim, StrMatcher quote) {
        this(input, delim);
        setQuoteMatcher(quote);
    }

    public int size() {
        checkTokenized();
        return this.tokens.length;
    }

    public String nextToken() {
        if (!hasNext()) {
            return null;
        }
        String[] strArr = this.tokens;
        int i = this.tokenPos;
        this.tokenPos = i + 1;
        return strArr[i];
    }

    public String previousToken() {
        if (!hasPrevious()) {
            return null;
        }
        String[] strArr = this.tokens;
        int i = this.tokenPos - 1;
        this.tokenPos = i;
        return strArr[i];
    }

    public String[] getTokenArray() {
        checkTokenized();
        return (String[]) this.tokens.clone();
    }

    public List getTokenList() {
        checkTokenized();
        List list = new ArrayList(this.tokens.length);
        for (String add : this.tokens) {
            list.add(add);
        }
        return list;
    }

    public StrTokenizer reset() {
        this.tokenPos = 0;
        this.tokens = null;
        return this;
    }

    public StrTokenizer reset(String input) {
        reset();
        if (input != null) {
            this.chars = input.toCharArray();
        } else {
            this.chars = null;
        }
        return this;
    }

    public StrTokenizer reset(char[] input) {
        reset();
        this.chars = input;
        return this;
    }

    public boolean hasNext() {
        checkTokenized();
        return this.tokenPos < this.tokens.length;
    }

    public Object next() {
        if (hasNext()) {
            String[] strArr = this.tokens;
            int i = this.tokenPos;
            this.tokenPos = i + 1;
            return strArr[i];
        }
        throw new NoSuchElementException();
    }

    public int nextIndex() {
        return this.tokenPos;
    }

    public boolean hasPrevious() {
        checkTokenized();
        return this.tokenPos > 0;
    }

    public Object previous() {
        if (hasPrevious()) {
            String[] strArr = this.tokens;
            int i = this.tokenPos - 1;
            this.tokenPos = i;
            return strArr[i];
        }
        throw new NoSuchElementException();
    }

    public int previousIndex() {
        return this.tokenPos - 1;
    }

    public void remove() {
        throw new UnsupportedOperationException("remove() is unsupported");
    }

    public void set(Object obj) {
        throw new UnsupportedOperationException("set() is unsupported");
    }

    public void add(Object obj) {
        throw new UnsupportedOperationException("add() is unsupported");
    }

    private void checkTokenized() {
        if (this.tokens != null) {
            return;
        }
        if (this.chars == null) {
            List split = tokenize((char[]) null, 0, 0);
            this.tokens = (String[]) split.toArray(new String[split.size()]);
            return;
        }
        List split2 = tokenize(this.chars, 0, this.chars.length);
        this.tokens = (String[]) split2.toArray(new String[split2.size()]);
    }

    /* access modifiers changed from: protected */
    public List tokenize(char[] chars2, int offset, int count) {
        if (chars2 == null || count == 0) {
            return Collections.EMPTY_LIST;
        }
        StrBuilder buf = new StrBuilder();
        List tokens2 = new ArrayList();
        int pos = offset;
        while (true) {
            int pos2 = pos;
            if (pos2 < 0 || pos2 >= count) {
                return tokens2;
            }
            pos = readNextToken(chars2, pos2, count, buf, tokens2);
            if (pos >= count) {
                addToken(tokens2, "");
            }
        }
        return tokens2;
    }

    private void addToken(List list, String tok) {
        if (tok == null || tok.length() == 0) {
            if (!isIgnoreEmptyTokens()) {
                if (isEmptyTokenAsNull()) {
                    tok = null;
                }
            } else {
                return;
            }
        }
        list.add(tok);
    }

    private int readNextToken(char[] chars2, int start, int len, StrBuilder workArea, List tokens2) {
        while (start < len) {
            int removeLen = Math.max(getIgnoredMatcher().isMatch(chars2, start, start, len), getTrimmerMatcher().isMatch(chars2, start, start, len));
            if (removeLen == 0 || getDelimiterMatcher().isMatch(chars2, start, start, len) > 0 || getQuoteMatcher().isMatch(chars2, start, start, len) > 0) {
                break;
            }
            start += removeLen;
        }
        if (start >= len) {
            addToken(tokens2, "");
            return -1;
        }
        int delimLen = getDelimiterMatcher().isMatch(chars2, start, start, len);
        if (delimLen > 0) {
            addToken(tokens2, "");
            return start + delimLen;
        }
        int quoteLen = getQuoteMatcher().isMatch(chars2, start, start, len);
        if (quoteLen <= 0) {
            return readWithQuotes(chars2, start, len, workArea, tokens2, 0, 0);
        }
        return readWithQuotes(chars2, start + quoteLen, len, workArea, tokens2, start, quoteLen);
    }

    private int readWithQuotes(char[] chars2, int start, int len, StrBuilder workArea, List tokens2, int quoteStart, int quoteLen) {
        int trimStart;
        char[] cArr = chars2;
        int i = start;
        int i2 = len;
        StrBuilder strBuilder = workArea;
        List list = tokens2;
        int i3 = quoteLen;
        workArea.clear();
        int pos = start;
        boolean quoting = i3 > 0;
        int pos2 = 0;
        while (true) {
            int trimStart2 = pos2;
            if (pos < i2) {
                if (quoting) {
                    trimStart = trimStart2;
                    if (isQuote(chars2, pos, len, quoteStart, quoteLen)) {
                        if (isQuote(chars2, pos + i3, len, quoteStart, quoteLen)) {
                            strBuilder.append(cArr, pos, i3);
                            pos += i3 * 2;
                            pos2 = workArea.size();
                        } else {
                            quoting = false;
                            pos += i3;
                        }
                    } else {
                        strBuilder.append(cArr[pos]);
                        pos++;
                        pos2 = workArea.size();
                    }
                } else {
                    trimStart = trimStart2;
                    int delimLen = getDelimiterMatcher().isMatch(cArr, pos, i, i2);
                    if (delimLen > 0) {
                        addToken(list, strBuilder.substring(0, trimStart));
                        return pos + delimLen;
                    } else if (i3 <= 0 || !isQuote(chars2, pos, len, quoteStart, quoteLen)) {
                        int ignoredLen = getIgnoredMatcher().isMatch(cArr, pos, i, i2);
                        if (ignoredLen > 0) {
                            pos += ignoredLen;
                        } else {
                            int trimmedLen = getTrimmerMatcher().isMatch(cArr, pos, i, i2);
                            if (trimmedLen > 0) {
                                strBuilder.append(cArr, pos, trimmedLen);
                                pos += trimmedLen;
                            } else {
                                strBuilder.append(cArr[pos]);
                                pos2 = workArea.size();
                                pos++;
                            }
                        }
                    } else {
                        quoting = true;
                        pos += i3;
                    }
                }
                pos2 = trimStart;
            } else {
                addToken(list, strBuilder.substring(0, trimStart2));
                return -1;
            }
        }
    }

    private boolean isQuote(char[] chars2, int pos, int len, int quoteStart, int quoteLen) {
        for (int i = 0; i < quoteLen; i++) {
            if (pos + i >= len || chars2[pos + i] != chars2[quoteStart + i]) {
                return false;
            }
        }
        return true;
    }

    public StrMatcher getDelimiterMatcher() {
        return this.delimMatcher;
    }

    public StrTokenizer setDelimiterMatcher(StrMatcher delim) {
        if (delim == null) {
            this.delimMatcher = StrMatcher.noneMatcher();
        } else {
            this.delimMatcher = delim;
        }
        return this;
    }

    public StrTokenizer setDelimiterChar(char delim) {
        return setDelimiterMatcher(StrMatcher.charMatcher(delim));
    }

    public StrTokenizer setDelimiterString(String delim) {
        return setDelimiterMatcher(StrMatcher.stringMatcher(delim));
    }

    public StrMatcher getQuoteMatcher() {
        return this.quoteMatcher;
    }

    public StrTokenizer setQuoteMatcher(StrMatcher quote) {
        if (quote != null) {
            this.quoteMatcher = quote;
        }
        return this;
    }

    public StrTokenizer setQuoteChar(char quote) {
        return setQuoteMatcher(StrMatcher.charMatcher(quote));
    }

    public StrMatcher getIgnoredMatcher() {
        return this.ignoredMatcher;
    }

    public StrTokenizer setIgnoredMatcher(StrMatcher ignored) {
        if (ignored != null) {
            this.ignoredMatcher = ignored;
        }
        return this;
    }

    public StrTokenizer setIgnoredChar(char ignored) {
        return setIgnoredMatcher(StrMatcher.charMatcher(ignored));
    }

    public StrMatcher getTrimmerMatcher() {
        return this.trimmerMatcher;
    }

    public StrTokenizer setTrimmerMatcher(StrMatcher trimmer) {
        if (trimmer != null) {
            this.trimmerMatcher = trimmer;
        }
        return this;
    }

    public boolean isEmptyTokenAsNull() {
        return this.emptyAsNull;
    }

    public StrTokenizer setEmptyTokenAsNull(boolean emptyAsNull2) {
        this.emptyAsNull = emptyAsNull2;
        return this;
    }

    public boolean isIgnoreEmptyTokens() {
        return this.ignoreEmptyTokens;
    }

    public StrTokenizer setIgnoreEmptyTokens(boolean ignoreEmptyTokens2) {
        this.ignoreEmptyTokens = ignoreEmptyTokens2;
        return this;
    }

    public String getContent() {
        if (this.chars == null) {
            return null;
        }
        return new String(this.chars);
    }

    public Object clone() {
        try {
            return cloneReset();
        } catch (CloneNotSupportedException e) {
            return null;
        }
    }

    /* access modifiers changed from: package-private */
    public Object cloneReset() throws CloneNotSupportedException {
        StrTokenizer cloned = (StrTokenizer) super.clone();
        if (cloned.chars != null) {
            cloned.chars = (char[]) cloned.chars.clone();
        }
        cloned.reset();
        return cloned;
    }

    public String toString() {
        if (this.tokens == null) {
            return "StrTokenizer[not tokenized yet]";
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("StrTokenizer");
        stringBuffer.append(getTokenList());
        return stringBuffer.toString();
    }
}
