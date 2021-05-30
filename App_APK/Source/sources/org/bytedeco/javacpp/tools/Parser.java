package org.bytedeco.javacpp.tools;

import com.github.rosjava.android_remocons.common_tools.master.MasterDescription;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.apache.commons.io.IOUtils;

public class Parser {
    String[] docTags;
    final String encoding;
    InfoMap infoMap;
    InfoMap leafInfoMap;
    String lineSeparator;
    final Logger logger;
    final Properties properties;
    TokenIndexer tokens;

    public Parser(Logger logger2, Properties properties2) {
        this(logger2, properties2, (String) null, (String) null);
    }

    public Parser(Logger logger2, Properties properties2, String encoding2, String lineSeparator2) {
        this.infoMap = null;
        this.leafInfoMap = null;
        this.tokens = null;
        this.lineSeparator = null;
        this.docTags = new String[]{"author", "deprecated", MasterDescription.ERROR, "param", "return", "see", "since", "throws", "version"};
        this.logger = logger2;
        this.properties = properties2;
        this.encoding = encoding2;
        this.lineSeparator = lineSeparator2;
    }

    Parser(Parser p, String text) {
        this.infoMap = null;
        this.leafInfoMap = null;
        this.tokens = null;
        this.lineSeparator = null;
        this.docTags = new String[]{"author", "deprecated", MasterDescription.ERROR, "param", "return", "see", "since", "throws", "version"};
        this.logger = p.logger;
        this.properties = p.properties;
        this.encoding = p.encoding;
        this.infoMap = p.infoMap;
        Token t = p.tokens != null ? p.tokens.get() : Token.EOF;
        this.tokens = new TokenIndexer(this.infoMap, new Tokenizer(text, t.file, t.lineNumber).tokenize(), false);
        this.lineSeparator = p.lineSeparator;
    }

    /* access modifiers changed from: package-private */
    public String translate(String text) {
        Info info2;
        Info info = this.infoMap.getFirst(text);
        if (info != null && info.javaNames != null && info.javaNames.length > 0) {
            return info.javaNames[0];
        }
        int namespace = text.lastIndexOf("::");
        if (namespace >= 0) {
            Info info22 = this.infoMap.getFirst(text.substring(0, namespace));
            String localName = text.substring(namespace + 2);
            if (info22 != null && info22.pointerTypes != null) {
                text = info22.pointerTypes[0] + "." + localName;
            } else if (localName.length() > 0 && Character.isJavaIdentifierStart(localName.charAt(0))) {
                char[] charArray = localName.toCharArray();
                int length = charArray.length;
                int i = 0;
                while (true) {
                    if (i >= length) {
                        break;
                    } else if (!Character.isJavaIdentifierPart(charArray[i])) {
                        localName = null;
                        break;
                    } else {
                        i++;
                    }
                }
                if (localName != null) {
                    text = localName;
                }
            }
        }
        int castStart = text.lastIndexOf(40);
        int castEnd = text.indexOf(41, castStart);
        if (castStart < 0 || castStart >= castEnd || (info2 = this.infoMap.getFirst(text.substring(castStart + 1, castEnd))) == null || info2.valueTypes == null || info2.valueTypes.length <= 0) {
            return text;
        }
        return text.substring(0, castStart + 1) + info2.valueTypes[0] + text.substring(castEnd);
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:207:0x047b  */
    /* JADX WARNING: Removed duplicated region for block: B:208:0x047e  */
    /* JADX WARNING: Removed duplicated region for block: B:211:0x048b  */
    /* JADX WARNING: Removed duplicated region for block: B:221:0x050f  */
    /* JADX WARNING: Removed duplicated region for block: B:222:0x0512  */
    /* JADX WARNING: Removed duplicated region for block: B:225:0x0534  */
    /* JADX WARNING: Removed duplicated region for block: B:226:0x0537  */
    /* JADX WARNING: Removed duplicated region for block: B:229:0x055e  */
    /* JADX WARNING: Removed duplicated region for block: B:247:0x0673 A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:251:0x06a2 A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:276:0x0831  */
    /* JADX WARNING: Removed duplicated region for block: B:286:0x08f1  */
    /* JADX WARNING: Removed duplicated region for block: B:289:0x08f6  */
    /* JADX WARNING: Removed duplicated region for block: B:320:0x0a42  */
    /* JADX WARNING: Removed duplicated region for block: B:322:0x0a46  */
    /* JADX WARNING: Removed duplicated region for block: B:341:0x0beb  */
    /* JADX WARNING: Removed duplicated region for block: B:343:0x0bfb  */
    /* JADX WARNING: Removed duplicated region for block: B:376:0x0de0 A[ADDED_TO_REGION] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void containers(org.bytedeco.javacpp.tools.Context r48, org.bytedeco.javacpp.tools.DeclarationList r49) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r47 = this;
            r0 = r47
            java.util.ArrayList r1 = new java.util.ArrayList
            r1.<init>()
            org.bytedeco.javacpp.tools.InfoMap r2 = r0.infoMap
            java.lang.String r3 = "basic/containers"
            java.util.List r2 = r2.get(r3)
            java.util.Iterator r2 = r2.iterator()
        L_0x0013:
            boolean r3 = r2.hasNext()
            if (r3 == 0) goto L_0x0029
            java.lang.Object r3 = r2.next()
            org.bytedeco.javacpp.tools.Info r3 = (org.bytedeco.javacpp.tools.Info) r3
            java.lang.String[] r4 = r3.cppTypes
            java.util.List r4 = java.util.Arrays.asList(r4)
            r1.addAll(r4)
            goto L_0x0013
        L_0x0029:
            java.util.Iterator r2 = r1.iterator()
        L_0x002d:
            boolean r3 = r2.hasNext()
            if (r3 == 0) goto L_0x101a
            java.lang.Object r3 = r2.next()
            java.lang.String r3 = (java.lang.String) r3
            java.util.LinkedHashSet r4 = new java.util.LinkedHashSet
            r4.<init>()
            org.bytedeco.javacpp.tools.InfoMap r5 = r0.leafInfoMap
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "const "
            r6.append(r7)
            r6.append(r3)
            java.lang.String r6 = r6.toString()
            java.util.List r5 = r5.get(r6)
            r4.addAll(r5)
            org.bytedeco.javacpp.tools.InfoMap r5 = r0.leafInfoMap
            java.util.List r5 = r5.get(r3)
            r4.addAll(r5)
            java.util.Iterator r5 = r4.iterator()
        L_0x0065:
            boolean r6 = r5.hasNext()
            if (r6 == 0) goto L_0x1010
            java.lang.Object r6 = r5.next()
            org.bytedeco.javacpp.tools.Info r6 = (org.bytedeco.javacpp.tools.Info) r6
            org.bytedeco.javacpp.tools.Declaration r7 = new org.bytedeco.javacpp.tools.Declaration
            r7.<init>()
            if (r6 == 0) goto L_0x100a
            boolean r8 = r6.skip
            if (r8 != 0) goto L_0x100a
            boolean r8 = r6.define
            if (r8 != 0) goto L_0x0081
            goto L_0x0065
        L_0x0081:
            java.lang.String r8 = "pair"
            boolean r8 = r3.endsWith(r8)
            r9 = 1
            r8 = r8 ^ r9
            java.lang.String[] r10 = r6.cppNames
            r11 = 0
            r10 = r10[r11]
            java.lang.String r12 = "const "
            boolean r10 = r10.startsWith(r12)
            r12 = r10 ^ 1
            org.bytedeco.javacpp.tools.Parser r13 = new org.bytedeco.javacpp.tools.Parser
            java.lang.String[] r14 = r6.cppNames
            r14 = r14[r11]
            r13.<init>((org.bytedeco.javacpp.tools.Parser) r0, (java.lang.String) r14)
            r14 = r48
            org.bytedeco.javacpp.tools.Type r13 = r13.type(r14)
            r15 = 0
            r16 = 0
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            if (r9 == 0) goto L_0x100a
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            int r9 = r9.length
            if (r9 == 0) goto L_0x100a
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            r9 = r9[r11]
            if (r9 == 0) goto L_0x100a
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            org.bytedeco.javacpp.tools.Type[] r11 = r13.arguments
            int r11 = r11.length
            r18 = r1
            r1 = 1
            int r11 = r11 - r1
            r9 = r9[r11]
            if (r9 != 0) goto L_0x00c8
            r1 = r18
            goto L_0x0065
        L_0x00c8:
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            int r9 = r9.length
            if (r9 <= r1) goto L_0x00e5
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            r9 = r9[r1]
            java.lang.String r9 = r9.javaName
            int r9 = r9.length()
            if (r9 <= 0) goto L_0x00e5
            r9 = 0
            org.bytedeco.javacpp.tools.Type[] r11 = r13.arguments
            r12 = 0
            r11 = r11[r12]
            org.bytedeco.javacpp.tools.Type[] r12 = r13.arguments
            r12 = r12[r1]
            r1 = r12
            goto L_0x0106
        L_0x00e5:
            org.bytedeco.javacpp.tools.Type[] r9 = r13.arguments
            int r9 = r9.length
            if (r9 != r1) goto L_0x00ec
            r1 = 1
            goto L_0x00ed
        L_0x00ec:
            r1 = 0
        L_0x00ed:
            r9 = r12 & r1
            org.bytedeco.javacpp.tools.Type r1 = new org.bytedeco.javacpp.tools.Type
            r1.<init>()
            r11 = r1
            java.lang.String r1 = "@Cast(\"size_t\") "
            r11.annotations = r1
            java.lang.String r1 = "size_t"
            r11.cppName = r1
            java.lang.String r1 = "long"
            r11.javaName = r1
            org.bytedeco.javacpp.tools.Type[] r1 = r13.arguments
            r12 = 0
            r1 = r1[r12]
        L_0x0106:
            java.lang.String r12 = "(function = \"at\")"
            r19 = r9
            r20 = r2
            java.lang.String r2 = r1.javaName
            if (r2 == 0) goto L_0x0142
            java.lang.String r2 = r1.javaName
            int r2 = r2.length()
            if (r2 == 0) goto L_0x0142
            java.lang.String r2 = "bitset"
            boolean r2 = r3.endsWith(r2)
            if (r2 == 0) goto L_0x0121
            goto L_0x0142
        L_0x0121:
            java.lang.String r2 = "list"
            boolean r2 = r3.endsWith(r2)
            if (r2 != 0) goto L_0x0139
            java.lang.String r2 = "set"
            boolean r2 = r3.endsWith(r2)
            if (r2 == 0) goto L_0x0132
            goto L_0x0139
        L_0x0132:
            if (r10 != 0) goto L_0x0149
            if (r9 != 0) goto L_0x0149
            java.lang.String r12 = ""
            goto L_0x0149
        L_0x0139:
            r11 = 0
            r9 = 0
            java.lang.String r2 = "list"
            boolean r19 = r3.endsWith(r2)
            goto L_0x0149
        L_0x0142:
            java.lang.String r12 = ""
            java.lang.String r2 = "boolean"
            r1.javaName = r2
            r9 = 0
        L_0x0149:
            java.lang.String r2 = r1.cppName
            boolean r2 = r2.startsWith(r3)
            if (r2 == 0) goto L_0x016f
            org.bytedeco.javacpp.tools.InfoMap r2 = r0.leafInfoMap
            r21 = r4
            java.lang.String r4 = r1.cppName
            r22 = r5
            r5 = 0
            java.util.List r2 = r2.get((java.lang.String) r4, (boolean) r5)
            int r2 = r2.size()
            if (r2 != 0) goto L_0x0173
            int r8 = r8 + 1
            org.bytedeco.javacpp.tools.Type[] r2 = r1.arguments
            r1 = r2[r5]
            r4 = r21
            r5 = r22
            goto L_0x0149
        L_0x016f:
            r21 = r4
            r22 = r5
        L_0x0173:
            java.lang.String r2 = r1.cppName
            java.lang.String r4 = "<"
            int r2 = r2.indexOf(r4)
            java.lang.String r4 = "pair"
            boolean r4 = r3.endsWith(r4)
            if (r4 == 0) goto L_0x0191
            org.bytedeco.javacpp.tools.Type[] r4 = r13.arguments
            r5 = 0
            r15 = r4[r5]
            org.bytedeco.javacpp.tools.Type[] r4 = r13.arguments
            r17 = 1
            r16 = r4[r17]
        L_0x018e:
            r4 = r16
            goto L_0x01ad
        L_0x0191:
            r5 = 0
            if (r2 < 0) goto L_0x018e
            java.lang.String r4 = r1.cppName
            java.lang.String r4 = r4.substring(r5, r2)
            java.lang.String r5 = "pair"
            boolean r4 = r4.endsWith(r5)
            if (r4 == 0) goto L_0x018e
            org.bytedeco.javacpp.tools.Type[] r4 = r1.arguments
            r5 = 0
            r15 = r4[r5]
            org.bytedeco.javacpp.tools.Type[] r4 = r1.arguments
            r5 = 1
            r16 = r4[r5]
            goto L_0x018e
        L_0x01ad:
            if (r15 == 0) goto L_0x01e9
            java.lang.String r5 = r15.annotations
            if (r5 == 0) goto L_0x01bf
            java.lang.String r5 = r15.annotations
            int r5 = r5.length()
            if (r5 != 0) goto L_0x01bc
            goto L_0x01bf
        L_0x01bc:
            r24 = r2
            goto L_0x01eb
        L_0x01bf:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r24 = r2
            boolean r2 = r15.constValue
            if (r2 == 0) goto L_0x01cd
            java.lang.String r2 = "@Const "
            goto L_0x01cf
        L_0x01cd:
            java.lang.String r2 = ""
        L_0x01cf:
            r5.append(r2)
            int r2 = r15.indirections
            if (r2 != 0) goto L_0x01dd
            boolean r2 = r15.value
            if (r2 != 0) goto L_0x01dd
            java.lang.String r2 = "@ByRef "
            goto L_0x01df
        L_0x01dd:
            java.lang.String r2 = ""
        L_0x01df:
            r5.append(r2)
            java.lang.String r2 = r5.toString()
            r15.annotations = r2
            goto L_0x01eb
        L_0x01e9:
            r24 = r2
        L_0x01eb:
            if (r4 == 0) goto L_0x0220
            java.lang.String r2 = r4.annotations
            if (r2 == 0) goto L_0x01f9
            java.lang.String r2 = r4.annotations
            int r2 = r2.length()
            if (r2 != 0) goto L_0x0220
        L_0x01f9:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            boolean r5 = r4.constValue
            if (r5 == 0) goto L_0x0205
            java.lang.String r5 = "@Const "
            goto L_0x0207
        L_0x0205:
            java.lang.String r5 = ""
        L_0x0207:
            r2.append(r5)
            int r5 = r4.indirections
            if (r5 != 0) goto L_0x0215
            boolean r5 = r4.value
            if (r5 != 0) goto L_0x0215
            java.lang.String r5 = "@ByRef "
            goto L_0x0217
        L_0x0215:
            java.lang.String r5 = ""
        L_0x0217:
            r2.append(r5)
            java.lang.String r2 = r2.toString()
            r4.annotations = r2
        L_0x0220:
            if (r11 == 0) goto L_0x0255
            java.lang.String r2 = r11.annotations
            if (r2 == 0) goto L_0x022e
            java.lang.String r2 = r11.annotations
            int r2 = r2.length()
            if (r2 != 0) goto L_0x0255
        L_0x022e:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            boolean r5 = r11.constValue
            if (r5 == 0) goto L_0x023a
            java.lang.String r5 = "@Const "
            goto L_0x023c
        L_0x023a:
            java.lang.String r5 = ""
        L_0x023c:
            r2.append(r5)
            int r5 = r11.indirections
            if (r5 != 0) goto L_0x024a
            boolean r5 = r11.value
            if (r5 != 0) goto L_0x024a
            java.lang.String r5 = "@ByRef "
            goto L_0x024c
        L_0x024a:
            java.lang.String r5 = ""
        L_0x024c:
            r2.append(r5)
            java.lang.String r2 = r2.toString()
            r11.annotations = r2
        L_0x0255:
            if (r1 == 0) goto L_0x028a
            java.lang.String r2 = r1.annotations
            if (r2 == 0) goto L_0x0263
            java.lang.String r2 = r1.annotations
            int r2 = r2.length()
            if (r2 != 0) goto L_0x028a
        L_0x0263:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            boolean r5 = r1.constValue
            if (r5 == 0) goto L_0x026f
            java.lang.String r5 = "@Const "
            goto L_0x0271
        L_0x026f:
            java.lang.String r5 = ""
        L_0x0271:
            r2.append(r5)
            int r5 = r1.indirections
            if (r5 != 0) goto L_0x027f
            boolean r5 = r1.value
            if (r5 != 0) goto L_0x027f
            java.lang.String r5 = "@ByRef "
            goto L_0x0281
        L_0x027f:
            java.lang.String r5 = ""
        L_0x0281:
            r2.append(r5)
            java.lang.String r2 = r2.toString()
            r1.annotations = r2
        L_0x028a:
            org.bytedeco.javacpp.tools.InfoMap r2 = r0.infoMap
            java.lang.String r5 = r1.cppName
            org.bytedeco.javacpp.tools.Info r2 = r2.getFirst(r5)
            if (r2 == 0) goto L_0x0343
            boolean r5 = r2.cast
            if (r5 == 0) goto L_0x0343
            java.lang.String r5 = r1.cppName
            boolean r0 = r1.constValue
            if (r0 == 0) goto L_0x02ba
            java.lang.String r0 = "const "
            boolean r0 = r5.startsWith(r0)
            if (r0 != 0) goto L_0x02ba
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r25 = r2
            java.lang.String r2 = "const "
            r0.append(r2)
            r0.append(r5)
            java.lang.String r5 = r0.toString()
            goto L_0x02bc
        L_0x02ba:
            r25 = r2
        L_0x02bc:
            boolean r0 = r1.constPointer
            if (r0 == 0) goto L_0x02d9
            java.lang.String r0 = " const"
            boolean r0 = r5.endsWith(r0)
            if (r0 != 0) goto L_0x02d9
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r5)
            java.lang.String r2 = " const"
            r0.append(r2)
            java.lang.String r5 = r0.toString()
        L_0x02d9:
            int r0 = r1.indirections
            if (r0 <= 0) goto L_0x02fb
            r0 = 0
        L_0x02de:
            int r2 = r1.indirections
            if (r0 >= r2) goto L_0x02f8
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r2.append(r5)
            r26 = r5
            java.lang.String r5 = "*"
            r2.append(r5)
            java.lang.String r5 = r2.toString()
            int r0 = r0 + 1
            goto L_0x02de
        L_0x02f8:
            r26 = r5
            goto L_0x0310
        L_0x02fb:
            boolean r0 = r1.value
            if (r0 != 0) goto L_0x0310
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r5)
            java.lang.String r2 = "*"
            r0.append(r2)
            java.lang.String r5 = r0.toString()
        L_0x0310:
            boolean r0 = r1.reference
            if (r0 == 0) goto L_0x0325
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r5)
            java.lang.String r2 = "&"
            r0.append(r2)
            java.lang.String r5 = r0.toString()
        L_0x0325:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r2 = "@Cast(\""
            r0.append(r2)
            r0.append(r5)
            java.lang.String r2 = "\") "
            r0.append(r2)
            java.lang.String r2 = r1.annotations
            r0.append(r2)
            java.lang.String r0 = r0.toString()
            r1.annotations = r0
            goto L_0x0345
        L_0x0343:
            r25 = r2
        L_0x0345:
            java.lang.String r0 = ""
            r2 = r0
            r0 = 0
        L_0x0349:
            int r5 = r8 + -1
            if (r0 >= r5) goto L_0x0365
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r2)
            r27 = r6
            java.lang.String r6 = "[]"
            r5.append(r6)
            java.lang.String r2 = r5.toString()
            int r0 = r0 + 1
            r6 = r27
            goto L_0x0349
        L_0x0365:
            r27 = r6
            org.bytedeco.javacpp.tools.Type r0 = new org.bytedeco.javacpp.tools.Type
            java.lang.String r5 = r13.javaName
            r0.<init>(r5)
            r7.type = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r5 = r7.text
            r0.append(r5)
            if (r8 != 0) goto L_0x037f
            java.lang.String r5 = "\n@NoOffset "
            goto L_0x0381
        L_0x037f:
            java.lang.String r5 = "\n"
        L_0x0381:
            r0.append(r5)
            java.lang.String r5 = "@Name(\""
            r0.append(r5)
            java.lang.String r5 = r13.cppName
            r0.append(r5)
            java.lang.String r5 = "\") public static class "
            r0.append(r5)
            java.lang.String r5 = r13.javaName
            r0.append(r5)
            java.lang.String r5 = " extends Pointer {\n    static { Loader.load(); }\n    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */\n    public "
            r0.append(r5)
            java.lang.String r5 = r13.javaName
            r0.append(r5)
            java.lang.String r5 = "(Pointer p) { super(p); }\n"
            r0.append(r5)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            if (r10 != 0) goto L_0x046d
            if (r8 == 0) goto L_0x03be
            org.bytedeco.javacpp.tools.Type[] r5 = r13.arguments
            int r5 = r5.length
            r6 = 1
            if (r5 != r6) goto L_0x03b8
            goto L_0x03be
        L_0x03b8:
            r28 = r3
            r29 = r12
            goto L_0x0471
        L_0x03be:
            if (r15 == 0) goto L_0x046d
            if (r4 == 0) goto L_0x046d
            java.lang.String[] r5 = r15.javaNames
            if (r5 == 0) goto L_0x03c9
            java.lang.String[] r5 = r15.javaNames
            goto L_0x03d3
        L_0x03c9:
            r5 = 1
            java.lang.String[] r6 = new java.lang.String[r5]
            java.lang.String r5 = r15.javaName
            r16 = 0
            r6[r16] = r5
            r5 = r6
        L_0x03d3:
            java.lang.String[] r6 = r4.javaNames
            if (r6 == 0) goto L_0x03db
            java.lang.String[] r6 = r4.javaNames
            r0 = r6
            goto L_0x03e4
        L_0x03db:
            r6 = 1
            java.lang.String[] r0 = new java.lang.String[r6]
            java.lang.String r6 = r4.javaName
            r16 = 0
            r0[r16] = r6
        L_0x03e4:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r2)
            if (r8 <= 0) goto L_0x03f3
            java.lang.String r16 = "[]"
        L_0x03f0:
            r14 = r16
            goto L_0x03f6
        L_0x03f3:
            java.lang.String r16 = ""
            goto L_0x03f0
        L_0x03f6:
            r6.append(r14)
            java.lang.String r6 = r6.toString()
            r14 = 0
        L_0x03fe:
            r28 = r3
            int r3 = r5.length
            if (r14 < r3) goto L_0x040c
            int r3 = r0.length
            if (r14 >= r3) goto L_0x0407
            goto L_0x040c
        L_0x0407:
            r29 = r12
            goto L_0x04f4
        L_0x040c:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r29 = r12
            java.lang.String r12 = r7.text
            r3.append(r12)
            java.lang.String r12 = "    public "
            r3.append(r12)
            java.lang.String r12 = r13.javaName
            r3.append(r12)
            java.lang.String r12 = "("
            r3.append(r12)
            int r12 = r5.length
            r16 = 1
            int r12 = r12 + -1
            int r12 = java.lang.Math.min(r14, r12)
            r12 = r5[r12]
            r3.append(r12)
            r3.append(r6)
            java.lang.String r12 = " firstValue, "
            r3.append(r12)
            int r12 = r0.length
            int r12 = r12 + -1
            int r12 = java.lang.Math.min(r14, r12)
            r12 = r0[r12]
            r3.append(r12)
            r3.append(r6)
            java.lang.String r12 = " secondValue) { this("
            r3.append(r12)
            if (r8 <= 0) goto L_0x0456
            java.lang.String r12 = "Math.min(firstValue.length, secondValue.length)"
            goto L_0x0458
        L_0x0456:
            java.lang.String r12 = ""
        L_0x0458:
            r3.append(r12)
            java.lang.String r12 = "); put(firstValue, secondValue); }\n"
            r3.append(r12)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            int r14 = r14 + 1
            r3 = r28
            r12 = r29
            goto L_0x03fe
        L_0x046d:
            r28 = r3
            r29 = r12
        L_0x0471:
            if (r9 == 0) goto L_0x04f4
            if (r15 != 0) goto L_0x04f4
            if (r4 != 0) goto L_0x04f4
            java.lang.String[] r0 = r1.javaNames
            if (r0 == 0) goto L_0x047e
            java.lang.String[] r0 = r1.javaNames
            goto L_0x0487
        L_0x047e:
            r0 = 1
            java.lang.String[] r3 = new java.lang.String[r0]
            java.lang.String r0 = r1.javaName
            r5 = 0
            r3[r5] = r0
            r0 = r3
        L_0x0487:
            int r3 = r0.length
            r5 = 0
        L_0x0489:
            if (r5 >= r3) goto L_0x04f4
            r6 = r0[r5]
            r12 = 2
            if (r8 >= r12) goto L_0x04c7
            java.lang.String r12 = "int"
            boolean r12 = r6.equals(r12)
            if (r12 != 0) goto L_0x04c7
            java.lang.String r12 = "long"
            boolean r12 = r6.equals(r12)
            if (r12 != 0) goto L_0x04c7
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r14 = r7.text
            r12.append(r14)
            java.lang.String r14 = "    public "
            r12.append(r14)
            java.lang.String r14 = r13.javaName
            r12.append(r14)
            java.lang.String r14 = "("
            r12.append(r14)
            r12.append(r6)
            java.lang.String r14 = " value) { this(1); put(0, value); }\n"
            r12.append(r14)
            java.lang.String r12 = r12.toString()
            r7.text = r12
        L_0x04c7:
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            java.lang.String r14 = r7.text
            r12.append(r14)
            java.lang.String r14 = "    public "
            r12.append(r14)
            java.lang.String r14 = r13.javaName
            r12.append(r14)
            java.lang.String r14 = "("
            r12.append(r14)
            r12.append(r6)
            r12.append(r2)
            java.lang.String r14 = " ... array) { this(array.length); put(array); }\n"
            r12.append(r14)
            java.lang.String r12 = r12.toString()
            r7.text = r12
            int r5 = r5 + 1
            goto L_0x0489
        L_0x04f4:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "    public "
            r0.append(r3)
            java.lang.String r3 = r13.javaName
            r0.append(r3)
            java.lang.String r3 = "()       { allocate();  }\n"
            r0.append(r3)
            if (r9 != 0) goto L_0x0512
            java.lang.String r3 = ""
            goto L_0x052a
        L_0x0512:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = "    public "
            r3.append(r5)
            java.lang.String r5 = r13.javaName
            r3.append(r5)
            java.lang.String r5 = "(long n) { allocate(n); }\n"
            r3.append(r5)
            java.lang.String r3 = r3.toString()
        L_0x052a:
            r0.append(r3)
            java.lang.String r3 = "    private native void allocate();\n"
            r0.append(r3)
            if (r9 != 0) goto L_0x0537
            java.lang.String r3 = ""
            goto L_0x0539
        L_0x0537:
            java.lang.String r3 = "    private native void allocate(@Cast(\"size_t\") long n);\n"
        L_0x0539:
            r0.append(r3)
            java.lang.String r3 = "    public native @Name(\"operator=\") @ByRef "
            r0.append(r3)
            java.lang.String r3 = r13.javaName
            r0.append(r3)
            java.lang.String r3 = " put(@ByRef "
            r0.append(r3)
            java.lang.String r3 = r13.javaName
            r0.append(r3)
            java.lang.String r3 = " x);\n\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            r0 = 0
        L_0x055c:
            if (r0 >= r8) goto L_0x0666
            if (r0 <= 0) goto L_0x0593
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = "@Index("
            r3.append(r5)
            r5 = 1
            if (r0 <= r5) goto L_0x0584
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "value = "
            r5.append(r6)
            r5.append(r0)
            java.lang.String r6 = ", "
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            goto L_0x0586
        L_0x0584:
            java.lang.String r5 = ""
        L_0x0586:
            r3.append(r5)
            java.lang.String r5 = "function = \"at\") "
            r3.append(r5)
            java.lang.String r3 = r3.toString()
            goto L_0x0595
        L_0x0593:
            java.lang.String r3 = ""
        L_0x0595:
            java.lang.String r5 = ""
            java.lang.String r6 = ""
            java.lang.String r12 = ""
            r14 = r12
            r12 = r6
            r6 = r5
            r5 = 0
        L_0x059f:
            if (r11 == 0) goto L_0x05e9
            if (r5 >= r0) goto L_0x05e9
            r30 = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r2.append(r6)
            r2.append(r14)
            r31 = r1
            java.lang.String r1 = r11.annotations
            r2.append(r1)
            java.lang.String r1 = r11.javaName
            r2.append(r1)
            java.lang.String r1 = " "
            r2.append(r1)
            int r1 = r5 + 105
            char r1 = (char) r1
            r2.append(r1)
            java.lang.String r6 = r2.toString()
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r12)
            r1.append(r14)
            int r2 = r5 + 105
            char r2 = (char) r2
            r1.append(r2)
            java.lang.String r12 = r1.toString()
            java.lang.String r14 = ", "
            int r5 = r5 + 1
            r2 = r30
            r1 = r31
            goto L_0x059f
        L_0x05e9:
            r31 = r1
            r30 = r2
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = r7.text
            r1.append(r2)
            java.lang.String r2 = "    public boolean empty("
            r1.append(r2)
            r1.append(r6)
            java.lang.String r2 = ") { return size("
            r1.append(r2)
            r1.append(r12)
            java.lang.String r2 = ") == 0; }\n    public native "
            r1.append(r2)
            r1.append(r3)
            java.lang.String r2 = "long size("
            r1.append(r2)
            r1.append(r6)
            java.lang.String r2 = ");\n"
            r1.append(r2)
            if (r9 != 0) goto L_0x0621
            java.lang.String r2 = ""
            goto L_0x0655
        L_0x0621:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r5 = "    public void clear("
            r2.append(r5)
            r2.append(r6)
            java.lang.String r5 = ") { resize("
            r2.append(r5)
            r2.append(r12)
            r2.append(r14)
            java.lang.String r5 = "0); }\n    public native "
            r2.append(r5)
            r2.append(r3)
            java.lang.String r5 = "void resize("
            r2.append(r5)
            r2.append(r6)
            r2.append(r14)
            java.lang.String r5 = "@Cast(\"size_t\") long n);\n"
            r2.append(r5)
            java.lang.String r2 = r2.toString()
        L_0x0655:
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r7.text = r1
            int r0 = r0 + 1
            r2 = r30
            r1 = r31
            goto L_0x055c
        L_0x0666:
            r31 = r1
            r30 = r2
            java.lang.String r0 = ""
            java.lang.String r1 = ""
            r2 = r1
            r1 = r0
            r0 = 0
        L_0x0671:
            if (r11 == 0) goto L_0x069e
            if (r0 >= r8) goto L_0x069e
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r1)
            r3.append(r2)
            java.lang.String r5 = r11.annotations
            r3.append(r5)
            java.lang.String r5 = r11.javaName
            r3.append(r5)
            java.lang.String r5 = " "
            r3.append(r5)
            int r5 = r0 + 105
            char r5 = (char) r5
            r3.append(r5)
            java.lang.String r1 = r3.toString()
            java.lang.String r2 = ", "
            int r0 = r0 + 1
            goto L_0x0671
        L_0x069e:
            r3 = 32
            if (r15 == 0) goto L_0x082d
            if (r4 == 0) goto L_0x082d
            if (r8 != 0) goto L_0x06a9
            java.lang.String r5 = "@MemberGetter "
            goto L_0x06db
        L_0x06a9:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "@Index("
            r5.append(r6)
            r6 = 1
            if (r8 <= r6) goto L_0x06cd
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "value = "
            r6.append(r12)
            r6.append(r8)
            java.lang.String r12 = ", "
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            goto L_0x06cf
        L_0x06cd:
            java.lang.String r6 = ""
        L_0x06cf:
            r5.append(r6)
            java.lang.String r6 = "function = \"at\") "
            r5.append(r6)
            java.lang.String r5 = r5.toString()
        L_0x06db:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = r7.text
            r6.append(r12)
            java.lang.String r12 = "\n    "
            r6.append(r12)
            r6.append(r5)
            java.lang.String r12 = "public native "
            r6.append(r12)
            java.lang.String r12 = r15.annotations
            r6.append(r12)
            java.lang.String r12 = r15.javaName
            r6.append(r12)
            java.lang.String r12 = " first("
            r6.append(r12)
            r6.append(r1)
            java.lang.String r12 = "); public native "
            r6.append(r12)
            java.lang.String r12 = r13.javaName
            r6.append(r12)
            java.lang.String r12 = " first("
            r6.append(r12)
            r6.append(r1)
            r6.append(r2)
            java.lang.String r12 = r15.javaName
            java.lang.String r14 = r15.javaName
            int r14 = r14.lastIndexOf(r3)
            r16 = 1
            int r14 = r14 + 1
            java.lang.String r12 = r12.substring(r14)
            r6.append(r12)
            java.lang.String r12 = " first);\n    "
            r6.append(r12)
            r6.append(r5)
            java.lang.String r12 = "public native "
            r6.append(r12)
            java.lang.String r12 = r4.annotations
            r6.append(r12)
            java.lang.String r12 = r4.javaName
            r6.append(r12)
            java.lang.String r12 = " second("
            r6.append(r12)
            r6.append(r1)
            java.lang.String r12 = ");  public native "
            r6.append(r12)
            java.lang.String r12 = r13.javaName
            r6.append(r12)
            java.lang.String r12 = " second("
            r6.append(r12)
            r6.append(r1)
            r6.append(r2)
            java.lang.String r12 = r4.javaName
            java.lang.String r14 = r4.javaName
            int r3 = r14.lastIndexOf(r3)
            r14 = 1
            int r3 = r3 + r14
            java.lang.String r3 = r12.substring(r3)
            r6.append(r3)
            java.lang.String r3 = " second);\n"
            r6.append(r3)
            java.lang.String r3 = r6.toString()
            r7.text = r3
            r3 = 1
        L_0x077d:
            if (r10 != 0) goto L_0x07cb
            java.lang.String[] r6 = r15.javaNames
            if (r6 == 0) goto L_0x07cb
            java.lang.String[] r6 = r15.javaNames
            int r6 = r6.length
            if (r3 >= r6) goto L_0x07cb
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = r7.text
            r6.append(r12)
            java.lang.String r12 = "    @MemberSetter @Index"
            r6.append(r12)
            r12 = r29
            r6.append(r12)
            java.lang.String r14 = " public native "
            r6.append(r14)
            java.lang.String r14 = r13.javaName
            r6.append(r14)
            java.lang.String r14 = " first("
            r6.append(r14)
            r6.append(r1)
            r6.append(r2)
            java.lang.String r14 = r15.annotations
            r6.append(r14)
            java.lang.String[] r14 = r15.javaNames
            r14 = r14[r3]
            r6.append(r14)
            java.lang.String r14 = " first);\n"
            r6.append(r14)
            java.lang.String r6 = r6.toString()
            r7.text = r6
            int r3 = r3 + 1
            goto L_0x077d
        L_0x07cb:
            r12 = r29
            r3 = 1
        L_0x07ce:
            if (r10 != 0) goto L_0x081a
            java.lang.String[] r6 = r4.javaNames
            if (r6 == 0) goto L_0x081a
            java.lang.String[] r6 = r4.javaNames
            int r6 = r6.length
            if (r3 >= r6) goto L_0x081a
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r14 = r7.text
            r6.append(r14)
            java.lang.String r14 = "    @MemberSetter @Index"
            r6.append(r14)
            r6.append(r12)
            java.lang.String r14 = " public native "
            r6.append(r14)
            java.lang.String r14 = r13.javaName
            r6.append(r14)
            java.lang.String r14 = " second("
            r6.append(r14)
            r6.append(r1)
            r6.append(r2)
            java.lang.String r14 = r4.annotations
            r6.append(r14)
            java.lang.String[] r14 = r4.javaNames
            r14 = r14[r3]
            r6.append(r14)
            java.lang.String r14 = " second);\n"
            r6.append(r14)
            java.lang.String r6 = r6.toString()
            r7.text = r6
            int r3 = r3 + 1
            goto L_0x07ce
        L_0x081a:
            r32 = r1
            r38 = r4
            r36 = r9
            r34 = r11
            r35 = r12
            r33 = r28
            r37 = r30
            r6 = r31
            goto L_0x0bf9
        L_0x082d:
            r12 = r29
            if (r11 == 0) goto L_0x08f1
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r7.text
            r5.append(r6)
            java.lang.String r6 = "\n    @Index"
            r5.append(r6)
            r5.append(r12)
            java.lang.String r6 = " public native "
            r5.append(r6)
            r6 = r31
            java.lang.String r14 = r6.annotations
            r5.append(r14)
            java.lang.String r14 = r6.javaName
            r5.append(r14)
            java.lang.String r14 = " get("
            r5.append(r14)
            r5.append(r1)
            java.lang.String r14 = ");\n"
            r5.append(r14)
            java.lang.String r5 = r5.toString()
            r7.text = r5
            if (r10 != 0) goto L_0x08a4
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r14 = r7.text
            r5.append(r14)
            java.lang.String r14 = "    public native "
            r5.append(r14)
            java.lang.String r14 = r13.javaName
            r5.append(r14)
            java.lang.String r14 = " put("
            r5.append(r14)
            r5.append(r1)
            r5.append(r2)
            java.lang.String r14 = r6.javaName
            java.lang.String r0 = r6.javaName
            int r0 = r0.lastIndexOf(r3)
            r3 = 1
            int r0 = r0 + r3
            java.lang.String r0 = r14.substring(r0)
            r5.append(r0)
            java.lang.String r0 = " value);\n"
            r5.append(r0)
            java.lang.String r0 = r5.toString()
            r7.text = r0
        L_0x08a4:
            r0 = 1
        L_0x08a5:
            if (r10 != 0) goto L_0x08f3
            java.lang.String[] r3 = r6.javaNames
            if (r3 == 0) goto L_0x08f3
            java.lang.String[] r3 = r6.javaNames
            int r3 = r3.length
            if (r0 >= r3) goto L_0x08f3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = r7.text
            r3.append(r5)
            java.lang.String r5 = "    @ValueSetter @Index"
            r3.append(r5)
            r3.append(r12)
            java.lang.String r5 = " public native "
            r3.append(r5)
            java.lang.String r5 = r13.javaName
            r3.append(r5)
            java.lang.String r5 = " put("
            r3.append(r5)
            r3.append(r1)
            r3.append(r2)
            java.lang.String r5 = r6.annotations
            r3.append(r5)
            java.lang.String[] r5 = r6.javaNames
            r5 = r5[r0]
            r3.append(r5)
            java.lang.String r5 = " value);\n"
            r3.append(r5)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            int r0 = r0 + 1
            goto L_0x08a5
        L_0x08f1:
            r6 = r31
        L_0x08f3:
            r0 = 1
            if (r8 != r0) goto L_0x0a42
            java.lang.String r3 = "bitset"
            r5 = r28
            boolean r3 = r5.endsWith(r3)
            if (r3 != 0) goto L_0x0a44
            org.bytedeco.javacpp.tools.Type[] r3 = r13.arguments
            int r3 = r3.length
            if (r3 < r0) goto L_0x0a44
            org.bytedeco.javacpp.tools.Type[] r3 = r13.arguments
            org.bytedeco.javacpp.tools.Type[] r14 = r13.arguments
            int r14 = r14.length
            int r14 = r14 - r0
            r0 = r3[r14]
            java.lang.String r0 = r0.javaName
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x0a44
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            if (r10 != 0) goto L_0x0988
            if (r19 == 0) goto L_0x0953
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "    public native @ByVal Iterator insert(@ByVal Iterator pos, "
            r0.append(r3)
            java.lang.String r3 = r6.annotations
            r0.append(r3)
            java.lang.String r3 = r6.javaName
            r0.append(r3)
            java.lang.String r3 = " value);\n    public native @ByVal Iterator erase(@ByVal Iterator pos);\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            goto L_0x0988
        L_0x0953:
            if (r11 != 0) goto L_0x0988
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "    public native void insert("
            r0.append(r3)
            java.lang.String r3 = r6.annotations
            r0.append(r3)
            java.lang.String r3 = r6.javaName
            r0.append(r3)
            java.lang.String r3 = " value);\n    public native void erase("
            r0.append(r3)
            java.lang.String r3 = r6.annotations
            r0.append(r3)
            java.lang.String r3 = r6.javaName
            r0.append(r3)
            java.lang.String r3 = " value);\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
        L_0x0988:
            if (r11 == 0) goto L_0x09b7
            java.lang.String r0 = r11.annotations
            java.lang.String r3 = "@Const"
            boolean r0 = r0.contains(r3)
            if (r0 != 0) goto L_0x09b7
            java.lang.String r0 = r11.annotations
            java.lang.String r3 = "@Cast"
            boolean r0 = r0.contains(r3)
            if (r0 != 0) goto L_0x09b7
            boolean r0 = r11.value
            if (r0 != 0) goto L_0x09b7
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r11.annotations
            r0.append(r3)
            java.lang.String r3 = "@Const "
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r11.annotations = r0
        L_0x09b7:
            java.lang.String r0 = r6.annotations
            java.lang.String r3 = "@Const"
            boolean r0 = r0.contains(r3)
            if (r0 != 0) goto L_0x09da
            boolean r0 = r6.value
            if (r0 != 0) goto L_0x09da
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r6.annotations
            r0.append(r3)
            java.lang.String r3 = "@Const "
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r6.annotations = r0
        L_0x09da:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "    public native @ByVal Iterator begin();\n    public native @ByVal Iterator end();\n    @NoOffset @Name(\"iterator\") public static class Iterator extends Pointer {\n        public Iterator(Pointer p) { super(p); }\n        public Iterator() { }\n\n        public native @Name(\"operator++\") @ByRef Iterator increment();\n        public native @Name(\"operator==\") boolean equals(@ByRef Iterator it);\n"
            r0.append(r3)
            org.bytedeco.javacpp.tools.Type[] r3 = r13.arguments
            int r3 = r3.length
            r14 = 1
            if (r3 <= r14) goto L_0x0a1c
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r14 = "        public native @Name(\"operator*().first\") @MemberGetter "
            r3.append(r14)
            java.lang.String r14 = r11.annotations
            r3.append(r14)
            java.lang.String r14 = r11.javaName
            r3.append(r14)
            java.lang.String r14 = " first();\n        public native @Name(\"operator*().second\") @MemberGetter "
            r3.append(r14)
            java.lang.String r14 = r6.annotations
            r3.append(r14)
            java.lang.String r14 = r6.javaName
            r3.append(r14)
            java.lang.String r14 = " second();\n"
        L_0x0a14:
            r3.append(r14)
            java.lang.String r3 = r3.toString()
            goto L_0x0a33
        L_0x0a1c:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r14 = "        public native @Name(\"operator*\") "
            r3.append(r14)
            java.lang.String r14 = r6.annotations
            r3.append(r14)
            java.lang.String r14 = r6.javaName
            r3.append(r14)
            java.lang.String r14 = " get();\n"
            goto L_0x0a14
        L_0x0a33:
            r0.append(r3)
            java.lang.String r3 = "    }\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            goto L_0x0a44
        L_0x0a42:
            r5 = r28
        L_0x0a44:
            if (r9 == 0) goto L_0x0beb
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "\n    public "
            r0.append(r3)
            java.lang.String r3 = r6.javaName
            r0.append(r3)
            r3 = r30
            r0.append(r3)
            java.lang.String r14 = "[] get() {\n"
            r0.append(r14)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            java.lang.String r0 = "        "
            java.lang.String r14 = ""
            java.lang.String r16 = ""
            r23 = r3
            java.lang.String r2 = ""
            r32 = r1
            r33 = r5
            r34 = r11
            r1 = r16
            r5 = r23
            r11 = r2
            r2 = r0
            r0 = 0
        L_0x0a81:
            if (r0 >= r8) goto L_0x0b71
            r35 = r12
            int r12 = r0 + 105
            char r12 = (char) r12
            r36 = r9
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r37 = r3
            java.lang.String r3 = r7.text
            r9.append(r3)
            r9.append(r2)
            if (r0 != 0) goto L_0x0ab4
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r38 = r4
            java.lang.String r4 = r6.javaName
            r3.append(r4)
            r3.append(r5)
            java.lang.String r4 = "[] "
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            goto L_0x0ab8
        L_0x0ab4:
            r38 = r4
            java.lang.String r3 = ""
        L_0x0ab8:
            r9.append(r3)
            java.lang.String r3 = "array"
            r9.append(r3)
            r9.append(r14)
            java.lang.String r3 = " = new "
            r9.append(r3)
            java.lang.String r3 = r6.javaName
            r9.append(r3)
            java.lang.String r3 = "[size("
            r9.append(r3)
            r9.append(r1)
            java.lang.String r3 = ") < Integer.MAX_VALUE ? (int)size("
            r9.append(r3)
            r9.append(r1)
            java.lang.String r3 = ") : Integer.MAX_VALUE]"
            r9.append(r3)
            r9.append(r5)
            java.lang.String r3 = ";\n"
            r9.append(r3)
            r9.append(r2)
            java.lang.String r3 = "for (int "
            r9.append(r3)
            r9.append(r12)
            java.lang.String r3 = " = 0; "
            r9.append(r3)
            r9.append(r12)
            java.lang.String r3 = " < array"
            r9.append(r3)
            r9.append(r14)
            java.lang.String r3 = ".length; "
            r9.append(r3)
            r9.append(r12)
            java.lang.String r3 = "++) {\n"
            r9.append(r3)
            java.lang.String r3 = r9.toString()
            r7.text = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r2)
            java.lang.String r4 = "    "
            r3.append(r4)
            java.lang.String r2 = r3.toString()
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r14)
            java.lang.String r4 = "["
            r3.append(r4)
            r3.append(r12)
            java.lang.String r4 = "]"
            r3.append(r4)
            java.lang.String r14 = r3.toString()
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r1)
            r3.append(r11)
            r3.append(r12)
            java.lang.String r1 = r3.toString()
            int r3 = r5.length()
            r4 = 2
            if (r3 >= r4) goto L_0x0b5e
            java.lang.String r3 = ""
            goto L_0x0b62
        L_0x0b5e:
            java.lang.String r3 = r5.substring(r4)
        L_0x0b62:
            r5 = r3
            java.lang.String r11 = ", "
            int r0 = r0 + 1
            r12 = r35
            r9 = r36
            r3 = r37
            r4 = r38
            goto L_0x0a81
        L_0x0b71:
            r37 = r3
            r38 = r4
            r36 = r9
            r35 = r12
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            r0.append(r2)
            java.lang.String r3 = "array"
            r0.append(r3)
            r0.append(r14)
            java.lang.String r3 = " = get("
            r0.append(r3)
            r0.append(r1)
            java.lang.String r3 = ");\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            r0 = 0
        L_0x0ba2:
            if (r0 >= r8) goto L_0x0bc4
            r3 = 4
            java.lang.String r2 = r2.substring(r3)
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r7.text
            r3.append(r4)
            r3.append(r2)
            java.lang.String r4 = "}\n"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            int r0 = r0 + 1
            goto L_0x0ba2
        L_0x0bc4:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r7.text
            r0.append(r3)
            java.lang.String r3 = "        return array;\n    }\n    @Override public String toString() {\n        return java.util.Arrays."
            r0.append(r3)
            r3 = 2
            if (r8 >= r3) goto L_0x0bd9
            java.lang.String r3 = "toString"
            goto L_0x0bdb
        L_0x0bd9:
            java.lang.String r3 = "deepToString"
        L_0x0bdb:
            r0.append(r3)
            java.lang.String r3 = "(get());\n    }\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            r2 = r11
            goto L_0x0bf9
        L_0x0beb:
            r32 = r1
            r38 = r4
            r33 = r5
            r36 = r9
            r34 = r11
            r35 = r12
            r37 = r30
        L_0x0bf9:
            if (r10 != 0) goto L_0x0dd8
            if (r8 == 0) goto L_0x0c0c
            org.bytedeco.javacpp.tools.Type[] r0 = r13.arguments
            int r0 = r0.length
            r1 = 1
            if (r0 != r1) goto L_0x0c04
            goto L_0x0c0c
        L_0x0c04:
            r42 = r10
            r5 = r37
            r1 = r38
            goto L_0x0dde
        L_0x0c0c:
            if (r15 == 0) goto L_0x0dd8
            if (r38 == 0) goto L_0x0dd8
            java.lang.String[] r0 = r15.javaNames
            if (r0 == 0) goto L_0x0c17
            java.lang.String[] r0 = r15.javaNames
            goto L_0x0c20
        L_0x0c17:
            r0 = 1
            java.lang.String[] r1 = new java.lang.String[r0]
            java.lang.String r0 = r15.javaName
            r3 = 0
            r1[r3] = r0
            r0 = r1
        L_0x0c20:
            r1 = r38
            java.lang.String[] r3 = r1.javaNames
            if (r3 == 0) goto L_0x0c29
            java.lang.String[] r3 = r1.javaNames
            goto L_0x0c32
        L_0x0c29:
            r3 = 1
            java.lang.String[] r4 = new java.lang.String[r3]
            java.lang.String r3 = r1.javaName
            r5 = 0
            r4[r5] = r3
            r3 = r4
        L_0x0c32:
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r5 = r37
            r4.append(r5)
            if (r8 <= 0) goto L_0x0c41
            java.lang.String r9 = "[]"
            goto L_0x0c43
        L_0x0c41:
            java.lang.String r9 = ""
        L_0x0c43:
            r4.append(r9)
            java.lang.String r4 = r4.toString()
            r9 = r2
            r2 = 0
        L_0x0c4c:
            int r11 = r0.length
            if (r2 < r11) goto L_0x0c5a
            int r11 = r3.length
            if (r2 >= r11) goto L_0x0c53
            goto L_0x0c5a
        L_0x0c53:
            r43 = r1
            r42 = r10
            goto L_0x0fe1
        L_0x0c5a:
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r12 = r7.text
            r11.append(r12)
            java.lang.String r12 = "\n    public "
            r11.append(r12)
            java.lang.String r12 = r13.javaName
            r11.append(r12)
            java.lang.String r12 = " put("
            r11.append(r12)
            int r12 = r0.length
            r14 = 1
            int r12 = r12 - r14
            int r12 = java.lang.Math.min(r2, r12)
            r12 = r0[r12]
            r11.append(r12)
            r11.append(r4)
            java.lang.String r12 = " firstValue, "
            r11.append(r12)
            int r12 = r3.length
            int r12 = r12 - r14
            int r12 = java.lang.Math.min(r2, r12)
            r12 = r3[r12]
            r11.append(r12)
            r11.append(r4)
            java.lang.String r12 = " secondValue) {\n"
            r11.append(r12)
            java.lang.String r11 = r11.toString()
            r7.text = r11
            java.lang.String r11 = "        "
            java.lang.String r12 = ""
            java.lang.String r14 = ""
            java.lang.String r9 = ""
            r39 = r0
            r0 = r9
            r9 = 0
        L_0x0cac:
            if (r9 >= r8) goto L_0x0d47
            r40 = r3
            int r3 = r9 + 105
            char r3 = (char) r3
            r41 = r4
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r42 = r10
            java.lang.String r10 = r7.text
            r4.append(r10)
            r4.append(r11)
            java.lang.String r10 = "for (int "
            r4.append(r10)
            r4.append(r3)
            java.lang.String r10 = " = 0; "
            r4.append(r10)
            r4.append(r3)
            java.lang.String r10 = " < firstValue"
            r4.append(r10)
            r4.append(r12)
            java.lang.String r10 = ".length && "
            r4.append(r10)
            r4.append(r3)
            java.lang.String r10 = " < secondValue"
            r4.append(r10)
            r4.append(r12)
            java.lang.String r10 = ".length; "
            r4.append(r10)
            r4.append(r3)
            java.lang.String r10 = "++) {\n"
            r4.append(r10)
            java.lang.String r4 = r4.toString()
            r7.text = r4
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r11)
            java.lang.String r10 = "    "
            r4.append(r10)
            java.lang.String r11 = r4.toString()
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r12)
            java.lang.String r10 = "["
            r4.append(r10)
            r4.append(r3)
            java.lang.String r10 = "]"
            r4.append(r10)
            java.lang.String r12 = r4.toString()
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r14)
            r4.append(r0)
            r4.append(r3)
            java.lang.String r14 = r4.toString()
            java.lang.String r0 = ", "
            int r9 = r9 + 1
            r3 = r40
            r4 = r41
            r10 = r42
            goto L_0x0cac
        L_0x0d47:
            r40 = r3
            r41 = r4
            r42 = r10
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r7.text
            r3.append(r4)
            r3.append(r11)
            java.lang.String r4 = "first("
            r3.append(r4)
            r3.append(r14)
            r3.append(r0)
            java.lang.String r4 = "firstValue"
            r3.append(r4)
            r3.append(r12)
            java.lang.String r4 = ");\n"
            r3.append(r4)
            r3.append(r11)
            java.lang.String r4 = "second("
            r3.append(r4)
            r3.append(r14)
            r3.append(r0)
            java.lang.String r4 = "secondValue"
            r3.append(r4)
            r3.append(r12)
            java.lang.String r4 = ");\n"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            r3 = 0
        L_0x0d94:
            if (r3 >= r8) goto L_0x0db6
            r4 = 4
            java.lang.String r11 = r11.substring(r4)
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r9 = r7.text
            r4.append(r9)
            r4.append(r11)
            java.lang.String r9 = "}\n"
            r4.append(r9)
            java.lang.String r4 = r4.toString()
            r7.text = r4
            int r3 = r3 + 1
            goto L_0x0d94
        L_0x0db6:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r7.text
            r3.append(r4)
            java.lang.String r4 = "        return this;\n    }\n"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            int r2 = r2 + 1
            r9 = r0
            r0 = r39
            r3 = r40
            r4 = r41
            r10 = r42
            goto L_0x0c4c
        L_0x0dd8:
            r42 = r10
            r5 = r37
            r1 = r38
        L_0x0dde:
            if (r36 == 0) goto L_0x0fde
            if (r15 != 0) goto L_0x0fde
            if (r1 != 0) goto L_0x0fde
            r0 = 1
            java.lang.String[] r3 = r6.javaNames
            if (r3 == 0) goto L_0x0ded
            java.lang.String[] r3 = r6.javaNames
            r12 = 0
            goto L_0x0df5
        L_0x0ded:
            r3 = 1
            java.lang.String[] r3 = new java.lang.String[r3]
            java.lang.String r4 = r6.javaName
            r12 = 0
            r3[r12] = r4
        L_0x0df5:
            int r4 = r3.length
            r9 = r2
            r2 = r0
            r0 = 0
        L_0x0df9:
            if (r0 >= r4) goto L_0x0fdb
            r10 = r3[r0]
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r14 = r7.text
            r11.append(r14)
            java.lang.String r14 = "\n"
            r11.append(r14)
            java.lang.String r11 = r11.toString()
            r7.text = r11
            r11 = 2
            if (r8 >= r11) goto L_0x0e75
            if (r2 == 0) goto L_0x0e3c
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r11 = r7.text
            r14.append(r11)
            java.lang.String r11 = "    public "
            r14.append(r11)
            r14.append(r10)
            java.lang.String r11 = " pop_back() {\n        long size = size();\n        "
            r14.append(r11)
            r14.append(r10)
            java.lang.String r11 = " value = get(size - 1);\n        resize(size - 1);\n        return value;\n    }\n"
            r14.append(r11)
            java.lang.String r11 = r14.toString()
            r7.text = r11
        L_0x0e3c:
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r14 = r7.text
            r11.append(r14)
            java.lang.String r14 = "    public "
            r11.append(r14)
            java.lang.String r14 = r13.javaName
            r11.append(r14)
            java.lang.String r14 = " push_back("
            r11.append(r14)
            r11.append(r10)
            java.lang.String r14 = " value) {\n        long size = size();\n        resize(size + 1);\n        return put(size, value);\n    }\n    public "
            r11.append(r14)
            java.lang.String r14 = r13.javaName
            r11.append(r14)
            java.lang.String r14 = " put("
            r11.append(r14)
            r11.append(r10)
            java.lang.String r14 = " value) {\n        if (size() != 1) { resize(1); }\n        return put(0, value);\n    }\n"
            r11.append(r14)
            java.lang.String r11 = r11.toString()
            r7.text = r11
        L_0x0e75:
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r14 = r7.text
            r11.append(r14)
            java.lang.String r14 = "    public "
            r11.append(r14)
            java.lang.String r14 = r13.javaName
            r11.append(r14)
            java.lang.String r14 = " put("
            r11.append(r14)
            r11.append(r10)
            r11.append(r5)
            java.lang.String r14 = " ... array) {\n"
            r11.append(r14)
            java.lang.String r11 = r11.toString()
            r7.text = r11
            java.lang.String r11 = "        "
            java.lang.String r14 = ""
            java.lang.String r16 = ""
            java.lang.String r9 = ""
            r43 = r1
            r1 = r9
            r12 = r14
            r14 = r16
            r9 = 0
        L_0x0eae:
            if (r9 >= r8) goto L_0x0f64
            r44 = r2
            int r2 = r9 + 105
            char r2 = (char) r2
            r45 = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r46 = r4
            java.lang.String r4 = r7.text
            r3.append(r4)
            r3.append(r11)
            java.lang.String r4 = "if (size("
            r3.append(r4)
            r3.append(r14)
            java.lang.String r4 = ") != array"
            r3.append(r4)
            r3.append(r12)
            java.lang.String r4 = ".length) { resize("
            r3.append(r4)
            r3.append(r14)
            r3.append(r1)
            java.lang.String r4 = "array"
            r3.append(r4)
            r3.append(r12)
            java.lang.String r4 = ".length); }\n"
            r3.append(r4)
            r3.append(r11)
            java.lang.String r4 = "for (int "
            r3.append(r4)
            r3.append(r2)
            java.lang.String r4 = " = 0; "
            r3.append(r4)
            r3.append(r2)
            java.lang.String r4 = " < array"
            r3.append(r4)
            r3.append(r12)
            java.lang.String r4 = ".length; "
            r3.append(r4)
            r3.append(r2)
            java.lang.String r4 = "++) {\n"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r11)
            java.lang.String r4 = "    "
            r3.append(r4)
            java.lang.String r11 = r3.toString()
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r12)
            java.lang.String r4 = "["
            r3.append(r4)
            r3.append(r2)
            java.lang.String r4 = "]"
            r3.append(r4)
            java.lang.String r12 = r3.toString()
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r14)
            r3.append(r1)
            r3.append(r2)
            java.lang.String r14 = r3.toString()
            java.lang.String r1 = ", "
            int r9 = r9 + 1
            r2 = r44
            r3 = r45
            r4 = r46
            goto L_0x0eae
        L_0x0f64:
            r44 = r2
            r45 = r3
            r46 = r4
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = r7.text
            r2.append(r3)
            r2.append(r11)
            java.lang.String r3 = "put("
            r2.append(r3)
            r2.append(r14)
            r2.append(r1)
            java.lang.String r3 = "array"
            r2.append(r3)
            r2.append(r12)
            java.lang.String r3 = ");\n"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r7.text = r2
            r2 = 0
        L_0x0f96:
            if (r2 >= r8) goto L_0x0fb8
            r3 = 4
            java.lang.String r11 = r11.substring(r3)
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r9 = r7.text
            r4.append(r9)
            r4.append(r11)
            java.lang.String r9 = "}\n"
            r4.append(r9)
            java.lang.String r4 = r4.toString()
            r7.text = r4
            int r2 = r2 + 1
            goto L_0x0f96
        L_0x0fb8:
            r3 = 4
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r4 = r7.text
            r2.append(r4)
            java.lang.String r4 = "        return this;\n    }\n"
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            r7.text = r2
            r2 = 0
            int r0 = r0 + 1
            r9 = r1
            r1 = r43
            r3 = r45
            r4 = r46
            r12 = 0
            goto L_0x0df9
        L_0x0fdb:
            r43 = r1
            goto L_0x0fe1
        L_0x0fde:
            r43 = r1
            r9 = r2
        L_0x0fe1:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r7.text
            r0.append(r1)
            java.lang.String r1 = "}\n"
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r7.text = r0
            r0 = r49
            r0.add((org.bytedeco.javacpp.tools.Declaration) r7)
            r1 = r18
            r2 = r20
            r4 = r21
            r5 = r22
            r3 = r33
            r0 = r47
            goto L_0x0065
        L_0x100a:
            r0 = r49
            r0 = r47
            goto L_0x0065
        L_0x1010:
            r0 = r49
            r18 = r1
            r20 = r2
            r0 = r47
            goto L_0x002d
        L_0x101a:
            r0 = r49
            r18 = r1
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.containers(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):void");
    }

    /* access modifiers changed from: package-private */
    public TemplateMap template(Context context) throws ParserException {
        Token token;
        if (!this.tokens.get().match(Token.TEMPLATE)) {
            return null;
        }
        TemplateMap map = new TemplateMap(context.templateMap);
        this.tokens.next().expect('<');
        Token token2 = this.tokens.next();
        while (true) {
            if (token2.match(Token.EOF)) {
                break;
            }
            if (token2.match(5)) {
                Token t = this.tokens.next();
                if (t.match("...")) {
                    map.variadic = true;
                    t = this.tokens.next();
                }
                if (t.match(5)) {
                    String key = t.value;
                    map.put(key, map.get(key));
                    token2 = this.tokens.next();
                }
            }
            if (!token.match(',', '>')) {
                int count = 0;
                token = this.tokens.get();
                while (true) {
                    if (token.match(Token.EOF)) {
                        break;
                    }
                    if (count == 0) {
                        if (token.match(',', '>')) {
                            break;
                        }
                    }
                    if (token.match('<', '(')) {
                        count++;
                    } else {
                        if (token.match('>', ')')) {
                            count--;
                        }
                    }
                    token = this.tokens.next();
                }
            }
            if (token.expect(',', '>').match('>')) {
                if (!this.tokens.next().match(Token.TEMPLATE)) {
                    break;
                }
                this.tokens.next().expect('<');
            }
            token2 = this.tokens.next();
        }
        return map;
    }

    /* access modifiers changed from: package-private */
    public Type[] templateArguments(Context context) throws ParserException {
        if (!this.tokens.get().match('<')) {
            return null;
        }
        List<Type> arguments = new ArrayList<>();
        Token token = this.tokens.next();
        while (true) {
            if (token.match(Token.EOF)) {
                break;
            }
            Type type = type(context);
            arguments.add(type);
            Token token2 = this.tokens.get();
            if (!token2.match(',', '>')) {
                int count = 0;
                token2 = this.tokens.get();
                while (true) {
                    if (token2.match(Token.EOF)) {
                        break;
                    }
                    if (count == 0) {
                        if (token2.match(',', '>')) {
                            break;
                        }
                    }
                    if (token2.match('<', '(')) {
                        count++;
                    } else {
                        if (token2.match('>', ')')) {
                            count--;
                        }
                    }
                    type.cppName += token2;
                    if (token2.match(Token.CONST, Token.__CONST)) {
                        type.cppName += " ";
                    }
                    token2 = this.tokens.next();
                }
                if (type.cppName.endsWith("*")) {
                    type.javaName = "PointerPointer";
                    type.annotations += "@Cast(\"" + type.cppName + "*\") ";
                }
            }
            if (token2.expect(',', '>').match('>')) {
                break;
            }
            token = this.tokens.next();
        }
        return (Type[]) arguments.toArray(new Type[arguments.size()]);
    }

    /* access modifiers changed from: package-private */
    public Type type(Context context) throws ParserException {
        return type(context, false);
    }

    /* access modifiers changed from: package-private */
    public Type type(Context context, boolean definition) throws ParserException {
        boolean z;
        Info info;
        String constName;
        String constName2;
        Token token;
        Context context2 = context;
        Type type = new Type();
        List<Attribute> attributes = new ArrayList<>();
        Token token2 = this.tokens.get();
        while (true) {
            z = false;
            if (token2.match(Token.EOF)) {
                break;
            }
            if (token2.match("::")) {
                Info info2 = this.infoMap.getFirst(type.cppName, false);
                if (info2 != null && info2.pointerTypes != null && info2.pointerTypes.length > 0 && !type.cppName.contains("::") && token2.spacing.length() > 0) {
                    break;
                }
                type.cppName += token2;
            } else {
                if (token2.match(Token.DECLTYPE)) {
                    type.cppName += token2.toString() + this.tokens.next().expect('(');
                    while (true) {
                        token = this.tokens.next();
                        if (token.match(')', Token.EOF)) {
                            break;
                        }
                        type.cppName += token;
                    }
                    type.cppName += token;
                    this.tokens.next();
                } else {
                    if (token2.match('<')) {
                        type.arguments = templateArguments(context);
                        type.cppName += "<";
                        String separator = "";
                        for (Type t : type.arguments) {
                            if (t != null) {
                                type.cppName += separator;
                                Info info3 = this.infoMap.getFirst(t.cppName);
                                String s = (info3 == null || info3.cppTypes == null) ? t.cppName : info3.cppTypes[0];
                                if (t.constValue && !s.startsWith("const ")) {
                                    s = "const " + s;
                                }
                                if (t.constPointer && !s.endsWith(" const")) {
                                    s = s + " const";
                                }
                                String s2 = s;
                                for (int i = 0; i < t.indirections; i++) {
                                    s2 = s2 + "*";
                                }
                                if (t.reference != 0) {
                                    s2 = s2 + "&";
                                }
                                type.cppName += s2;
                                separator = ",";
                            }
                        }
                        StringBuilder sb = new StringBuilder();
                        sb.append(type.cppName);
                        sb.append(type.cppName.endsWith(">") ? " >" : ">");
                        type.cppName = sb.toString();
                    } else {
                        if (token2.match(Token.CONST, Token.__CONST, Token.CONSTEXPR)) {
                            int template = type.cppName.lastIndexOf(60);
                            if (!(template >= 0 ? type.cppName.substring(0, template) : type.cppName).trim().contains(" ") || type.simple) {
                                type.constValue = true;
                            } else {
                                type.constPointer = true;
                            }
                        } else {
                            if (token2.match('*')) {
                                type.indirections++;
                                this.tokens.next();
                                break;
                            }
                            if (token2.match('&')) {
                                type.reference = true;
                                this.tokens.next();
                                break;
                            }
                            if (!token2.match("&&")) {
                                if (token2.match('~')) {
                                    type.cppName += "~";
                                    type.destructor = true;
                                } else {
                                    if (token2.match(Token.STATIC)) {
                                        type.staticMember = true;
                                    } else {
                                        if (!token2.match(Token.OPERATOR)) {
                                            if (token2.match(Token.USING)) {
                                                type.using = true;
                                            } else {
                                                if (token2.match(Token.FRIEND)) {
                                                    type.friend = true;
                                                } else {
                                                    if (token2.match(Token.TYPEDEF)) {
                                                        type.typedef = true;
                                                    } else {
                                                        if (token2.match(Token.VIRTUAL)) {
                                                            type.virtual = true;
                                                        } else {
                                                            if (token2.match(Token.AUTO, Token.ENUM, Token.EXPLICIT, Token.EXTERN, Token.INLINE, Token.CLASS, Token.FINAL, Token.INTERFACE, Token.__INTERFACE, Token.MUTABLE, Token.NAMESPACE, Token.STRUCT, Token.UNION, Token.TYPENAME, Token.REGISTER, Token.THREAD_LOCAL, Token.VOLATILE)) {
                                                                Token token3 = this.tokens.next();
                                                            } else if (!token2.match((Object[]) this.infoMap.getFirst("basic/types").cppTypes) || (type.cppName.length() != 0 && !type.simple)) {
                                                                if (token2.match(5)) {
                                                                    int backIndex = this.tokens.index;
                                                                    Attribute attr = attribute();
                                                                    if (attr == null || !attr.annotation) {
                                                                        this.tokens.index = backIndex;
                                                                        if (type.cppName.length() != 0 && !type.cppName.endsWith("::") && !type.cppName.endsWith("~")) {
                                                                            Info info4 = this.infoMap.getFirst(this.tokens.get(1).value);
                                                                            if (info4 != null && info4.annotations != null) {
                                                                                break;
                                                                            }
                                                                            if (!this.tokens.get(1).match('*', '&', 5, Token.CONST, Token.__CONST, Token.CONSTEXPR, Token.FINAL)) {
                                                                                break;
                                                                            }
                                                                        } else {
                                                                            type.cppName += token2.value;
                                                                        }
                                                                    } else {
                                                                        type.annotations += attr.javaName;
                                                                        attributes.add(attr);
                                                                    }
                                                                } else {
                                                                    if (token2.match('}')) {
                                                                        type.anonymous = true;
                                                                        this.tokens.next();
                                                                    }
                                                                }
                                                            } else {
                                                                type.cppName += token2.value + " ";
                                                                type.simple = true;
                                                            }
                                                            token2 = this.tokens.get();
                                                        }
                                                    }
                                                }
                                            }
                                        } else if (type.cppName.length() == 0) {
                                            type.operator = true;
                                            this.tokens.next();
                                            token2 = this.tokens.get();
                                        } else if (type.cppName.endsWith("::")) {
                                            type.operator = true;
                                            this.tokens.next();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            this.tokens.next();
            token2 = this.tokens.get();
        }
        if (attributes.size() > 0) {
            type.attributes = (Attribute[]) attributes.toArray(new Attribute[attributes.size()]);
        }
        type.cppName = type.cppName.trim();
        if (this.tokens.get().match("...")) {
            this.tokens.next();
            if (!this.tokens.get().match(5)) {
                return null;
            }
            this.tokens.next();
            return null;
        }
        if (type.operator) {
            Token token4 = this.tokens.get();
            while (true) {
                if (token4.match(Token.EOF, '(', ';')) {
                    break;
                }
                type.cppName += token4;
                token4 = this.tokens.next();
            }
        }
        if (type.cppName.endsWith("*")) {
            type.indirections++;
            type.cppName = type.cppName.substring(0, type.cppName.length() - 1);
        }
        if (type.cppName.endsWith("&")) {
            type.reference = true;
            type.cppName = type.cppName.substring(0, type.cppName.length() - 1);
        }
        if (context2.templateMap != null) {
            String[] types = type.cppName.split("::");
            type.cppName = "";
            List<Type> arguments = new ArrayList<>();
            String separator2 = "";
            for (String t2 : types) {
                Type t22 = context2.templateMap.get(t2);
                StringBuilder sb2 = new StringBuilder();
                sb2.append(type.cppName);
                sb2.append(separator2);
                sb2.append(t22 != null ? t22.cppName : t2);
                type.cppName = sb2.toString();
                if (!(t22 == null || t22.arguments == null)) {
                    arguments.addAll(Arrays.asList(t22.arguments));
                }
                separator2 = "::";
            }
            if (arguments.size() > 0) {
                type.arguments = (Type[]) arguments.toArray(new Type[arguments.size()]);
            }
        }
        if (type.cppName.startsWith("const ")) {
            type.constValue = true;
            type.cppName = type.cppName.substring(6);
        }
        if (type.cppName.endsWith("*")) {
            type.indirections++;
            if (type.reference) {
                type.constValue = false;
            }
            type.cppName = type.cppName.substring(0, type.cppName.length() - 1);
        }
        if (type.cppName.endsWith("&")) {
            type.reference = true;
            type.cppName = type.cppName.substring(0, type.cppName.length() - 1);
        }
        if (type.cppName.endsWith(" const")) {
            type.constPointer = true;
            type.cppName = type.cppName.substring(0, type.cppName.length() - 6);
        }
        String[] names = context2.qualify(type.cppName);
        if (!definition || names.length <= 0) {
            int length = names.length;
            info = null;
            int i2 = 0;
            while (true) {
                if (i2 >= length) {
                    break;
                }
                String name = names[i2];
                if (type.constValue || type.constPointer) {
                    constName = "const " + name;
                } else {
                    constName = name;
                }
                Info first = this.infoMap.getFirst(constName, false);
                info = first;
                if (first != null) {
                    type.cppName = name;
                    break;
                }
                if (this.infoMap.getFirst(constName) != null) {
                    type.cppName = name;
                }
                i2++;
            }
        } else {
            if (type.constValue || type.constPointer) {
                constName2 = "const " + names[0];
            } else {
                constName2 = names[0];
            }
            Info info5 = this.infoMap.getFirst(constName2, false);
            type.cppName = names[0];
            info = info5;
        }
        if (!(info == null || info.cppTypes == null || info.cppTypes.length <= 0)) {
            type.cppName = info.cppTypes[0];
        }
        if (type.cppName.startsWith("const ")) {
            type.constValue = true;
            type.cppName = type.cppName.substring(6);
        }
        if (type.cppName.endsWith("*")) {
            type.indirections++;
            if (type.reference) {
                type.constValue = false;
            }
            type.cppName = type.cppName.substring(0, type.cppName.length() - 1);
        }
        if (type.cppName.endsWith("&")) {
            type.reference = true;
            type.cppName = type.cppName.substring(0, type.cppName.length() - 1);
        }
        if (type.cppName.endsWith(" const")) {
            type.constPointer = true;
            type.cppName = type.cppName.substring(0, type.cppName.length() - 6);
        }
        int namespace = type.cppName.lastIndexOf("::");
        int template2 = type.cppName.lastIndexOf(60);
        type.javaName = (namespace < 0 || template2 >= 0) ? type.cppName : type.cppName.substring(namespace + 2);
        if (info != null) {
            if (type.indirections == 0 && !type.reference && info.valueTypes != null && info.valueTypes.length > 0) {
                type.javaName = info.valueTypes[0];
                type.javaNames = info.valueTypes;
                type.value = true;
            } else if (info.pointerTypes != null && info.pointerTypes.length > 0) {
                type.javaName = info.pointerTypes[0];
                type.javaNames = info.pointerTypes;
            }
        }
        if (type.operator) {
            if (type.constValue) {
                type.annotations += "@Const ";
            }
            if (type.indirections == 0 && !type.reference && !type.value) {
                type.annotations += "@ByVal ";
            } else if (type.indirections == 0 && type.reference && !type.value) {
                type.annotations += "@ByRef ";
            }
            if (info != null && info.cast) {
                StringBuilder sb3 = new StringBuilder();
                sb3.append(type.annotations);
                sb3.append("@Cast(\"");
                sb3.append(type.cppName);
                sb3.append((type.indirections != 0 || type.value) ? "" : "*");
                sb3.append("\") ");
                type.annotations = sb3.toString();
            }
            StringBuilder sb4 = new StringBuilder();
            sb4.append(type.annotations);
            sb4.append("@Name(\"operator ");
            sb4.append(type.constValue ? "const " : "");
            sb4.append(type.cppName);
            sb4.append(type.indirections > 0 ? "*" : type.reference ? "&" : "");
            sb4.append("\") ");
            type.annotations = sb4.toString();
        }
        if (!(info == null || info.annotations == null)) {
            for (String s3 : info.annotations) {
                type.annotations += s3 + " ";
            }
        }
        if (context2.cppName != null && type.javaName.length() > 0) {
            String cppName = type.cppName;
            String groupName = context2.cppName;
            int namespace2 = -1;
            int template22 = groupName != null ? groupName.lastIndexOf(60) : -1;
            if (template2 < 0 && template22 >= 0) {
                groupName = groupName.substring(0, template22);
            } else if (template2 >= 0 && template22 < 0) {
                cppName = cppName.substring(0, template2);
            }
            if (groupName != null) {
                namespace2 = groupName.lastIndexOf("::");
            }
            if (namespace < 0 && namespace2 >= 0) {
                groupName = groupName.substring(namespace2 + 2);
            }
            if (cppName.equals(groupName)) {
                if (!type.destructor && !type.operator && type.indirections == 0 && !type.reference) {
                    if (this.tokens.get().match('(', ':')) {
                        z = true;
                    }
                }
                type.constructor = z;
            }
            type.javaName = context2.shorten(type.javaName);
        }
        return type;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r29v2, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v202, resolved type: org.bytedeco.javacpp.tools.Attribute} */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Multi-variable type inference failed */
    /* JADX WARNING: Removed duplicated region for block: B:249:0x07fb  */
    /* JADX WARNING: Removed duplicated region for block: B:253:0x0813  */
    /* JADX WARNING: Removed duplicated region for block: B:292:0x08aa  */
    /* JADX WARNING: Removed duplicated region for block: B:308:0x093f  */
    /* JADX WARNING: Removed duplicated region for block: B:310:0x0943  */
    /* JADX WARNING: Removed duplicated region for block: B:314:0x0966  */
    /* JADX WARNING: Removed duplicated region for block: B:317:0x0980  */
    /* JADX WARNING: Removed duplicated region for block: B:334:0x0a4a  */
    /* JADX WARNING: Removed duplicated region for block: B:339:0x0a8f  */
    /* JADX WARNING: Removed duplicated region for block: B:340:0x0a91  */
    /* JADX WARNING: Removed duplicated region for block: B:347:0x0ac0  */
    /* JADX WARNING: Removed duplicated region for block: B:348:0x0ac4  */
    /* JADX WARNING: Removed duplicated region for block: B:365:0x0af5  */
    /* JADX WARNING: Removed duplicated region for block: B:366:0x0b07  */
    /* JADX WARNING: Removed duplicated region for block: B:370:0x0b1b  */
    /* JADX WARNING: Removed duplicated region for block: B:407:0x0c69 A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:442:0x0cb9  */
    /* JADX WARNING: Removed duplicated region for block: B:443:0x0cbd  */
    /* JADX WARNING: Removed duplicated region for block: B:446:0x0cc7  */
    /* JADX WARNING: Removed duplicated region for block: B:451:0x0cd9  */
    /* JADX WARNING: Removed duplicated region for block: B:458:0x0cfd  */
    /* JADX WARNING: Removed duplicated region for block: B:459:0x0cff  */
    /* JADX WARNING: Removed duplicated region for block: B:465:0x0d10  */
    /* JADX WARNING: Removed duplicated region for block: B:466:0x0d14  */
    /* JADX WARNING: Removed duplicated region for block: B:523:0x0e92  */
    /* JADX WARNING: Removed duplicated region for block: B:550:0x0f45  */
    /* JADX WARNING: Removed duplicated region for block: B:551:0x0f4a  */
    /* JADX WARNING: Removed duplicated region for block: B:554:0x0f52  */
    /* JADX WARNING: Removed duplicated region for block: B:564:0x0f91  */
    /* JADX WARNING: Removed duplicated region for block: B:566:0x0f97  */
    /* JADX WARNING: Removed duplicated region for block: B:567:0x0f9d  */
    /* JADX WARNING: Removed duplicated region for block: B:589:0x0fdc A[LOOP:22: B:588:0x0fda->B:589:0x0fdc, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:595:0x1019  */
    /* JADX WARNING: Removed duplicated region for block: B:598:0x1029 A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:603:0x104e  */
    /* JADX WARNING: Removed duplicated region for block: B:606:0x1073  */
    /* JADX WARNING: Removed duplicated region for block: B:616:0x10ce  */
    /* JADX WARNING: Removed duplicated region for block: B:632:0x1151  */
    /* JADX WARNING: Removed duplicated region for block: B:635:0x116f  */
    /* JADX WARNING: Removed duplicated region for block: B:638:0x117a  */
    /* JADX WARNING: Removed duplicated region for block: B:644:0x11a2  */
    /* JADX WARNING: Removed duplicated region for block: B:645:0x11a8  */
    /* JADX WARNING: Removed duplicated region for block: B:665:0x1226 A[LOOP:26: B:664:0x1224->B:665:0x1226, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:668:0x125f  */
    /* JADX WARNING: Removed duplicated region for block: B:688:0x13ae  */
    /* JADX WARNING: Removed duplicated region for block: B:700:0x13e0  */
    /* JADX WARNING: Removed duplicated region for block: B:703:0x13e8  */
    /* JADX WARNING: Removed duplicated region for block: B:704:0x1420  */
    /* JADX WARNING: Removed duplicated region for block: B:716:0x1450  */
    /* JADX WARNING: Removed duplicated region for block: B:734:0x14d6 A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:754:0x0272 A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:795:0x088e A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:823:0x14e0 A[ADDED_TO_REGION, EDGE_INSN: B:823:0x14e0->B:736:0x14e0 ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:85:0x0284 A[LOOP:4: B:64:0x0218->B:85:0x0284, LOOP_END] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.bytedeco.javacpp.tools.Declarator declarator(org.bytedeco.javacpp.tools.Context r83, java.lang.String r84, int r85, boolean r86, int r87, boolean r88, boolean r89) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r82 = this;
            r1 = r82
            r2 = r83
            r3 = r84
            r4 = r85
            r5 = r87
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r8 = 1
            java.lang.Object[] r9 = new java.lang.Object[r8]
            org.bytedeco.javacpp.tools.Token r10 = org.bytedeco.javacpp.tools.Token.TYPEDEF
            r11 = 0
            r9[r11] = r10
            boolean r0 = r0.match(r9)
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r9.get()
            java.lang.Object[] r10 = new java.lang.Object[r8]
            org.bytedeco.javacpp.tools.Token r12 = org.bytedeco.javacpp.tools.Token.USING
            r10[r11] = r12
            boolean r9 = r9.match(r10)
            org.bytedeco.javacpp.tools.Declarator r10 = new org.bytedeco.javacpp.tools.Declarator
            r10.<init>()
            org.bytedeco.javacpp.tools.Type r12 = r82.type(r83)
            r13 = 0
            if (r12 != 0) goto L_0x0039
            return r13
        L_0x0039:
            boolean r14 = r12.typedef
            r14 = r14 | r0
            r0 = 0
            r15 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r13 = r1.tokens
            org.bytedeco.javacpp.tools.Token r13 = r13.get()
        L_0x0044:
            r16 = 59
            r17 = 44
            r18 = 93
            r20 = 91
            if (r15 >= r5) goto L_0x00e0
            java.lang.Object[] r11 = new java.lang.Object[r8]
            org.bytedeco.javacpp.tools.Token r23 = org.bytedeco.javacpp.tools.Token.EOF
            r19 = 0
            r11[r19] = r23
            boolean r11 = r13.match(r11)
            if (r11 != 0) goto L_0x00e0
            r11 = 3
            java.lang.Object[] r8 = new java.lang.Object[r11]
            r11 = 40
            java.lang.Character r11 = java.lang.Character.valueOf(r11)
            r8[r19] = r11
            java.lang.Character r11 = java.lang.Character.valueOf(r20)
            r20 = 1
            r8[r20] = r11
            r11 = 123(0x7b, float:1.72E-43)
            java.lang.Character r11 = java.lang.Character.valueOf(r11)
            r20 = 2
            r8[r20] = r11
            boolean r8 = r13.match(r8)
            if (r8 == 0) goto L_0x0082
            int r0 = r0 + 1
            goto L_0x00d6
        L_0x0082:
            r8 = 3
            java.lang.Object[] r8 = new java.lang.Object[r8]
            r11 = 41
            java.lang.Character r11 = java.lang.Character.valueOf(r11)
            r19 = 0
            r8[r19] = r11
            java.lang.Character r11 = java.lang.Character.valueOf(r18)
            r18 = 1
            r8[r18] = r11
            r11 = 125(0x7d, float:1.75E-43)
            java.lang.Character r11 = java.lang.Character.valueOf(r11)
            r18 = 2
            r8[r18] = r11
            boolean r8 = r13.match(r8)
            if (r8 == 0) goto L_0x00aa
            int r0 = r0 + -1
            goto L_0x00d6
        L_0x00aa:
            if (r0 <= 0) goto L_0x00ad
            goto L_0x00d6
        L_0x00ad:
            r8 = 1
            java.lang.Object[] r11 = new java.lang.Object[r8]
            java.lang.Character r17 = java.lang.Character.valueOf(r17)
            r18 = 0
            r11[r18] = r17
            boolean r11 = r13.match(r11)
            if (r11 == 0) goto L_0x00c1
            int r15 = r15 + 1
            goto L_0x00d6
        L_0x00c1:
            java.lang.Object[] r11 = new java.lang.Object[r8]
            java.lang.Character r8 = java.lang.Character.valueOf(r16)
            r11[r18] = r8
            boolean r8 = r13.match(r11)
            if (r8 == 0) goto L_0x00d6
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            r8.next()
            r8 = 0
            return r8
        L_0x00d6:
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            org.bytedeco.javacpp.tools.Token r13 = r8.next()
            r8 = 1
            r11 = 0
            goto L_0x0044
        L_0x00e0:
            r8 = 0
            java.lang.String r11 = r12.cppName
            boolean r13 = r12.constPointer
            if (r13 == 0) goto L_0x00fe
            r13 = 1
            r10.constPointer = r13
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r11)
            r25 = r0
            java.lang.String r0 = " const"
            r13.append(r0)
            java.lang.String r11 = r13.toString()
            goto L_0x0100
        L_0x00fe:
            r25 = r0
        L_0x0100:
            if (r5 != 0) goto L_0x012a
            int r0 = r12.indirections
            if (r0 <= 0) goto L_0x012a
            int r0 = r10.indirections
            int r13 = r12.indirections
            int r0 = r0 + r13
            r10.indirections = r0
            r0 = 0
        L_0x010e:
            int r13 = r12.indirections
            if (r0 >= r13) goto L_0x012a
            java.lang.StringBuilder r13 = new java.lang.StringBuilder
            r13.<init>()
            r13.append(r11)
            r26 = r8
            java.lang.String r8 = "*"
            r13.append(r8)
            java.lang.String r11 = r13.toString()
            int r0 = r0 + 1
            r8 = r26
            goto L_0x010e
        L_0x012a:
            r26 = r8
            if (r5 != 0) goto L_0x0146
            boolean r0 = r12.reference
            if (r0 == 0) goto L_0x0146
            r8 = 1
            r10.reference = r8
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            java.lang.String r8 = "&"
            r0.append(r8)
            java.lang.String r11 = r0.toString()
        L_0x0146:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
        L_0x014c:
            r8 = 1
            java.lang.Object[] r13 = new java.lang.Object[r8]
            org.bytedeco.javacpp.tools.Token r23 = org.bytedeco.javacpp.tools.Token.EOF
            r19 = 0
            r13[r19] = r23
            boolean r13 = r0.match(r13)
            r8 = 42
            if (r13 != 0) goto L_0x01c5
            r13 = 1
            java.lang.Object[] r5 = new java.lang.Object[r13]
            java.lang.Character r23 = java.lang.Character.valueOf(r8)
            r5[r19] = r23
            boolean r5 = r0.match(r5)
            if (r5 == 0) goto L_0x0172
            int r5 = r10.indirections
            int r5 = r5 + r13
            r10.indirections = r5
            goto L_0x01ad
        L_0x0172:
            java.lang.Object[] r5 = new java.lang.Object[r13]
            r23 = 38
            java.lang.Character r23 = java.lang.Character.valueOf(r23)
            r19 = 0
            r5[r19] = r23
            boolean r5 = r0.match(r5)
            if (r5 == 0) goto L_0x0187
            r10.reference = r13
            goto L_0x01ad
        L_0x0187:
            java.lang.Object[] r5 = new java.lang.Object[r13]
            java.lang.String r23 = "&&"
            r5[r19] = r23
            boolean r5 = r0.match(r5)
            if (r5 == 0) goto L_0x0194
            goto L_0x01ad
        L_0x0194:
            r5 = 3
            java.lang.Object[] r8 = new java.lang.Object[r5]
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.CONST
            r8[r19] = r5
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.__CONST
            r8[r13] = r5
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.CONSTEXPR
            r22 = 2
            r8[r22] = r5
            boolean r5 = r0.match(r8)
            if (r5 == 0) goto L_0x01c5
            r10.constPointer = r13
        L_0x01ad:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r11)
            r5.append(r0)
            java.lang.String r11 = r5.toString()
            org.bytedeco.javacpp.tools.TokenIndexer r5 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r5.next()
            r5 = r87
            goto L_0x014c
        L_0x01c5:
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r5 = r0
            org.bytedeco.javacpp.tools.Attribute[] r0 = r12.attributes
            if (r0 == 0) goto L_0x01d8
            org.bytedeco.javacpp.tools.Attribute[] r0 = r12.attributes
            java.util.List r0 = java.util.Arrays.asList(r0)
            r5.addAll(r0)
        L_0x01d8:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r0 = r0.index
            org.bytedeco.javacpp.tools.Attribute r8 = r82.attribute()
            r13 = r0
        L_0x01e1:
            r0 = r8
            if (r0 == 0) goto L_0x020d
            boolean r8 = r0.annotation
            if (r8 == 0) goto L_0x020d
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r28 = r15
            java.lang.String r15 = r12.annotations
            r8.append(r15)
            java.lang.String r15 = r0.javaName
            r8.append(r15)
            java.lang.String r8 = r8.toString()
            r12.annotations = r8
            r5.add(r0)
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            int r13 = r8.index
            org.bytedeco.javacpp.tools.Attribute r8 = r82.attribute()
            r15 = r28
            goto L_0x01e1
        L_0x020d:
            r28 = r15
            r0 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            r8.index = r13
            java.util.Iterator r8 = r5.iterator()
        L_0x0218:
            boolean r15 = r8.hasNext()
            if (r15 == 0) goto L_0x028b
            java.lang.Object r15 = r8.next()
            org.bytedeco.javacpp.tools.Attribute r15 = (org.bytedeco.javacpp.tools.Attribute) r15
            r29 = r0
            java.lang.String r0 = r15.javaName
            if (r0 == 0) goto L_0x026a
            java.lang.String r0 = r15.javaName
            r30 = r8
            java.lang.String r8 = "@Name "
            boolean r0 = r0.contains(r8)
            if (r0 == 0) goto L_0x026c
            java.lang.String r0 = r15.arguments
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x026c
            java.lang.String r0 = r15.arguments
            r8 = 0
            char r0 = r0.charAt(r8)
            boolean r0 = java.lang.Character.isJavaIdentifierStart(r0)
            if (r0 == 0) goto L_0x026c
            r0 = r15
            java.lang.String r8 = r15.arguments
            char[] r8 = r8.toCharArray()
            r31 = r0
            int r0 = r8.length
            r32 = r13
            r13 = 0
        L_0x0258:
            if (r13 >= r0) goto L_0x0267
            char r23 = r8[r13]
            boolean r27 = java.lang.Character.isJavaIdentifierPart(r23)
            if (r27 != 0) goto L_0x0264
            r0 = 0
            goto L_0x0270
        L_0x0264:
            int r13 = r13 + 1
            goto L_0x0258
        L_0x0267:
            r0 = r31
            goto L_0x0270
        L_0x026a:
            r30 = r8
        L_0x026c:
            r32 = r13
            r0 = r29
        L_0x0270:
            if (r0 == 0) goto L_0x0284
            java.lang.String r8 = r12.annotations
            java.lang.String r13 = "@Name "
            r33 = r0
            java.lang.String r0 = ""
            java.lang.String r0 = r8.replace(r13, r0)
            r12.annotations = r0
            r8 = r33
            goto L_0x0291
        L_0x0284:
            r33 = r0
            r8 = r30
            r13 = r32
            goto L_0x0218
        L_0x028b:
            r29 = r0
            r32 = r13
            r8 = r29
        L_0x0291:
            r0 = 0
            r13 = r0
        L_0x0293:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r34 = r8
            r15 = 1
            java.lang.Object[] r8 = new java.lang.Object[r15]
            r21 = 40
            java.lang.Character r23 = java.lang.Character.valueOf(r21)
            r19 = 0
            r8[r19] = r23
            boolean r0 = r0.match(r8)
            if (r0 == 0) goto L_0x02cc
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r15)
            java.lang.Object[] r8 = new java.lang.Object[r15]
            java.lang.Character r15 = java.lang.Character.valueOf(r21)
            r8[r19] = r15
            boolean r0 = r0.match(r8)
            if (r0 == 0) goto L_0x02cc
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
            int r13 = r13 + 1
            r8 = r34
            goto L_0x0293
        L_0x02cc:
            r0 = 256(0x100, float:3.59E-43)
            int[] r8 = new int[r0]
            r0 = 0
            java.lang.String r15 = ""
            r10.cppName = r15
            r15 = 0
            org.bytedeco.javacpp.tools.Declaration r23 = new org.bytedeco.javacpp.tools.Declaration
            r23.<init>()
            r35 = r23
            r23 = 0
            r25 = 0
            java.util.Iterator r27 = r5.iterator()
        L_0x02e5:
            boolean r29 = r27.hasNext()
            if (r29 == 0) goto L_0x0313
            java.lang.Object r29 = r27.next()
            r36 = r0
            r0 = r29
            org.bytedeco.javacpp.tools.Attribute r0 = (org.bytedeco.javacpp.tools.Attribute) r0
            r37 = r13
            boolean r13 = r0.annotation
            if (r13 == 0) goto L_0x030e
            java.lang.String r13 = r0.javaName
            int r13 = r13.length()
            if (r13 != 0) goto L_0x030e
            java.lang.String r13 = r0.arguments
            int r13 = r13.length()
            if (r13 != 0) goto L_0x030e
            r25 = r0
        L_0x030e:
            r0 = r36
            r13 = r37
            goto L_0x02e5
        L_0x0313:
            r36 = r0
            r37 = r13
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r38 = r15
            r13 = 1
            java.lang.Object[] r15 = new java.lang.Object[r13]
            r13 = 40
            java.lang.Character r27 = java.lang.Character.valueOf(r13)
            r13 = 0
            r15[r13] = r27
            boolean r0 = r0.match(r15)
            if (r0 != 0) goto L_0x05f1
            if (r14 == 0) goto L_0x0353
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r13 = 1
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r13)
            java.lang.Object[] r15 = new java.lang.Object[r13]
            r13 = 40
            java.lang.Character r27 = java.lang.Character.valueOf(r13)
            r13 = 0
            r15[r13] = r27
            boolean r0 = r0.match(r15)
            if (r0 == 0) goto L_0x0354
            r39 = r9
            r42 = r14
            r13 = r35
            goto L_0x05f7
        L_0x0353:
            r13 = 0
        L_0x0354:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r39 = r9
            r15 = 2
            java.lang.Object[] r9 = new java.lang.Object[r15]
            r15 = 5
            java.lang.Integer r19 = java.lang.Integer.valueOf(r15)
            r9[r13] = r19
            java.lang.String r13 = "::"
            r15 = 1
            r9[r15] = r13
            boolean r0 = r0.match(r9)
            if (r0 == 0) goto L_0x05e7
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
        L_0x0377:
            java.lang.Object[] r9 = new java.lang.Object[r15]
            org.bytedeco.javacpp.tools.Token r13 = org.bytedeco.javacpp.tools.Token.EOF
            r15 = 0
            r9[r15] = r13
            boolean r9 = r0.match(r9)
            if (r9 != 0) goto L_0x05dd
            java.lang.String r9 = r10.cppName
            int r9 = r9.length()
            if (r9 <= 0) goto L_0x0456
            r9 = 1
            java.lang.Object[] r13 = new java.lang.Object[r9]
            r9 = 42
            java.lang.Character r19 = java.lang.Character.valueOf(r9)
            r13[r15] = r19
            boolean r9 = r0.match(r13)
            if (r9 == 0) goto L_0x0456
            java.lang.String r9 = r10.cppName
            java.lang.String r13 = r10.cppName
            int r13 = r13.length()
            r19 = 2
            int r13 = r13 + -2
            java.lang.String r9 = r9.substring(r15, r13)
            r10.cppName = r9
            java.lang.String r9 = r10.cppName
            java.lang.String[] r9 = r2.qualify(r9)
            int r13 = r9.length
            r15 = 0
        L_0x03b7:
            if (r15 >= r13) goto L_0x03e2
            r40 = r13
            r13 = r9[r15]
            r41 = r9
            org.bytedeco.javacpp.tools.InfoMap r9 = r1.infoMap
            r42 = r14
            r14 = 0
            org.bytedeco.javacpp.tools.Info r9 = r9.getFirst(r13, r14)
            r38 = r9
            if (r9 == 0) goto L_0x03cf
            r10.cppName = r13
            goto L_0x03e4
        L_0x03cf:
            org.bytedeco.javacpp.tools.InfoMap r9 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r9 = r9.getFirst(r13)
            if (r9 == 0) goto L_0x03d9
            r10.cppName = r13
        L_0x03d9:
            int r15 = r15 + 1
            r13 = r40
            r9 = r41
            r14 = r42
            goto L_0x03b7
        L_0x03e2:
            r42 = r14
        L_0x03e4:
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r13 = r35
            java.lang.String r14 = r13.text
            r9.append(r14)
            java.lang.String r14 = "@Namespace(\""
            r9.append(r14)
            java.lang.String r14 = r10.cppName
            r9.append(r14)
            java.lang.String r14 = "\") "
            r9.append(r14)
            java.lang.String r9 = r9.toString()
            r13.text = r9
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.get()
        L_0x040b:
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.EOF
            r19 = 0
            r14[r19] = r15
            boolean r14 = r0.match(r14)
            if (r14 != 0) goto L_0x0433
            java.lang.Object[] r14 = new java.lang.Object[r9]
            r9 = 42
            java.lang.Character r15 = java.lang.Character.valueOf(r9)
            r14[r19] = r15
            boolean r9 = r0.match(r14)
            if (r9 == 0) goto L_0x0433
            int r36 = r36 + 1
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.next()
            goto L_0x040b
        L_0x0433:
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            r9 = 5
            java.lang.Integer r15 = java.lang.Integer.valueOf(r9)
            r9 = 0
            r14[r9] = r15
            boolean r9 = r0.match(r14)
            if (r9 == 0) goto L_0x0449
            java.lang.String r9 = r0.toString()
            goto L_0x044b
        L_0x0449:
            java.lang.String r9 = ""
        L_0x044b:
            r10.cppName = r9
            if (r38 == 0) goto L_0x0451
            r9 = 1
            goto L_0x0452
        L_0x0451:
            r9 = 0
        L_0x0452:
            r23 = r9
            goto L_0x05d0
        L_0x0456:
            r42 = r14
            r13 = r35
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            java.lang.String r9 = "::"
            r15 = 0
            r14[r15] = r9
            boolean r9 = r0.match(r14)
            if (r9 == 0) goto L_0x047d
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r14 = r10.cppName
            r9.append(r14)
            r9.append(r0)
            java.lang.String r9 = r9.toString()
            r10.cppName = r9
            goto L_0x05d0
        L_0x047d:
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.OPERATOR
            r19 = 0
            r14[r19] = r15
            boolean r14 = r0.match(r14)
            if (r14 == 0) goto L_0x050c
            r10.operator = r9
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r1.tokens
            org.bytedeco.javacpp.tools.Token r14 = r14.get(r9)
            java.lang.Object[] r15 = new java.lang.Object[r9]
            r24 = 5
            java.lang.Integer r27 = java.lang.Integer.valueOf(r24)
            r15[r19] = r27
            boolean r14 = r14.match(r15)
            if (r14 == 0) goto L_0x04bd
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r1.tokens
            org.bytedeco.javacpp.tools.Token r14 = r14.get(r9)
            r15 = 2
            java.lang.Object[] r9 = new java.lang.Object[r15]
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.NEW
            r9[r19] = r15
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.DELETE
            r24 = 1
            r9[r24] = r15
            boolean r9 = r14.match(r9)
            if (r9 == 0) goto L_0x05d0
        L_0x04bd:
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r14 = r10.cppName
            r9.append(r14)
            java.lang.String r14 = "operator "
            r9.append(r14)
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r1.tokens
            org.bytedeco.javacpp.tools.Token r14 = r14.next()
            r9.append(r14)
            java.lang.String r9 = r9.toString()
            r10.cppName = r9
        L_0x04db:
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.next()
            r9 = 2
            java.lang.Object[] r14 = new java.lang.Object[r9]
            org.bytedeco.javacpp.tools.Token r9 = org.bytedeco.javacpp.tools.Token.EOF
            r15 = 0
            r14[r15] = r9
            r9 = 40
            java.lang.Character r15 = java.lang.Character.valueOf(r9)
            r9 = 1
            r14[r9] = r15
            boolean r9 = r0.match(r14)
            if (r9 != 0) goto L_0x05e1
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r14 = r10.cppName
            r9.append(r14)
            r9.append(r0)
            java.lang.String r9 = r9.toString()
            r10.cppName = r9
            goto L_0x04db
        L_0x050c:
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            r9 = 60
            java.lang.Character r15 = java.lang.Character.valueOf(r9)
            r9 = 0
            r14[r9] = r15
            boolean r9 = r0.match(r14)
            if (r9 == 0) goto L_0x059a
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r14 = r10.cppName
            r9.append(r14)
            r9.append(r0)
            java.lang.String r9 = r9.toString()
            r10.cppName = r9
            r9 = 0
        L_0x0532:
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r14.next()
            r14 = 1
            java.lang.Object[] r15 = new java.lang.Object[r14]
            org.bytedeco.javacpp.tools.Token r14 = org.bytedeco.javacpp.tools.Token.EOF
            r19 = 0
            r15[r19] = r14
            boolean r14 = r0.match(r15)
            if (r14 != 0) goto L_0x0599
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = r10.cppName
            r14.append(r15)
            r14.append(r0)
            java.lang.String r14 = r14.toString()
            r10.cppName = r14
            if (r9 != 0) goto L_0x0570
            r14 = 1
            java.lang.Object[] r15 = new java.lang.Object[r14]
            r24 = 62
            java.lang.Character r24 = java.lang.Character.valueOf(r24)
            r19 = 0
            r15[r19] = r24
            boolean r15 = r0.match(r15)
            if (r15 == 0) goto L_0x0573
            goto L_0x0599
        L_0x0570:
            r14 = 1
            r19 = 0
        L_0x0573:
            java.lang.Object[] r15 = new java.lang.Object[r14]
            r24 = 60
            java.lang.Character r27 = java.lang.Character.valueOf(r24)
            r15[r19] = r27
            boolean r15 = r0.match(r15)
            if (r15 == 0) goto L_0x0586
            int r9 = r9 + 1
            goto L_0x0532
        L_0x0586:
            java.lang.Object[] r15 = new java.lang.Object[r14]
            r14 = 62
            java.lang.Character r14 = java.lang.Character.valueOf(r14)
            r15[r19] = r14
            boolean r14 = r0.match(r15)
            if (r14 == 0) goto L_0x0532
            int r9 = r9 + -1
            goto L_0x0532
        L_0x0599:
            goto L_0x05d0
        L_0x059a:
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            r9 = 5
            java.lang.Integer r15 = java.lang.Integer.valueOf(r9)
            r9 = 0
            r14[r9] = r15
            boolean r9 = r0.match(r14)
            if (r9 == 0) goto L_0x05e1
            java.lang.String r9 = r10.cppName
            int r9 = r9.length()
            if (r9 == 0) goto L_0x05bd
            java.lang.String r9 = r10.cppName
            java.lang.String r14 = "::"
            boolean r9 = r9.endsWith(r14)
            if (r9 == 0) goto L_0x05e1
        L_0x05bd:
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r14 = r10.cppName
            r9.append(r14)
            r9.append(r0)
            java.lang.String r9 = r9.toString()
            r10.cppName = r9
        L_0x05d0:
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.next()
            r35 = r13
            r14 = r42
            r15 = 1
            goto L_0x0377
        L_0x05dd:
            r42 = r14
            r13 = r35
        L_0x05e1:
            r9 = r25
            r4 = r38
            goto L_0x07f3
        L_0x05e7:
            r42 = r14
            r13 = r35
            r9 = r25
            r4 = r38
            goto L_0x07f3
        L_0x05f1:
            r39 = r9
            r42 = r14
            r13 = r35
        L_0x05f7:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r9 = 1
            java.lang.Object[] r14 = new java.lang.Object[r9]
            r9 = 40
            java.lang.Character r15 = java.lang.Character.valueOf(r9)
            r9 = 0
            r14[r9] = r15
            boolean r0 = r0.match(r14)
            if (r0 == 0) goto L_0x0614
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
        L_0x0614:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r9 = r25
            r15 = r38
        L_0x061e:
            r14 = r0
            r43 = r15
            r15 = 1
            java.lang.Object[] r0 = new java.lang.Object[r15]
            org.bytedeco.javacpp.tools.Token r24 = org.bytedeco.javacpp.tools.Token.EOF
            r19 = 0
            r0[r19] = r24
            boolean r0 = r14.match(r0)
            if (r0 != 0) goto L_0x07d4
            r15 = 3
            java.lang.Object[] r0 = new java.lang.Object[r15]
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.CONST
            r0[r19] = r15
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.__CONST
            r4 = 1
            r0[r4] = r15
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.CONSTEXPR
            r4 = 2
            r0[r4] = r15
            boolean r0 = r14.match(r0)
            if (r0 == 0) goto L_0x064c
            r15 = 1
            r10.constPointer = r15
            goto L_0x07c3
        L_0x064c:
            r15 = 1
            java.lang.Object[] r0 = new java.lang.Object[r4]
            r4 = 5
            java.lang.Integer r24 = java.lang.Integer.valueOf(r4)
            r4 = 0
            r0[r4] = r24
            java.lang.String r4 = "::"
            r0[r15] = r4
            boolean r0 = r14.match(r0)
            if (r0 == 0) goto L_0x06a7
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r0 = r0.index
            org.bytedeco.javacpp.tools.Attribute r4 = r82.attribute()
            if (r4 == 0) goto L_0x068e
            boolean r15 = r4.annotation
            if (r15 == 0) goto L_0x068e
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            java.lang.String r7 = r12.annotations
            r15.append(r7)
            java.lang.String r7 = r4.javaName
            r15.append(r7)
            java.lang.String r7 = r15.toString()
            r12.annotations = r7
            r5.add(r4)
            r7 = r4
            r9 = r7
            r15 = r43
            goto L_0x07ca
        L_0x068e:
            org.bytedeco.javacpp.tools.TokenIndexer r7 = r1.tokens
            r7.index = r0
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r15 = r10.cppName
            r7.append(r15)
            r7.append(r14)
            java.lang.String r7 = r7.toString()
            r10.cppName = r7
            goto L_0x07c3
        L_0x06a7:
            r4 = 1
            java.lang.Object[] r0 = new java.lang.Object[r4]
            r4 = 42
            java.lang.Character r7 = java.lang.Character.valueOf(r4)
            r4 = 0
            r0[r4] = r7
            boolean r0 = r14.match(r0)
            if (r0 == 0) goto L_0x0760
            int r36 = r36 + 1
            java.lang.String r0 = r10.cppName
            java.lang.String r4 = "::"
            boolean r0 = r0.endsWith(r4)
            if (r0 == 0) goto L_0x0727
            java.lang.String r0 = r10.cppName
            java.lang.String r4 = r10.cppName
            int r4 = r4.length()
            r7 = 2
            int r4 = r4 - r7
            r7 = 0
            java.lang.String r0 = r0.substring(r7, r4)
            r10.cppName = r0
            java.lang.String r0 = r10.cppName
            java.lang.String[] r0 = r2.qualify(r0)
            int r4 = r0.length
            r7 = 0
        L_0x06de:
            if (r7 >= r4) goto L_0x0705
            r15 = r0[r7]
            r44 = r0
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            r45 = r4
            r4 = 0
            org.bytedeco.javacpp.tools.Info r0 = r0.getFirst(r15, r4)
            r43 = r0
            if (r0 == 0) goto L_0x06f4
            r10.cppName = r15
            goto L_0x0705
        L_0x06f4:
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r0 = r0.getFirst(r15)
            if (r0 == 0) goto L_0x06fe
            r10.cppName = r15
        L_0x06fe:
            int r7 = r7 + 1
            r0 = r44
            r4 = r45
            goto L_0x06de
        L_0x0705:
            r15 = r43
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r4 = r13.text
            r0.append(r4)
            java.lang.String r4 = "@Namespace(\""
            r0.append(r4)
            java.lang.String r4 = r10.cppName
            r0.append(r4)
            java.lang.String r4 = "\") "
            r0.append(r4)
            java.lang.String r0 = r0.toString()
            r13.text = r0
            goto L_0x075b
        L_0x0727:
            if (r9 != 0) goto L_0x0735
            java.lang.String r0 = r10.cppName
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x0732
            goto L_0x0735
        L_0x0732:
            r15 = r43
            goto L_0x075b
        L_0x0735:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r4 = r13.text
            r0.append(r4)
            java.lang.String r4 = "@Convention(\""
            r0.append(r4)
            if (r9 == 0) goto L_0x0749
            java.lang.String r4 = r9.cppName
            goto L_0x074b
        L_0x0749:
            java.lang.String r4 = r10.cppName
        L_0x074b:
            r0.append(r4)
            java.lang.String r4 = "\") "
            r0.append(r4)
            java.lang.String r0 = r0.toString()
            r13.text = r0
            r9 = 0
            goto L_0x0732
        L_0x075b:
            java.lang.String r0 = ""
            r10.cppName = r0
            goto L_0x07c5
        L_0x0760:
            r4 = 1
            java.lang.Object[] r0 = new java.lang.Object[r4]
            java.lang.Character r7 = java.lang.Character.valueOf(r20)
            r15 = 0
            r0[r15] = r7
            boolean r0 = r14.match(r0)
            if (r0 == 0) goto L_0x07a7
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r4)
            r4 = r0
            int r0 = r10.indices     // Catch:{ NumberFormatException -> 0x079c }
            int r7 = r0 + 1
            r10.indices = r7     // Catch:{ NumberFormatException -> 0x079c }
            r7 = 1
            java.lang.Object[] r15 = new java.lang.Object[r7]     // Catch:{ NumberFormatException -> 0x079c }
            java.lang.Integer r25 = java.lang.Integer.valueOf(r7)     // Catch:{ NumberFormatException -> 0x079c }
            r7 = 0
            r15[r7] = r25     // Catch:{ NumberFormatException -> 0x079c }
            boolean r7 = r4.match(r15)     // Catch:{ NumberFormatException -> 0x079c }
            if (r7 == 0) goto L_0x0798
            java.lang.String r7 = r4.value     // Catch:{ NumberFormatException -> 0x079c }
            java.lang.Integer r7 = java.lang.Integer.decode(r7)     // Catch:{ NumberFormatException -> 0x079c }
            int r7 = r7.intValue()     // Catch:{ NumberFormatException -> 0x079c }
            goto L_0x0799
        L_0x0798:
            r7 = -1
        L_0x0799:
            r8[r0] = r7     // Catch:{ NumberFormatException -> 0x079c }
            goto L_0x07a6
        L_0x079c:
            r0 = move-exception
            int r7 = r10.indices
            int r15 = r7 + 1
            r10.indices = r15
            r15 = -1
            r8[r7] = r15
        L_0x07a6:
            goto L_0x07c3
        L_0x07a7:
            r4 = 2
            java.lang.Object[] r0 = new java.lang.Object[r4]
            r4 = 40
            java.lang.Character r7 = java.lang.Character.valueOf(r4)
            r4 = 0
            r0[r4] = r7
            r4 = 41
            java.lang.Character r7 = java.lang.Character.valueOf(r4)
            r4 = 1
            r0[r4] = r7
            boolean r0 = r14.match(r0)
            if (r0 == 0) goto L_0x07c3
            goto L_0x07d4
        L_0x07c3:
            r15 = r43
        L_0x07c5:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
        L_0x07ca:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r4 = r85
            goto L_0x061e
        L_0x07d4:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r4 = 1
            java.lang.Object[] r7 = new java.lang.Object[r4]
            r4 = 41
            java.lang.Character r14 = java.lang.Character.valueOf(r4)
            r4 = 0
            r7[r4] = r14
            boolean r0 = r0.match(r7)
            if (r0 == 0) goto L_0x07f1
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
        L_0x07f1:
            r4 = r43
        L_0x07f3:
            java.lang.String r0 = r10.cppName
            int r0 = r0.length()
            if (r0 != 0) goto L_0x07fd
            r10.cppName = r3
        L_0x07fd:
            r0 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r7 = r1.tokens
            org.bytedeco.javacpp.tools.Token r7 = r7.get()
        L_0x0804:
            r14 = 1
            java.lang.Object[] r15 = new java.lang.Object[r14]
            org.bytedeco.javacpp.tools.Token r24 = org.bytedeco.javacpp.tools.Token.EOF
            r19 = 0
            r15[r19] = r24
            boolean r15 = r7.match(r15)
            if (r15 != 0) goto L_0x088e
            if (r0 != 0) goto L_0x086c
            java.lang.Object[] r15 = new java.lang.Object[r14]
            java.lang.Character r24 = java.lang.Character.valueOf(r20)
            r15[r19] = r24
            boolean r15 = r7.match(r15)
            if (r15 == 0) goto L_0x086c
            r15 = 1
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r14)
            r14 = r0
            int r0 = r10.indices     // Catch:{ NumberFormatException -> 0x085a }
            r46 = r5
            int r5 = r0 + 1
            r10.indices = r5     // Catch:{ NumberFormatException -> 0x0856 }
            r47 = r15
            r5 = 1
            java.lang.Object[] r15 = new java.lang.Object[r5]     // Catch:{ NumberFormatException -> 0x0854 }
            java.lang.Integer r25 = java.lang.Integer.valueOf(r5)     // Catch:{ NumberFormatException -> 0x0854 }
            r5 = 0
            r15[r5] = r25     // Catch:{ NumberFormatException -> 0x0854 }
            boolean r5 = r14.match(r15)     // Catch:{ NumberFormatException -> 0x0854 }
            if (r5 == 0) goto L_0x0850
            java.lang.String r5 = r14.value     // Catch:{ NumberFormatException -> 0x0854 }
            java.lang.Integer r5 = java.lang.Integer.decode(r5)     // Catch:{ NumberFormatException -> 0x0854 }
            int r5 = r5.intValue()     // Catch:{ NumberFormatException -> 0x0854 }
            goto L_0x0851
        L_0x0850:
            r5 = -1
        L_0x0851:
            r8[r0] = r5     // Catch:{ NumberFormatException -> 0x0854 }
            goto L_0x0868
        L_0x0854:
            r0 = move-exception
            goto L_0x085f
        L_0x0856:
            r0 = move-exception
            r47 = r15
            goto L_0x085f
        L_0x085a:
            r0 = move-exception
            r46 = r5
            r47 = r15
        L_0x085f:
            int r5 = r10.indices
            int r15 = r5 + 1
            r10.indices = r15
            r15 = -1
            r8[r5] = r15
        L_0x0868:
            r0 = r47
            goto L_0x0884
        L_0x086c:
            r46 = r5
            if (r0 != 0) goto L_0x0871
            goto L_0x0890
        L_0x0871:
            if (r0 == 0) goto L_0x0884
            r5 = 1
            java.lang.Object[] r14 = new java.lang.Object[r5]
            java.lang.Character r5 = java.lang.Character.valueOf(r18)
            r15 = 0
            r14[r15] = r5
            boolean r5 = r7.match(r14)
            if (r5 == 0) goto L_0x0884
            r0 = 0
        L_0x0884:
            org.bytedeco.javacpp.tools.TokenIndexer r5 = r1.tokens
            org.bytedeco.javacpp.tools.Token r7 = r5.next()
            r5 = r46
            goto L_0x0804
        L_0x088e:
            r46 = r5
        L_0x0890:
            r5 = r36
        L_0x0892:
            int r7 = r10.indices
            if (r7 <= 0) goto L_0x08a4
            if (r5 <= 0) goto L_0x08a4
            int r7 = r10.indices
            int r14 = r7 + 1
            r10.indices = r14
            r14 = -1
            r8[r7] = r14
            int r5 = r5 + -1
            goto L_0x0892
        L_0x08a4:
            if (r88 == 0) goto L_0x093f
            int r7 = r10.indices
            if (r7 <= 0) goto L_0x093f
            int r7 = r10.indirections
            r14 = 1
            int r7 = r7 + r14
            r10.indirections = r7
            java.lang.String r7 = ""
            r14 = r7
            r7 = 1
        L_0x08b4:
            int r15 = r10.indices
            if (r7 >= r15) goto L_0x08e2
            r15 = r8[r7]
            if (r15 <= 0) goto L_0x08db
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r15.append(r14)
            r48 = r0
            java.lang.String r0 = "["
            r15.append(r0)
            r0 = r8[r7]
            r15.append(r0)
            java.lang.String r0 = "]"
            r15.append(r0)
            java.lang.String r0 = r15.toString()
            r14 = r0
            goto L_0x08dd
        L_0x08db:
            r48 = r0
        L_0x08dd:
            int r7 = r7 + 1
            r0 = r48
            goto L_0x08b4
        L_0x08e2:
            r48 = r0
            boolean r0 = r14.isEmpty()
            if (r0 != 0) goto L_0x092d
            r7 = 0
            r0 = r8[r7]
            r15 = -1
            if (r0 == r15) goto L_0x090c
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            java.lang.String r15 = "(* /*["
            r0.append(r15)
            r15 = r8[r7]
            r0.append(r15)
            java.lang.String r7 = "]*/ )"
            r0.append(r7)
            java.lang.String r0 = r0.toString()
            goto L_0x091d
        L_0x090c:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            java.lang.String r7 = "(*)"
            r0.append(r7)
            java.lang.String r0 = r0.toString()
        L_0x091d:
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r0)
            r7.append(r14)
            java.lang.String r11 = r7.toString()
            goto L_0x0941
        L_0x092d:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            java.lang.String r7 = "*"
            r0.append(r7)
            java.lang.String r11 = r0.toString()
            goto L_0x0941
        L_0x093f:
            r48 = r0
        L_0x0941:
            if (r89 == 0) goto L_0x0966
            int r0 = r10.indirections
            boolean r14 = r12.anonymous
            r15 = 1
            r14 = r14 ^ r15
            if (r0 <= r14) goto L_0x0964
            int r0 = r10.indices
            int r14 = r0 + 1
            r10.indices = r14
            r14 = -1
            r8[r0] = r14
            int r0 = r10.indirections
            int r0 = r0 - r15
            r10.indirections = r0
            int r0 = r11.length()
            int r0 = r0 - r15
            r14 = 0
            java.lang.String r11 = r11.substring(r14, r0)
            goto L_0x0968
        L_0x0964:
            r14 = 0
            goto L_0x0968
        L_0x0966:
            r14 = 0
            r15 = 1
        L_0x0968:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r14 = new java.lang.Object[r15]
            r15 = 58
            java.lang.Character r15 = java.lang.Character.valueOf(r15)
            r19 = 0
            r14[r19] = r15
            boolean r0 = r0.match(r14)
            if (r0 == 0) goto L_0x09d2
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r14 = r12.annotations
            r0.append(r14)
            java.lang.String r14 = "@NoOffset "
            r0.append(r14)
            java.lang.String r0 = r0.toString()
            r12.annotations = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            r14 = 2
            java.lang.Object[] r15 = new java.lang.Object[r14]
            r24 = 1
            java.lang.Integer r22 = java.lang.Integer.valueOf(r24)
            r19 = 0
            r15[r19] = r22
            r22 = 5
            java.lang.Integer r25 = java.lang.Integer.valueOf(r22)
            r15[r24] = r25
            r0.expect(r15)
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r15 = new java.lang.Object[r14]
            java.lang.Character r14 = java.lang.Character.valueOf(r17)
            r15[r19] = r14
            java.lang.Character r14 = java.lang.Character.valueOf(r16)
            r15[r24] = r14
            r0.expect(r15)
            java.lang.String r0 = r10.cppName
            if (r0 != 0) goto L_0x09d2
            java.lang.String r0 = ""
            r10.cppName = r0
        L_0x09d2:
            r15 = r86
            r14 = r85
            org.bytedeco.javacpp.tools.Parameters r0 = r1.parameters(r2, r14, r15)
            r10.parameters = r0
            java.lang.String r0 = r12.cppName
            java.lang.String r15 = "void"
            boolean r0 = r0.equals(r15)
            if (r0 == 0) goto L_0x0a2d
            r15 = 1
            if (r5 != r15) goto L_0x0a2d
            if (r42 != 0) goto L_0x0a2d
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r15)
            r49 = r8
            java.lang.Object[] r8 = new java.lang.Object[r15]
            r16 = 40
            java.lang.Character r17 = java.lang.Character.valueOf(r16)
            r19 = 0
            r8[r19] = r17
            boolean r0 = r0.match(r8)
            if (r0 == 0) goto L_0x0a2f
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r8 = new java.lang.Object[r15]
            java.lang.Character r17 = java.lang.Character.valueOf(r16)
            r8[r19] = r17
            r0.expect(r8)
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r8 = new java.lang.Object[r15]
            r15 = 5
            java.lang.Integer r15 = java.lang.Integer.valueOf(r15)
            r8[r19] = r15
            r0.expect(r8)
            r82.type(r83)
            r5 = 0
            goto L_0x0a86
        L_0x0a2d:
            r49 = r8
        L_0x0a2f:
            r8 = 1
            if (r5 != r8) goto L_0x0a86
            if (r42 != 0) goto L_0x0a86
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r8)
            java.lang.Object[] r15 = new java.lang.Object[r8]
            java.lang.Character r16 = java.lang.Character.valueOf(r20)
            r17 = 0
            r15[r17] = r16
            boolean r0 = r0.match(r15)
            if (r0 == 0) goto L_0x0a86
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r15 = new java.lang.Object[r8]
            java.lang.Character r16 = java.lang.Character.valueOf(r20)
            r15[r17] = r16
            r0.expect(r15)
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r15 = new java.lang.Object[r8]
            r16 = 5
            java.lang.Integer r16 = java.lang.Integer.valueOf(r16)
            r15[r17] = r16
            r0.expect(r15)
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r15 = new java.lang.Object[r8]
            java.lang.Character r16 = java.lang.Character.valueOf(r18)
            r15[r17] = r16
            r0.expect(r15)
            int r0 = r10.indirections
            int r0 = r0 + r8
            r10.indirections = r0
            int r5 = r5 + -1
        L_0x0a86:
            r0 = 1
            r8 = 0
            if (r88 == 0) goto L_0x0a91
            int r15 = r10.indices
            r6 = 1
            if (r15 <= r6) goto L_0x0a91
            r6 = 1
            goto L_0x0a92
        L_0x0a91:
            r6 = 0
        L_0x0a92:
            r15 = 0
            r50 = r0
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            r51 = r6
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r52 = r8
            java.lang.String r8 = "const "
            r6.append(r8)
            java.lang.String r8 = r12.cppName
            r6.append(r8)
            java.lang.String r6 = r6.toString()
            r8 = 0
            org.bytedeco.javacpp.tools.Info r0 = r0.getFirst(r6, r8)
            boolean r6 = r12.constValue
            if (r6 == 0) goto L_0x0ac4
            int r6 = r10.indirections
            r8 = 2
            if (r6 >= r8) goto L_0x0ac4
            boolean r6 = r10.reference
            if (r6 != 0) goto L_0x0ac4
            r6 = r0
            r53 = r15
            goto L_0x0acf
        L_0x0ac4:
            org.bytedeco.javacpp.tools.InfoMap r6 = r1.infoMap
            java.lang.String r8 = r12.cppName
            r53 = r15
            r15 = 0
            org.bytedeco.javacpp.tools.Info r6 = r6.getFirst(r8, r15)
        L_0x0acf:
            if (r42 == 0) goto L_0x0adc
            org.bytedeco.javacpp.tools.Parameters r8 = r10.parameters
            if (r8 == 0) goto L_0x0ad6
            goto L_0x0adc
        L_0x0ad6:
            r54 = r0
            r56 = r6
            goto L_0x0c63
        L_0x0adc:
            if (r0 == 0) goto L_0x0ae7
            java.lang.String[] r8 = r0.cppTypes
            if (r8 == 0) goto L_0x0ad6
            java.lang.String[] r8 = r0.cppTypes
            int r8 = r8.length
            if (r8 <= 0) goto L_0x0ad6
        L_0x0ae7:
            if (r6 == 0) goto L_0x0af2
            java.lang.String[] r8 = r6.cppTypes
            if (r8 == 0) goto L_0x0ad6
            java.lang.String[] r8 = r6.cppTypes
            int r8 = r8.length
            if (r8 <= 0) goto L_0x0ad6
        L_0x0af2:
            r8 = r12
            if (r6 == 0) goto L_0x0b07
            org.bytedeco.javacpp.tools.Parser r15 = new org.bytedeco.javacpp.tools.Parser
            r54 = r0
            java.lang.String[] r0 = r6.cppTypes
            r16 = 0
            r0 = r0[r16]
            r15.<init>((org.bytedeco.javacpp.tools.Parser) r1, (java.lang.String) r0)
            org.bytedeco.javacpp.tools.Type r8 = r15.type(r2)
            goto L_0x0b09
        L_0x0b07:
            r54 = r0
        L_0x0b09:
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            java.lang.String r15 = r8.cppName
            java.util.List r0 = r0.get(r15)
            java.util.Iterator r15 = r0.iterator()
        L_0x0b15:
            boolean r16 = r15.hasNext()
            if (r16 == 0) goto L_0x0c61
            java.lang.Object r16 = r15.next()
            r55 = r0
            r0 = r16
            org.bytedeco.javacpp.tools.Info r0 = (org.bytedeco.javacpp.tools.Info) r0
            r56 = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            if (r6 == 0) goto L_0x0c57
            java.lang.String[] r6 = r0.annotations
            if (r6 == 0) goto L_0x0c57
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r15 = 0
            r6 = r6[r15]
            boolean r6 = r6.constPointer
            r12.constPointer = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            boolean r6 = r6.constValue
            r12.constValue = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            boolean r6 = r6.simple
            r12.simple = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            int r6 = r6.indirections
            r12.indirections = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            boolean r6 = r6.reference
            r12.reference = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            java.lang.String r6 = r6.annotations
            r12.annotations = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            java.lang.String r6 = r6.cppName
            r12.cppName = r6
            org.bytedeco.javacpp.tools.Type[] r6 = r8.arguments
            r6 = r6[r15]
            java.lang.String r6 = r6.javaName
            r12.javaName = r6
            r6 = 1
            r10.indirections = r6
            r10.reference = r15
            boolean r6 = r2.virtualize
            if (r6 == 0) goto L_0x0b7e
            r6 = 1
            r15 = r11
            r26 = r15
            goto L_0x0b80
        L_0x0b7e:
            r6 = r51
        L_0x0b80:
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r57 = r6
            java.lang.String r6 = r12.cppName
            r15.append(r6)
            java.lang.String r6 = "*"
            r15.append(r6)
            java.lang.String r6 = r15.toString()
            boolean r11 = r12.constValue
            if (r11 == 0) goto L_0x0bb2
            java.lang.String r11 = "const "
            boolean r11 = r6.startsWith(r11)
            if (r11 != 0) goto L_0x0bb2
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r15 = "const "
            r11.append(r15)
            r11.append(r6)
            java.lang.String r6 = r11.toString()
        L_0x0bb2:
            boolean r11 = r12.constPointer
            if (r11 == 0) goto L_0x0bcf
            java.lang.String r11 = " const"
            boolean r11 = r6.endsWith(r11)
            if (r11 != 0) goto L_0x0bcf
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r6)
            java.lang.String r15 = " const"
            r11.append(r15)
            java.lang.String r6 = r11.toString()
        L_0x0bcf:
            int r11 = r12.indirections
            if (r11 <= 0) goto L_0x0bfb
            int r11 = r10.indirections
            int r15 = r12.indirections
            int r11 = r11 + r15
            r10.indirections = r11
            r11 = r6
            r6 = 0
        L_0x0bdc:
            int r15 = r12.indirections
            if (r6 >= r15) goto L_0x0bf8
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r15.append(r11)
            r58 = r8
            java.lang.String r8 = "*"
            r15.append(r8)
            java.lang.String r11 = r15.toString()
            int r6 = r6 + 1
            r8 = r58
            goto L_0x0bdc
        L_0x0bf8:
            r58 = r8
            goto L_0x0bfe
        L_0x0bfb:
            r58 = r8
            r11 = r6
        L_0x0bfe:
            boolean r6 = r12.reference
            if (r6 == 0) goto L_0x0c17
            r6 = 1
            r10.reference = r6
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r11)
            java.lang.String r8 = "&"
            r6.append(r8)
            java.lang.String r6 = r6.toString()
            r11 = r6
        L_0x0c17:
            java.lang.String[] r6 = r0.annotations
            int r8 = r6.length
            r15 = 0
        L_0x0c1b:
            if (r15 >= r8) goto L_0x0c46
            r59 = r0
            r0 = r6[r15]
            r60 = r6
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r61 = r8
            java.lang.String r8 = r12.annotations
            r6.append(r8)
            r6.append(r0)
            java.lang.String r8 = " "
            r6.append(r8)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            int r15 = r15 + 1
            r0 = r59
            r6 = r60
            r8 = r61
            goto L_0x0c1b
        L_0x0c46:
            r59 = r0
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            java.lang.String r6 = r12.cppName
            r8 = 0
            org.bytedeco.javacpp.tools.Info r6 = r0.getFirst(r6, r8)
            r0 = r26
            r51 = r57
            goto L_0x0c67
        L_0x0c57:
            r58 = r8
            r0 = r55
            r6 = r56
            r8 = r58
            goto L_0x0b15
        L_0x0c61:
            r56 = r6
        L_0x0c63:
            r0 = r26
            r6 = r56
        L_0x0c67:
            if (r39 != 0) goto L_0x0d03
            if (r6 == 0) goto L_0x0d03
            boolean r8 = r6.enumerate
            if (r8 != 0) goto L_0x0c73
            java.lang.String[] r8 = r6.valueTypes
            if (r8 == 0) goto L_0x0c8d
        L_0x0c73:
            boolean r8 = r12.constValue
            if (r8 == 0) goto L_0x0c7f
            int r8 = r10.indirections
            if (r8 != 0) goto L_0x0c7f
            boolean r8 = r10.reference
            if (r8 != 0) goto L_0x0c8b
        L_0x0c7f:
            int r8 = r10.indirections
            if (r8 != 0) goto L_0x0c87
            boolean r8 = r10.reference
            if (r8 == 0) goto L_0x0c8b
        L_0x0c87:
            java.lang.String[] r8 = r6.pointerTypes
            if (r8 != 0) goto L_0x0c8d
        L_0x0c8b:
            r8 = 1
            goto L_0x0c8e
        L_0x0c8d:
            r8 = 0
        L_0x0c8e:
            java.lang.String[] r15 = r6.cppNames
            r16 = 0
            r15 = r15[r16]
            java.lang.String r7 = "const "
            boolean r7 = r15.startsWith(r7)
            if (r7 == 0) goto L_0x0ca2
            boolean r7 = r6.define
            if (r7 != 0) goto L_0x0ca2
            r7 = 1
            goto L_0x0ca3
        L_0x0ca2:
            r7 = 0
        L_0x0ca3:
            r15 = r7
            if (r8 == 0) goto L_0x0cb0
            java.lang.String[] r7 = r6.valueTypes
            if (r7 == 0) goto L_0x0cae
            java.lang.String[] r7 = r6.valueTypes
            int r7 = r7.length
            goto L_0x0cb7
        L_0x0cae:
            r7 = 1
            goto L_0x0cb7
        L_0x0cb0:
            java.lang.String[] r7 = r6.pointerTypes
            if (r7 == 0) goto L_0x0cae
            java.lang.String[] r7 = r6.pointerTypes
            int r7 = r7.length
        L_0x0cb7:
            if (r14 >= 0) goto L_0x0cbd
            r62 = r7
            r7 = 0
            goto L_0x0cc3
        L_0x0cbd:
            int r16 = r14 % r7
            r62 = r7
            r7 = r16
        L_0x0cc3:
            r10.infoNumber = r7
            if (r8 == 0) goto L_0x0cd9
            java.lang.String[] r7 = r6.valueTypes
            if (r7 == 0) goto L_0x0cd4
            java.lang.String[] r7 = r6.valueTypes
            r63 = r8
            int r8 = r10.infoNumber
            r7 = r7[r8]
            goto L_0x0ce5
        L_0x0cd4:
            r63 = r8
        L_0x0cd6:
            java.lang.String r7 = r12.javaName
            goto L_0x0ce5
        L_0x0cd9:
            r63 = r8
            java.lang.String[] r7 = r6.pointerTypes
            if (r7 == 0) goto L_0x0cd6
            java.lang.String[] r7 = r6.pointerTypes
            int r8 = r10.infoNumber
            r7 = r7[r8]
        L_0x0ce5:
            r12.javaName = r7
            java.lang.String r7 = r12.javaName
            java.lang.String r7 = r2.shorten(r7)
            r12.javaName = r7
            boolean r7 = r6.cast
            if (r7 == 0) goto L_0x0cff
            java.lang.String r7 = r12.cppName
            java.lang.String r8 = r12.javaName
            boolean r7 = r7.equals(r8)
            if (r7 != 0) goto L_0x0cff
            r7 = 1
            goto L_0x0d00
        L_0x0cff:
            r7 = 0
        L_0x0d00:
            r51 = r51 | r7
            goto L_0x0d09
        L_0x0d03:
            r62 = r50
            r63 = r52
            r15 = r53
        L_0x0d09:
            if (r63 == 0) goto L_0x0d14
            boolean r7 = r2.virtualize
            if (r7 == 0) goto L_0x0d10
            goto L_0x0d14
        L_0x0d10:
            r64 = r6
            goto L_0x0e90
        L_0x0d14:
            if (r63 != 0) goto L_0x0d37
            int r7 = r10.indirections
            if (r7 != 0) goto L_0x0d37
            boolean r7 = r10.reference
            if (r7 != 0) goto L_0x0d37
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = r12.annotations
            r7.append(r8)
            java.lang.String r8 = "@ByVal "
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r12.annotations = r7
            r64 = r6
            goto L_0x0e37
        L_0x0d37:
            int r7 = r10.indirections
            if (r7 != 0) goto L_0x0d72
            boolean r7 = r10.reference
            if (r7 == 0) goto L_0x0d72
            java.lang.String r7 = r12.javaName
            java.lang.String r8 = "@ByPtrPtr "
            boolean r7 = r7.contains(r8)
            if (r7 == 0) goto L_0x0d59
            java.lang.String r7 = r12.javaName
            java.lang.String r8 = "@ByPtrPtr "
            r64 = r6
            java.lang.String r6 = "@ByPtrRef "
            java.lang.String r6 = r7.replace(r8, r6)
            r12.javaName = r6
            goto L_0x0e37
        L_0x0d59:
            r64 = r6
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r7 = "@ByRef "
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            goto L_0x0e37
        L_0x0d72:
            r64 = r6
            java.lang.String r6 = r12.javaName
            java.lang.String r7 = "@ByPtrRef "
            boolean r6 = r6.contains(r7)
            if (r6 != 0) goto L_0x0d9e
            int r6 = r10.indirections
            r7 = 1
            if (r6 != r7) goto L_0x0d9e
            boolean r6 = r10.reference
            if (r6 == 0) goto L_0x0d9e
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r7 = "@ByPtrRef "
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            goto L_0x0e37
        L_0x0d9e:
            java.lang.String r6 = r12.javaName
            java.lang.String r7 = "@ByPtrPtr "
            boolean r6 = r6.contains(r7)
            if (r6 != 0) goto L_0x0ddd
            int r6 = r10.indirections
            r7 = 2
            if (r6 != r7) goto L_0x0ddd
            boolean r6 = r10.reference
            if (r6 != 0) goto L_0x0ddd
            if (r14 >= 0) goto L_0x0dbd
            java.lang.String r6 = r12.javaName
            java.lang.String r7 = "PointerPointer"
            boolean r6 = r6.equals(r7)
            if (r6 == 0) goto L_0x0ddd
        L_0x0dbd:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r7 = "@ByPtrPtr "
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            java.lang.String r6 = r12.cppName
            java.lang.String r7 = "void"
            boolean r6 = r6.equals(r7)
            r51 = r51 | r6
            goto L_0x0e37
        L_0x0ddd:
            int r6 = r10.indirections
            r7 = 2
            if (r6 < r7) goto L_0x0e37
            int r6 = r10.infoNumber
            int r6 = r6 + r62
            r10.infoNumber = r6
            r51 = 1
            java.lang.String r6 = r12.javaName
            java.lang.String r7 = "@ByPtrRef "
            boolean r6 = r6.contains(r7)
            if (r6 != 0) goto L_0x0e1e
            boolean r6 = r10.reference
            if (r6 == 0) goto L_0x0df9
            goto L_0x0e1e
        L_0x0df9:
            java.lang.String r6 = r12.javaName
            java.lang.String r7 = "@ByPtrPtr "
            boolean r6 = r6.contains(r7)
            if (r6 != 0) goto L_0x0e08
            int r6 = r10.indirections
            r7 = 3
            if (r6 < r7) goto L_0x0e33
        L_0x0e08:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r7 = "@ByPtrPtr "
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            goto L_0x0e33
        L_0x0e1e:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r7 = "@ByRef "
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
        L_0x0e33:
            java.lang.String r6 = "PointerPointer"
            r12.javaName = r6
        L_0x0e37:
            if (r51 != 0) goto L_0x0e90
            java.lang.String r6 = r12.javaName
            java.lang.String r7 = "@Cast"
            boolean r6 = r6.contains(r7)
            if (r6 != 0) goto L_0x0e90
            boolean r6 = r12.constValue
            if (r6 == 0) goto L_0x0e63
            if (r15 != 0) goto L_0x0e63
            boolean r6 = r12.constPointer
            if (r6 != 0) goto L_0x0e63
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "@Const "
            r6.append(r7)
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            goto L_0x0e90
        L_0x0e63:
            boolean r6 = r12.constPointer
            if (r6 == 0) goto L_0x0e90
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "@Const({"
            r6.append(r7)
            boolean r7 = r12.constValue
            r6.append(r7)
            java.lang.String r7 = ", "
            r6.append(r7)
            boolean r7 = r12.constPointer
            r6.append(r7)
            java.lang.String r7 = "}) "
            r6.append(r7)
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
        L_0x0e90:
            if (r51 == 0) goto L_0x0f42
            int r6 = r10.indirections
            if (r6 != 0) goto L_0x0ea2
            boolean r6 = r10.reference
            if (r6 == 0) goto L_0x0ea2
            r6 = 38
            r7 = 42
            java.lang.String r11 = r11.replace(r6, r7)
        L_0x0ea2:
            if (r63 == 0) goto L_0x0eb7
            boolean r6 = r12.constValue
            if (r6 == 0) goto L_0x0eb7
            boolean r6 = r10.reference
            if (r6 == 0) goto L_0x0eb7
            int r6 = r11.length()
            r7 = 1
            int r6 = r6 - r7
            r7 = 0
            java.lang.String r11 = r11.substring(r7, r6)
        L_0x0eb7:
            boolean r6 = r12.constValue
            if (r6 == 0) goto L_0x0ed5
            java.lang.String r6 = "const "
            boolean r6 = r11.startsWith(r6)
            if (r6 != 0) goto L_0x0ed5
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "const "
            r6.append(r7)
            r6.append(r11)
            java.lang.String r6 = r6.toString()
            r11 = r6
        L_0x0ed5:
            if (r0 == 0) goto L_0x0efd
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "@Cast({\""
            r6.append(r7)
            r6.append(r11)
            java.lang.String r7 = "\", \""
            r6.append(r7)
            r6.append(r0)
            java.lang.String r7 = "\"}) "
            r6.append(r7)
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            goto L_0x0f42
        L_0x0efd:
            if (r63 != 0) goto L_0x0f25
            int r6 = r10.indirections
            if (r6 != 0) goto L_0x0f25
            boolean r6 = r10.reference
            if (r6 != 0) goto L_0x0f25
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r7 = "@Cast(\""
            r6.append(r7)
            r6.append(r11)
            java.lang.String r7 = "*\") "
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
            goto L_0x0f42
        L_0x0f25:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "@Cast(\""
            r6.append(r7)
            r6.append(r11)
            java.lang.String r7 = "\") "
            r6.append(r7)
            java.lang.String r7 = r12.annotations
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r12.annotations = r6
        L_0x0f42:
            r6 = 0
            if (r34 == 0) goto L_0x0f4a
            r7 = r34
            java.lang.String r8 = r7.arguments
            goto L_0x0f4e
        L_0x0f4a:
            r7 = r34
            java.lang.String r8 = r10.cppName
        L_0x0f4e:
            r10.javaName = r8
            if (r3 != 0) goto L_0x0f91
            java.lang.String r8 = r10.cppName
            java.lang.String[] r8 = r2.qualify(r8)
            r65 = r0
            int r0 = r8.length
            r16 = r6
            r6 = 0
        L_0x0f5e:
            if (r6 >= r0) goto L_0x0f8c
            r66 = r0
            r0 = r8[r6]
            r67 = r8
            org.bytedeco.javacpp.tools.InfoMap r8 = r1.infoMap
            r68 = r11
            r11 = 0
            org.bytedeco.javacpp.tools.Info r8 = r8.getFirst(r0, r11)
            r16 = r8
            if (r8 == 0) goto L_0x0f79
            r10.cppName = r0
            r6 = r16
            goto L_0x0f95
        L_0x0f79:
            org.bytedeco.javacpp.tools.InfoMap r8 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r8 = r8.getFirst(r0)
            if (r8 == 0) goto L_0x0f83
            r10.cppName = r0
        L_0x0f83:
            int r6 = r6 + 1
            r0 = r66
            r8 = r67
            r11 = r68
            goto L_0x0f5e
        L_0x0f8c:
            r68 = r11
            r6 = r16
            goto L_0x0f95
        L_0x0f91:
            r65 = r0
            r68 = r11
        L_0x0f95:
            if (r23 == 0) goto L_0x0f9d
            java.lang.String[] r0 = r4.pointerTypes
            r8 = 0
            r0 = r0[r8]
            goto L_0x0f9f
        L_0x0f9d:
            java.lang.String r0 = r10.javaName
        L_0x0f9f:
            if (r7 != 0) goto L_0x0fd0
            if (r3 != 0) goto L_0x0fd0
            if (r6 == 0) goto L_0x0fd0
            java.lang.String[] r8 = r6.javaNames
            if (r8 == 0) goto L_0x0fd0
            java.lang.String[] r8 = r6.javaNames
            int r8 = r8.length
            if (r8 <= 0) goto L_0x0fd0
            boolean r8 = r10.operator
            if (r8 != 0) goto L_0x0fc9
            java.lang.String[] r8 = r6.cppNames
            r11 = 0
            r8 = r8[r11]
            java.lang.String r11 = "<"
            boolean r8 = r8.contains(r11)
            if (r8 == 0) goto L_0x0fc9
            org.bytedeco.javacpp.tools.TemplateMap r8 = r2.templateMap
            if (r8 == 0) goto L_0x0fd0
            org.bytedeco.javacpp.tools.TemplateMap r8 = r2.templateMap
            org.bytedeco.javacpp.tools.Type r8 = r8.type
            if (r8 != 0) goto L_0x0fd0
        L_0x0fc9:
            java.lang.String[] r8 = r6.javaNames
            r11 = 0
            r8 = r8[r11]
            r10.javaName = r8
        L_0x0fd0:
            if (r6 == 0) goto L_0x1005
            java.lang.String[] r8 = r6.annotations
            if (r8 == 0) goto L_0x1005
            java.lang.String[] r8 = r6.annotations
            int r11 = r8.length
            r3 = 0
        L_0x0fda:
            if (r3 >= r11) goto L_0x1005
            r69 = r6
            r6 = r8[r3]
            r70 = r7
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r71 = r8
            java.lang.String r8 = r12.annotations
            r7.append(r8)
            r7.append(r6)
            java.lang.String r8 = " "
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r12.annotations = r7
            int r3 = r3 + 1
            r6 = r69
            r7 = r70
            r8 = r71
            goto L_0x0fda
        L_0x1005:
            r69 = r6
            r70 = r7
            r10.type = r12
            java.lang.String r3 = r10.javaName
            r10.signature = r3
            org.bytedeco.javacpp.tools.Parameters r3 = r10.parameters
            if (r3 != 0) goto L_0x1015
            if (r23 == 0) goto L_0x1044
        L_0x1015:
            org.bytedeco.javacpp.tools.Parameters r3 = r10.parameters
            if (r3 == 0) goto L_0x1025
            int r3 = r10.infoNumber
            org.bytedeco.javacpp.tools.Parameters r6 = r10.parameters
            int r6 = r6.infoNumber
            int r3 = java.lang.Math.max(r3, r6)
            r10.infoNumber = r3
        L_0x1025:
            org.bytedeco.javacpp.tools.Parameters r3 = r10.parameters
            if (r3 == 0) goto L_0x104c
            if (r5 != 0) goto L_0x104c
            if (r42 != 0) goto L_0x104c
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r6 = r10.signature
            r3.append(r6)
            org.bytedeco.javacpp.tools.Parameters r6 = r10.parameters
            java.lang.String r6 = r6.signature
            r3.append(r6)
            java.lang.String r3 = r3.toString()
            r10.signature = r3
        L_0x1044:
            r78 = r0
            r81 = r4
            r75 = r9
            goto L_0x144c
        L_0x104c:
            if (r9 == 0) goto L_0x106d
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r6 = r13.text
            r3.append(r6)
            java.lang.String r6 = "@Convention(\""
            r3.append(r6)
            java.lang.String r6 = r9.cppName
            r3.append(r6)
            java.lang.String r6 = "\") "
            r3.append(r6)
            java.lang.String r3 = r3.toString()
            r13.text = r3
        L_0x106d:
            java.lang.String r3 = ""
            org.bytedeco.javacpp.tools.Type r6 = r10.type
            if (r6 == 0) goto L_0x10b7
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r3)
            org.bytedeco.javacpp.tools.Type r7 = r10.type
            java.lang.String r7 = r7.cppName
            r6.append(r7)
            java.lang.String r3 = r6.toString()
            r6 = r3
            r3 = 0
        L_0x1088:
            int r7 = r10.indirections
            if (r3 >= r7) goto L_0x10a0
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r6)
            java.lang.String r8 = "*"
            r7.append(r8)
            java.lang.String r6 = r7.toString()
            int r3 = r3 + 1
            goto L_0x1088
        L_0x10a0:
            boolean r3 = r10.reference
            if (r3 == 0) goto L_0x10b6
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r6)
            java.lang.String r7 = "&"
            r3.append(r7)
            java.lang.String r3 = r3.toString()
            goto L_0x10b7
        L_0x10b6:
            r3 = r6
        L_0x10b7:
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r3)
            java.lang.String r7 = " (*)("
            r6.append(r7)
            java.lang.String r3 = r6.toString()
            java.lang.String r6 = ""
            org.bytedeco.javacpp.tools.Parameters r7 = r10.parameters
            if (r7 == 0) goto L_0x1151
            org.bytedeco.javacpp.tools.Parameters r7 = r10.parameters
            org.bytedeco.javacpp.tools.Declarator[] r7 = r7.declarators
            int r8 = r7.length
            r11 = r6
            r6 = r3
            r3 = 0
        L_0x10d6:
            if (r3 >= r8) goto L_0x1148
            r72 = r8
            r8 = r7[r3]
            if (r8 == 0) goto L_0x1137
            r73 = r7
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r6)
            r7.append(r11)
            r74 = r6
            org.bytedeco.javacpp.tools.Type r6 = r8.type
            java.lang.String r6 = r6.cppName
            r7.append(r6)
            java.lang.String r6 = r7.toString()
            r7 = r6
            r6 = 0
        L_0x10fa:
            r75 = r9
            int r9 = r8.indirections
            if (r6 >= r9) goto L_0x111a
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r7)
            r76 = r11
            java.lang.String r11 = "*"
            r9.append(r11)
            java.lang.String r7 = r9.toString()
            int r6 = r6 + 1
            r9 = r75
            r11 = r76
            goto L_0x10fa
        L_0x111a:
            r76 = r11
            boolean r6 = r8.reference
            if (r6 == 0) goto L_0x1132
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r7)
            java.lang.String r9 = "&"
            r6.append(r9)
            java.lang.String r6 = r6.toString()
            goto L_0x1133
        L_0x1132:
            r6 = r7
        L_0x1133:
            java.lang.String r7 = ", "
            r11 = r7
            goto L_0x113f
        L_0x1137:
            r74 = r6
            r73 = r7
            r75 = r9
            r76 = r11
        L_0x113f:
            int r3 = r3 + 1
            r8 = r72
            r7 = r73
            r9 = r75
            goto L_0x10d6
        L_0x1148:
            r74 = r6
            r75 = r9
            r76 = r11
            r3 = r74
            goto L_0x1155
        L_0x1151:
            r75 = r9
            r76 = r6
        L_0x1155:
            org.bytedeco.javacpp.tools.InfoMap r6 = r1.infoMap
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r3)
            java.lang.String r8 = ")"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r3 = r7
            org.bytedeco.javacpp.tools.Info r6 = r6.getFirst(r7)
            if (r6 != 0) goto L_0x1177
            org.bytedeco.javacpp.tools.InfoMap r7 = r1.infoMap
            java.lang.String r8 = r10.cppName
            org.bytedeco.javacpp.tools.Info r6 = r7.getFirst(r8)
        L_0x1177:
            r7 = 0
            if (r0 == 0) goto L_0x1197
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r9 = 0
            char r11 = r0.charAt(r9)
            char r9 = java.lang.Character.toUpperCase(r11)
            r8.append(r9)
            r9 = 1
            java.lang.String r11 = r0.substring(r9)
            r8.append(r11)
            java.lang.String r7 = r8.toString()
        L_0x1197:
            if (r6 == 0) goto L_0x11a8
            java.lang.String[] r8 = r6.pointerTypes
            if (r8 == 0) goto L_0x11a8
            java.lang.String[] r8 = r6.pointerTypes
            int r8 = r8.length
            if (r8 <= 0) goto L_0x11a8
            java.lang.String[] r8 = r6.pointerTypes
            r9 = 0
            r7 = r8[r9]
            goto L_0x11ab
        L_0x11a8:
            if (r42 == 0) goto L_0x11ae
            r7 = r0
        L_0x11ab:
            r78 = r0
            goto L_0x121a
        L_0x11ae:
            org.bytedeco.javacpp.tools.Parameters r8 = r10.parameters
            if (r8 == 0) goto L_0x11d0
            org.bytedeco.javacpp.tools.Parameters r8 = r10.parameters
            java.lang.String r8 = r8.signature
            int r8 = r8.length()
            if (r8 <= 0) goto L_0x11d0
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r8.append(r7)
            org.bytedeco.javacpp.tools.Parameters r9 = r10.parameters
            java.lang.String r9 = r9.signature
            r8.append(r9)
            java.lang.String r7 = r8.toString()
            goto L_0x11ab
        L_0x11d0:
            java.lang.String r8 = r12.javaName
            java.lang.String r9 = "void"
            boolean r8 = r8.equals(r9)
            if (r8 != 0) goto L_0x1218
            java.lang.String r8 = r12.javaName
            java.lang.String r8 = r8.trim()
            r9 = 32
            int r9 = r8.lastIndexOf(r9)
            if (r9 <= 0) goto L_0x11ee
            int r11 = r9 + 1
            java.lang.String r8 = r8.substring(r11)
        L_0x11ee:
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r77 = r9
            r9 = 0
            char r16 = r8.charAt(r9)
            char r9 = java.lang.Character.toUpperCase(r16)
            r11.append(r9)
            r78 = r0
            r9 = 1
            java.lang.String r0 = r8.substring(r9)
            r11.append(r0)
            java.lang.String r0 = "_"
            r11.append(r0)
            r11.append(r7)
            java.lang.String r7 = r11.toString()
            goto L_0x121a
        L_0x1218:
            r78 = r0
        L_0x121a:
            if (r6 == 0) goto L_0x124b
            java.lang.String[] r0 = r6.annotations
            if (r0 == 0) goto L_0x124b
            java.lang.String[] r0 = r6.annotations
            int r8 = r0.length
            r9 = 0
        L_0x1224:
            if (r9 >= r8) goto L_0x124b
            r11 = r0[r9]
            r79 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r80 = r8
            java.lang.String r8 = r13.text
            r0.append(r8)
            r0.append(r11)
            java.lang.String r8 = " "
            r0.append(r8)
            java.lang.String r0 = r0.toString()
            r13.text = r0
            int r9 = r9 + 1
            r0 = r79
            r8 = r80
            goto L_0x1224
        L_0x124b:
            r0 = 32
            int r0 = r7.lastIndexOf(r0)
            r8 = 1
            int r0 = r0 + r8
            java.lang.String r0 = r7.substring(r0)
            java.lang.String r7 = "Pointer"
            boolean r7 = r0.equals(r7)
            if (r7 != 0) goto L_0x13ae
            org.bytedeco.javacpp.tools.Type r7 = new org.bytedeco.javacpp.tools.Type
            r7.<init>(r0)
            r13.type = r7
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = r13.text
            r7.append(r8)
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            org.bytedeco.javacpp.tools.Token r8 = r8.get()
            r9 = 3
            java.lang.Object[] r9 = new java.lang.Object[r9]
            org.bytedeco.javacpp.tools.Token r11 = org.bytedeco.javacpp.tools.Token.CONST
            r16 = 0
            r9[r16] = r11
            org.bytedeco.javacpp.tools.Token r11 = org.bytedeco.javacpp.tools.Token.__CONST
            r16 = 1
            r9[r16] = r11
            org.bytedeco.javacpp.tools.Token r11 = org.bytedeco.javacpp.tools.Token.CONSTEXPR
            r16 = 2
            r9[r16] = r11
            boolean r8 = r8.match(r9)
            if (r8 == 0) goto L_0x1294
            java.lang.String r8 = "@Const "
            goto L_0x1296
        L_0x1294:
            java.lang.String r8 = ""
        L_0x1296:
            r7.append(r8)
            java.lang.String r8 = "public static class "
            r7.append(r8)
            r7.append(r0)
            java.lang.String r8 = " extends FunctionPointer {\n    static { Loader.load(); }\n    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */\n    public    "
            r7.append(r8)
            r7.append(r0)
            java.lang.String r8 = "(Pointer p) { super(p); }\n"
            r7.append(r8)
            if (r4 == 0) goto L_0x12b3
            java.lang.String r8 = ""
            goto L_0x12c9
        L_0x12b3:
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r9 = "    protected "
            r8.append(r9)
            r8.append(r0)
            java.lang.String r9 = "() { allocate(); }\n    private native void allocate();\n"
            r8.append(r9)
            java.lang.String r8 = r8.toString()
        L_0x12c9:
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r13.text = r7
            if (r23 == 0) goto L_0x132c
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = r13.text
            r7.append(r8)
            java.lang.String r8 = "    public native "
            r7.append(r8)
            java.lang.String r8 = r12.annotations
            r7.append(r8)
            java.lang.String r8 = r12.javaName
            r7.append(r8)
            java.lang.String r8 = " get("
            r7.append(r8)
            java.lang.String[] r8 = r4.pointerTypes
            r9 = 0
            r8 = r8[r9]
            r7.append(r8)
            java.lang.String r8 = " o);\n    public native "
            r7.append(r8)
            r7.append(r0)
            java.lang.String r8 = " put("
            r7.append(r8)
            java.lang.String[] r8 = r4.pointerTypes
            r8 = r8[r9]
            r7.append(r8)
            java.lang.String r8 = " o, "
            r7.append(r8)
            java.lang.String r8 = r12.annotations
            r7.append(r8)
            java.lang.String r8 = r12.javaName
            r7.append(r8)
            java.lang.String r8 = " v);\n}\n"
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r13.text = r7
            r81 = r4
            goto L_0x13b0
        L_0x132c:
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = r13.text
            r7.append(r8)
            java.lang.String r8 = "    public native "
            r7.append(r8)
            java.lang.String r8 = r12.annotations
            r7.append(r8)
            java.lang.String r8 = r12.javaName
            r7.append(r8)
            java.lang.String r8 = " call"
            r7.append(r8)
            if (r4 == 0) goto L_0x1399
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r9 = "("
            r8.append(r9)
            java.lang.String[] r9 = r4.pointerTypes
            r11 = 0
            r9 = r9[r11]
            r8.append(r9)
            java.lang.String r9 = " o"
            r8.append(r9)
            org.bytedeco.javacpp.tools.Parameters r9 = r10.parameters
            java.lang.String r9 = r9.list
            r11 = 1
            char r9 = r9.charAt(r11)
            r11 = 41
            if (r9 != r11) goto L_0x1375
            java.lang.String r9 = ")"
            r81 = r4
            goto L_0x1391
        L_0x1375:
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r11 = ", "
            r9.append(r11)
            org.bytedeco.javacpp.tools.Parameters r11 = r10.parameters
            java.lang.String r11 = r11.list
            r81 = r4
            r4 = 1
            java.lang.String r11 = r11.substring(r4)
            r9.append(r11)
            java.lang.String r9 = r9.toString()
        L_0x1391:
            r8.append(r9)
            java.lang.String r4 = r8.toString()
            goto L_0x139f
        L_0x1399:
            r81 = r4
            org.bytedeco.javacpp.tools.Parameters r4 = r10.parameters
            java.lang.String r4 = r4.list
        L_0x139f:
            r7.append(r4)
            java.lang.String r4 = ";\n}\n"
            r7.append(r4)
            java.lang.String r4 = r7.toString()
            r13.text = r4
            goto L_0x13b0
        L_0x13ae:
            r81 = r4
        L_0x13b0:
            r13.signature = r0
            org.bytedeco.javacpp.tools.Declarator r4 = new org.bytedeco.javacpp.tools.Declarator
            r4.<init>()
            r13.declarator = r4
            org.bytedeco.javacpp.tools.Declarator r4 = r13.declarator
            org.bytedeco.javacpp.tools.Parameters r7 = r10.parameters
            r4.parameters = r7
            if (r6 == 0) goto L_0x13c5
            boolean r4 = r6.skip
            if (r4 != 0) goto L_0x13c7
        L_0x13c5:
            r10.definition = r13
        L_0x13c7:
            r10.indirections = r5
            if (r89 == 0) goto L_0x13de
            int r7 = r10.indirections
            r8 = 1
            if (r7 <= r8) goto L_0x13de
            int r7 = r10.indices
            int r9 = r7 + 1
            r10.indices = r9
            r9 = -1
            r49[r7] = r9
            int r7 = r10.indirections
            int r7 = r7 - r8
            r10.indirections = r7
        L_0x13de:
            if (r23 != 0) goto L_0x13e3
            r7 = 0
            r10.parameters = r7
        L_0x13e3:
            int r7 = r10.indirections
            r8 = 1
            if (r7 <= r8) goto L_0x1420
            r7 = 40
            int r7 = r3.indexOf(r7)
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r9 = "@Cast(\""
            r8.append(r9)
            int r9 = r7 + 1
            r11 = 0
            java.lang.String r9 = r3.substring(r11, r9)
            r8.append(r9)
            java.lang.String r9 = "*"
            r8.append(r9)
            int r9 = r7 + 1
            java.lang.String r9 = r3.substring(r9)
            r8.append(r9)
            java.lang.String r9 = "\") "
            r8.append(r9)
            java.lang.String r8 = r8.toString()
            r12.annotations = r8
            java.lang.String r8 = "PointerPointer"
            r12.javaName = r8
            goto L_0x144a
        L_0x1420:
            if (r6 == 0) goto L_0x143d
            boolean r7 = r6.cast
            if (r7 == 0) goto L_0x143d
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "@Cast(\""
            r7.append(r8)
            r7.append(r3)
            java.lang.String r8 = "\") "
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            goto L_0x1446
        L_0x143d:
            boolean r7 = r10.constPointer
            if (r7 == 0) goto L_0x1444
            java.lang.String r7 = "@Const "
            goto L_0x1446
        L_0x1444:
            java.lang.String r7 = ""
        L_0x1446:
            r12.annotations = r7
            r12.javaName = r0
        L_0x144a:
            r69 = r6
        L_0x144c:
            java.lang.String r0 = r10.cppName
            if (r0 == 0) goto L_0x14be
            java.lang.String r0 = r10.cppName
            java.lang.String r3 = r2.namespace
            if (r3 == 0) goto L_0x147d
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r6 = r2.namespace
            r3.append(r6)
            java.lang.String r6 = "::"
            r3.append(r6)
            java.lang.String r3 = r3.toString()
            boolean r3 = r0.startsWith(r3)
            if (r3 == 0) goto L_0x147d
            java.lang.String r3 = r10.cppName
            java.lang.String r6 = r2.namespace
            int r6 = r6.length()
            r7 = 2
            int r6 = r6 + r7
            java.lang.String r0 = r3.substring(r6)
        L_0x147d:
            r3 = 60
            int r3 = r0.lastIndexOf(r3)
            if (r3 < 0) goto L_0x148b
            r6 = 0
            java.lang.String r7 = r0.substring(r6, r3)
            goto L_0x148c
        L_0x148b:
            r7 = r0
        L_0x148c:
            r6 = r7
            java.lang.String r7 = r10.javaName
            boolean r7 = r6.equals(r7)
            if (r7 != 0) goto L_0x14be
            java.lang.String r7 = "::"
            boolean r7 = r0.contains(r7)
            if (r7 == 0) goto L_0x14a1
            java.lang.String r7 = r2.javaName
            if (r7 != 0) goto L_0x14be
        L_0x14a1:
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = r12.annotations
            r7.append(r8)
            java.lang.String r8 = "@Name(\""
            r7.append(r8)
            r7.append(r0)
            java.lang.String r8 = "\") "
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r12.annotations = r7
        L_0x14be:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r3 = 1
            java.lang.Object[] r6 = new java.lang.Object[r3]
            r7 = 41
            java.lang.Character r8 = java.lang.Character.valueOf(r7)
            r9 = 0
            r6[r9] = r8
            boolean r0 = r0.match(r6)
            if (r0 == 0) goto L_0x14e0
            if (r37 <= 0) goto L_0x14e0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
            int r37 = r37 + -1
            goto L_0x14be
        L_0x14e0:
            return r10
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.declarator(org.bytedeco.javacpp.tools.Context, java.lang.String, int, boolean, int, boolean, boolean):org.bytedeco.javacpp.tools.Declarator");
    }

    /* access modifiers changed from: package-private */
    public String commentDoc(String s, int startIndex) {
        int n;
        if (startIndex < 0 || startIndex > s.length()) {
            return s;
        }
        int index = s.indexOf("/**", startIndex);
        StringBuilder sb = new StringBuilder(s);
        while (index < sb.length()) {
            char c = sb.charAt(index);
            String ss = sb.substring(index + 1);
            if (c == '`' && ss.startsWith("``") && sb.length() - index > 3) {
                int i = index + 3;
                StringBuilder sb2 = new StringBuilder();
                sb2.append("<pre>{@code");
                sb2.append(Character.isWhitespace(sb.charAt(index + 3)) ? "" : " ");
                sb.replace(index, i, sb2.toString());
                index = sb.indexOf("```", index);
                if (index < 0) {
                    break;
                }
                sb.replace(index, index + 3, "}</pre>");
            } else if (c == '`') {
                sb.replace(index, index + 1, "{@code ");
                index = sb.indexOf("`", index);
                if (index < 0) {
                    break;
                }
                sb.replace(index, index + 1, "}");
            } else if ((c == '\\' || c == '@') && ss.startsWith("code")) {
                int i2 = index + 5;
                StringBuilder sb3 = new StringBuilder();
                sb3.append("<pre>{@code");
                sb3.append(Character.isWhitespace(sb.charAt(index + 5)) ? "" : " ");
                sb.replace(index, i2, sb3.toString());
                index = sb.indexOf(c + "endcode", index);
                if (index < 0) {
                    break;
                }
                sb.replace(index, index + 8, "}</pre>");
            } else if ((c == '\\' || c == '@') && ss.startsWith("verbatim")) {
                int i3 = index + 9;
                StringBuilder sb4 = new StringBuilder();
                sb4.append("<pre>{@literal");
                sb4.append(Character.isWhitespace(sb.charAt(index + 9)) ? "" : " ");
                sb.replace(index, i3, sb4.toString());
                index = sb.indexOf(c + "endverbatim", index);
                if (index < 0) {
                    break;
                }
                sb.replace(index, index + 12, "}</pre>");
            } else {
                int n2 = 0;
                if (c != 10 || ss.length() <= 0 || ss.charAt(0) != 10) {
                    if (c != '\\' && c != '@') {
                        if (c == '*' && ss.charAt(0) == '/' && (index = sb.indexOf("/**", index)) < 0) {
                            break;
                        }
                    } else {
                        String foundTag = null;
                        String[] strArr = this.docTags;
                        int length = strArr.length;
                        while (true) {
                            if (n2 >= length) {
                                break;
                            }
                            String tag = strArr[n2];
                            if (ss.startsWith(tag)) {
                                foundTag = tag;
                                break;
                            }
                            n2++;
                        }
                        if (foundTag != null) {
                            sb.setCharAt(index, '@');
                            int n3 = foundTag.length() + index + 1;
                            if (sb.charAt(n3) == 's' && !foundTag.endsWith("s")) {
                                sb.deleteCharAt(n3);
                            } else if (!Character.isWhitespace(sb.charAt(n3))) {
                                sb.insert(n3, ' ');
                            }
                        } else {
                            sb.setCharAt(index, IOUtils.DIR_SEPARATOR_WINDOWS);
                        }
                    }
                } else {
                    while (true) {
                        n = n2;
                        if (n >= ss.length() || ss.charAt(n) != 10) {
                            String indent = "";
                        } else {
                            n2 = n + 1;
                        }
                    }
                    String indent2 = "";
                    while (n < ss.length() && Character.isWhitespace(ss.charAt(n))) {
                        indent2 = indent2 + ss.charAt(n);
                        n++;
                    }
                    sb.insert(index + 1, indent2 + "<p>");
                }
            }
            index++;
        }
        return sb.toString();
    }

    /* access modifiers changed from: package-private */
    public String commentBefore() throws ParserException {
        String comment = "";
        this.tokens.raw = true;
        while (this.tokens.index > 0) {
            if (!this.tokens.get(-1).match(4)) {
                break;
            }
            this.tokens.index--;
        }
        boolean closeComment = false;
        int startDoc = -1;
        Token token = this.tokens.get();
        while (true) {
            if (!token.match(4)) {
                break;
            }
            String s = token.value;
            if (s.startsWith("/**") || s.startsWith("/*!") || s.startsWith("///") || s.startsWith("//!")) {
                if (s.startsWith("//") && s.contains("*/") && s.indexOf("*/") < s.length() - 2) {
                    s = s.replace("*/", "* /");
                }
                if (s.length() > 3 && s.charAt(3) == '<') {
                    token = this.tokens.next();
                } else if (s.length() >= 3 && ((s.startsWith("///") || s.startsWith("//!")) && !s.startsWith("////") && !s.startsWith("///*"))) {
                    String lastComment = comment.trim();
                    int n2 = lastComment.indexOf(10);
                    while (!lastComment.startsWith("/*") && n2 > 0) {
                        lastComment = n2 + 1 < lastComment.length() ? lastComment.substring(n2 + 1).trim() : "";
                        n2 = lastComment.indexOf(10);
                    }
                    StringBuilder sb = new StringBuilder();
                    sb.append((comment.length() == 0 || comment.contains("*/") || !lastComment.startsWith("/*")) ? "/**" : " * ");
                    sb.append(s.substring(3));
                    s = sb.toString();
                    closeComment = true;
                } else if (s.length() > 3 && !s.startsWith("///")) {
                    s = "/**" + s.substring(3);
                }
            } else if (closeComment && !comment.endsWith("*/")) {
                closeComment = false;
                comment = comment + " */";
            }
            if (startDoc < 0 && s.startsWith("/**")) {
                startDoc = comment.length();
            }
            comment = comment + token.spacing + s;
            token = this.tokens.next();
        }
        if (closeComment && !comment.endsWith("*/")) {
            comment = comment + " */";
        }
        this.tokens.raw = false;
        return commentDoc(comment, startDoc);
    }

    /* access modifiers changed from: package-private */
    public String commentAfter() throws ParserException {
        String comment = "";
        this.tokens.raw = true;
        while (this.tokens.index > 0) {
            if (!this.tokens.get(-1).match(4)) {
                break;
            }
            this.tokens.index--;
        }
        boolean closeComment = false;
        int startDoc = -1;
        Token token = this.tokens.get();
        while (true) {
            if (!token.match(4)) {
                break;
            }
            String s = token.value;
            String spacing = token.spacing;
            int n = spacing.lastIndexOf(10) + 1;
            if ((s.startsWith("/**") || s.startsWith("/*!") || s.startsWith("///") || s.startsWith("//!")) && (s.length() <= 3 || s.charAt(3) == '<')) {
                if (s.length() > 4 && (s.startsWith("///") || s.startsWith("//!"))) {
                    String lastComment = comment.trim();
                    int n2 = lastComment.indexOf(10);
                    while (!lastComment.startsWith("/*") && n2 > 0) {
                        lastComment = n2 + 1 < lastComment.length() ? lastComment.substring(n2 + 1).trim() : "";
                        n2 = lastComment.indexOf(10);
                    }
                    StringBuilder sb = new StringBuilder();
                    sb.append((comment.length() == 0 || comment.contains("*/") || !lastComment.startsWith("/*")) ? "/**" : " * ");
                    sb.append(s.substring(4));
                    s = sb.toString();
                    closeComment = true;
                } else if (s.length() > 4) {
                    s = "/**" + s.substring(4);
                }
                if (startDoc < 0 && s.startsWith("/**")) {
                    startDoc = comment.length();
                }
                comment = comment + spacing.substring(0, n) + s;
            }
            token = this.tokens.next();
        }
        if (closeComment && !comment.endsWith("*/")) {
            comment = comment + " */";
        }
        if (comment.length() > 0) {
            comment = comment + "\n";
        }
        this.tokens.raw = false;
        return commentDoc(comment, startDoc);
    }

    /* access modifiers changed from: package-private */
    public Attribute attribute() throws ParserException {
        return attribute(false);
    }

    /* access modifiers changed from: package-private */
    public Attribute attribute(boolean explicit) throws ParserException {
        if (this.tokens.get().match(5)) {
            if (!this.tokens.get(1).match('<')) {
                Attribute attr = new Attribute();
                InfoMap infoMap2 = this.infoMap;
                String str = this.tokens.get().value;
                attr.cppName = str;
                Info info = infoMap2.getFirst(str);
                boolean z = info != null && info.annotations != null && info.javaNames == null && info.valueTypes == null && info.pointerTypes == null;
                attr.annotation = z;
                if (z) {
                    for (String s : info.annotations) {
                        attr.javaName += s + " ";
                    }
                }
                if (explicit && !attr.annotation) {
                    return null;
                }
                if (!this.tokens.next().match('(')) {
                    return attr;
                }
                int count = 1;
                this.tokens.raw = true;
                Token token = this.tokens.next();
                while (true) {
                    if (token.match(Token.EOF) || count <= 0) {
                        this.tokens.raw = false;
                    } else {
                        if (token.match('(')) {
                            count++;
                        } else {
                            if (token.match(')')) {
                                count--;
                            } else if (info == null || !info.skip) {
                                attr.arguments += token.value;
                            }
                        }
                        token = this.tokens.next();
                    }
                }
                this.tokens.raw = false;
                return attr;
            }
        }
        return null;
    }

    /* access modifiers changed from: package-private */
    public String body() throws ParserException {
        String text = "";
        if (!this.tokens.get().match('{')) {
            return null;
        }
        int count = 1;
        this.tokens.raw = true;
        Token token = this.tokens.next();
        while (true) {
            if (token.match(Token.EOF) || count <= 0) {
                this.tokens.raw = false;
            } else {
                if (token.match('{')) {
                    count++;
                } else {
                    if (token.match('}')) {
                        count--;
                    }
                }
                if (count > 0) {
                    text = text + token.spacing + token;
                }
                token = this.tokens.next();
            }
        }
        this.tokens.raw = false;
        return text;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r5v26, resolved type: org.bytedeco.javacpp.tools.Token} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r5v28, resolved type: org.bytedeco.javacpp.tools.Token} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r5v30, resolved type: java.lang.String} */
    /* JADX WARNING: type inference failed for: r14v0 */
    /* JADX WARNING: type inference failed for: r14v1, types: [boolean] */
    /* JADX WARNING: type inference failed for: r14v6 */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Multi-variable type inference failed */
    /* JADX WARNING: Removed duplicated region for block: B:117:0x043a A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:122:0x0473  */
    /* JADX WARNING: Removed duplicated region for block: B:145:0x0478 A[SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.bytedeco.javacpp.tools.Parameters parameters(org.bytedeco.javacpp.tools.Context r30, int r31, boolean r32) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r29 = this;
            r9 = r29
            r10 = r30
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r9.tokens
            int r11 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r9.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r12 = 1
            java.lang.Object[] r1 = new java.lang.Object[r12]
            r13 = 40
            java.lang.Character r2 = java.lang.Character.valueOf(r13)
            r14 = 0
            r1[r14] = r2
            boolean r0 = r0.match(r1)
            r15 = 0
            if (r0 != 0) goto L_0x0022
            return r15
        L_0x0022:
            r0 = 0
            org.bytedeco.javacpp.tools.Parameters r1 = new org.bytedeco.javacpp.tools.Parameters
            r1.<init>()
            r8 = r1
            java.util.ArrayList r1 = new java.util.ArrayList
            r1.<init>()
            r7 = r1
            java.lang.String r1 = "("
            r8.list = r1
            java.lang.String r1 = "("
            r8.names = r1
            r1 = -1
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r9.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.next()
            r6 = r1
        L_0x003f:
            r5 = r2
            java.lang.Object[] r1 = new java.lang.Object[r12]
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.EOF
            r1[r14] = r2
            boolean r1 = r5.match(r1)
            if (r1 != 0) goto L_0x0487
            java.lang.String r4 = r5.spacing
            java.lang.Object[] r1 = new java.lang.Object[r12]
            r16 = 41
            java.lang.Character r2 = java.lang.Character.valueOf(r16)
            r1[r14] = r2
            boolean r1 = r5.match(r1)
            if (r1 == 0) goto L_0x0097
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = r8.list
            r1.append(r2)
            r1.append(r4)
            java.lang.String r2 = ")"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r8.list = r1
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = r8.names
            r1.append(r2)
            java.lang.String r2 = ")"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r8.names = r1
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r9.tokens
            r1.next()
            r12 = r31
            r3 = r6
            r5 = r8
            goto L_0x048b
        L_0x0097:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "arg"
            r1.append(r2)
            int r3 = r0 + 1
            r1.append(r0)
            java.lang.String r0 = r1.toString()
            r17 = 0
            r18 = 1
            r19 = 0
            r1 = r29
            r2 = r30
            r20 = r3
            r3 = r0
            r21 = r4
            r4 = r31
            r0 = r5
            r5 = r32
            r22 = r6
            r6 = r17
            r23 = r7
            r7 = r18
            r24 = r8
            r8 = r19
            org.bytedeco.javacpp.tools.Declarator r1 = r1.declarator(r2, r3, r4, r5, r6, r7, r8)
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r9.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.get()
            r3 = 2
            java.lang.Object[] r4 = new java.lang.Object[r3]
            r5 = 44
            java.lang.Character r6 = java.lang.Character.valueOf(r5)
            r4[r14] = r6
            java.lang.Character r6 = java.lang.Character.valueOf(r16)
            r4[r12] = r6
            boolean r2 = r2.match(r4)
            r2 = r2 ^ r12
            r4 = 0
            java.lang.String r6 = ""
            r7 = 3
            if (r1 == 0) goto L_0x02c6
            if (r2 == 0) goto L_0x02c6
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r9.tokens
            org.bytedeco.javacpp.tools.Token r4 = r8.get()
            r8 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r13 = r9.tokens
            org.bytedeco.javacpp.tools.Token r0 = r13.next()
            java.lang.String r13 = ""
            r0.spacing = r13
        L_0x0103:
            java.lang.Object[] r13 = new java.lang.Object[r12]
            org.bytedeco.javacpp.tools.Token r17 = org.bytedeco.javacpp.tools.Token.EOF
            r13[r14] = r17
            boolean r13 = r0.match(r13)
            if (r13 != 0) goto L_0x0202
            r13 = 125(0x7d, float:1.75E-43)
            if (r8 != 0) goto L_0x0130
            java.lang.Object[] r3 = new java.lang.Object[r7]
            java.lang.Character r17 = java.lang.Character.valueOf(r5)
            r3[r14] = r17
            java.lang.Character r17 = java.lang.Character.valueOf(r16)
            r3[r12] = r17
            java.lang.Character r17 = java.lang.Character.valueOf(r13)
            r5 = 2
            r3[r5] = r17
            boolean r3 = r0.match(r3)
            if (r3 == 0) goto L_0x0131
            goto L_0x0202
        L_0x0130:
            r5 = 2
        L_0x0131:
            java.lang.Object[] r3 = new java.lang.Object[r5]
            r5 = 40
            java.lang.Character r17 = java.lang.Character.valueOf(r5)
            r3[r14] = r17
            r17 = 123(0x7b, float:1.72E-43)
            java.lang.Character r17 = java.lang.Character.valueOf(r17)
            r3[r12] = r17
            boolean r3 = r0.match(r3)
            if (r3 == 0) goto L_0x014c
            int r8 = r8 + 1
            goto L_0x0163
        L_0x014c:
            r3 = 2
            java.lang.Object[] r5 = new java.lang.Object[r3]
            java.lang.Character r3 = java.lang.Character.valueOf(r16)
            r5[r14] = r3
            java.lang.Character r3 = java.lang.Character.valueOf(r13)
            r5[r12] = r3
            boolean r3 = r0.match(r5)
            if (r3 == 0) goto L_0x0163
            int r8 = r8 + -1
        L_0x0163:
            java.lang.String r3 = r0.value
            java.lang.String[] r5 = r10.qualify(r3)
            int r13 = r5.length
            r17 = r3
            r3 = 0
        L_0x016d:
            if (r3 >= r13) goto L_0x018c
            r7 = r5[r3]
            org.bytedeco.javacpp.tools.InfoMap r12 = r9.infoMap
            org.bytedeco.javacpp.tools.Info r12 = r12.getFirst(r7, r14)
            if (r12 == 0) goto L_0x017c
            r17 = r7
            goto L_0x018c
        L_0x017c:
            org.bytedeco.javacpp.tools.InfoMap r12 = r9.infoMap
            org.bytedeco.javacpp.tools.Info r12 = r12.getFirst(r7)
            if (r12 == 0) goto L_0x0187
            r17 = r7
        L_0x0187:
            int r3 = r3 + 1
            r7 = 3
            r12 = 1
            goto L_0x016d
        L_0x018c:
            r3 = 1
            java.lang.Object[] r5 = new java.lang.Object[r3]
            r7 = 5
            java.lang.Integer r7 = java.lang.Integer.valueOf(r7)
            r5[r14] = r7
            boolean r5 = r0.match(r5)
            if (r5 == 0) goto L_0x01d5
            r5 = r17
        L_0x019e:
            org.bytedeco.javacpp.tools.TokenIndexer r7 = r9.tokens
            org.bytedeco.javacpp.tools.Token r7 = r7.get(r3)
            java.lang.String r3 = "::"
            boolean r3 = r7.equals(r3)
            if (r3 == 0) goto L_0x01d3
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r9.tokens
            r3.next()
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r9.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.next()
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r5)
            java.lang.String r12 = "::"
            r7.append(r12)
            java.lang.String r12 = r3.spacing
            r7.append(r12)
            r7.append(r3)
            java.lang.String r5 = r7.toString()
            r3 = 1
            goto L_0x019e
        L_0x01d3:
            r17 = r5
        L_0x01d5:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r6)
            java.lang.String r5 = r0.spacing
            r3.append(r5)
            if (r17 == 0) goto L_0x01ed
            int r5 = r17.length()
            if (r5 <= 0) goto L_0x01ed
            r5 = r17
            goto L_0x01ee
        L_0x01ed:
            r5 = r0
        L_0x01ee:
            r3.append(r5)
            java.lang.String r6 = r3.toString()
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r9.tokens
            org.bytedeco.javacpp.tools.Token r0 = r3.next()
            r3 = 2
            r5 = 44
            r7 = 3
            r12 = 1
            goto L_0x0103
        L_0x0202:
            java.lang.String[] r3 = r10.qualify(r6)
            int r5 = r3.length
            r7 = r6
            r6 = 0
        L_0x0209:
            if (r6 >= r5) goto L_0x0223
            r12 = r3[r6]
            org.bytedeco.javacpp.tools.InfoMap r13 = r9.infoMap
            org.bytedeco.javacpp.tools.Info r13 = r13.getFirst(r12, r14)
            if (r13 == 0) goto L_0x0217
            r7 = r12
            goto L_0x0223
        L_0x0217:
            org.bytedeco.javacpp.tools.InfoMap r13 = r9.infoMap
            org.bytedeco.javacpp.tools.Info r13 = r13.getFirst(r12)
            if (r13 == 0) goto L_0x0220
            r7 = r12
        L_0x0220:
            int r6 = r6 + 1
            goto L_0x0209
        L_0x0223:
            org.bytedeco.javacpp.tools.Type r3 = r1.type
            java.lang.String r3 = r3.annotations
            java.lang.String r5 = "@ByVal "
            int r5 = r3.indexOf(r5)
            if (r5 >= 0) goto L_0x0235
            java.lang.String r6 = "@ByRef "
            int r5 = r3.indexOf(r6)
        L_0x0235:
            if (r5 < 0) goto L_0x02bc
            org.bytedeco.javacpp.tools.Type r6 = r1.type
            java.lang.String r6 = r6.cppName
            boolean r6 = r7.startsWith(r6)
            if (r6 != 0) goto L_0x025e
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            org.bytedeco.javacpp.tools.Type r12 = r1.type
            java.lang.String r12 = r12.cppName
            r6.append(r12)
            java.lang.String r12 = "("
            r6.append(r12)
            r6.append(r7)
            java.lang.String r12 = ")"
            r6.append(r12)
            java.lang.String r7 = r6.toString()
        L_0x025e:
            org.bytedeco.javacpp.tools.InfoMap r6 = r9.infoMap
            org.bytedeco.javacpp.tools.Info r6 = r6.getFirst(r7)
            if (r6 == 0) goto L_0x027d
            boolean r12 = r6.skip
            if (r12 == 0) goto L_0x027d
            if (r32 == 0) goto L_0x0277
            org.bytedeco.javacpp.tools.TokenIndexer r12 = r9.tokens
            r12.index = r11
            r12 = r31
            org.bytedeco.javacpp.tools.Parameters r13 = r9.parameters(r10, r12, r14)
            return r13
        L_0x0277:
            r12 = r31
            r25 = r0
            r6 = r7
            goto L_0x02c1
        L_0x027d:
            r12 = r31
            java.lang.String r13 = "\""
            java.lang.String r14 = "\\\\\""
            java.lang.String r13 = r7.replaceAll(r13, r14)
            java.lang.String r14 = "\n(\\s*)"
            r25 = r0
            java.lang.String r0 = "\"\n$1 + \""
            java.lang.String r0 = r13.replaceAll(r14, r0)
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            int r13 = r5 + 6
            r14 = 0
            java.lang.String r13 = r3.substring(r14, r13)
            r7.append(r13)
            java.lang.String r13 = "(nullValue = \""
            r7.append(r13)
            r7.append(r0)
            java.lang.String r13 = "\")"
            r7.append(r13)
            int r13 = r5 + 6
            java.lang.String r13 = r3.substring(r13)
            r7.append(r13)
            java.lang.String r3 = r7.toString()
            r6 = r0
            goto L_0x02c1
        L_0x02bc:
            r12 = r31
            r25 = r0
            r6 = r7
        L_0x02c1:
            org.bytedeco.javacpp.tools.Type r0 = r1.type
            r0.annotations = r3
            goto L_0x02ca
        L_0x02c6:
            r12 = r31
            r25 = r0
        L_0x02ca:
            if (r1 == 0) goto L_0x042d
            org.bytedeco.javacpp.tools.Type r0 = r1.type
            java.lang.String r0 = r0.javaName
            java.lang.String r3 = "void"
            boolean r0 = r0.equals(r3)
            if (r0 != 0) goto L_0x042d
            if (r2 == 0) goto L_0x02e7
            if (r32 != 0) goto L_0x02dd
            goto L_0x02e7
        L_0x02dd:
            r8 = r20
            r13 = r21
            r3 = r22
            r5 = r24
            goto L_0x0435
        L_0x02e7:
            r3 = r22
            if (r3 < 0) goto L_0x0313
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r5 = r24
            java.lang.String r7 = r5.list
            r8 = 0
            java.lang.String r7 = r7.substring(r8, r3)
            r0.append(r7)
            java.lang.String r7 = "[]"
            r0.append(r7)
            java.lang.String r7 = r5.list
            int r8 = r3 + 3
            java.lang.String r7 = r7.substring(r8)
            r0.append(r7)
            java.lang.String r0 = r0.toString()
            r5.list = r0
            goto L_0x0315
        L_0x0313:
            r5 = r24
        L_0x0315:
            java.lang.String r0 = r5.list
            int r7 = r0.length()
            int r0 = r5.infoNumber
            int r8 = r1.infoNumber
            int r0 = java.lang.Math.max(r0, r8)
            r5.infoNumber = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r8 = r5.list
            r0.append(r8)
            r8 = r20
            r13 = 1
            if (r8 <= r13) goto L_0x0337
            java.lang.String r13 = ","
            goto L_0x0339
        L_0x0337:
            java.lang.String r13 = ""
        L_0x0339:
            r0.append(r13)
            r13 = r21
            r0.append(r13)
            org.bytedeco.javacpp.tools.Type r14 = r1.type
            java.lang.String r14 = r14.annotations
            r0.append(r14)
            org.bytedeco.javacpp.tools.Type r14 = r1.type
            java.lang.String r14 = r14.javaName
            r0.append(r14)
            java.lang.String r14 = " "
            r0.append(r14)
            java.lang.String r14 = r1.javaName
            r0.append(r14)
            java.lang.String r0 = r0.toString()
            r5.list = r0
            java.lang.String r0 = r5.list
            java.lang.String r14 = "..."
            int r3 = r0.indexOf(r14, r7)
            if (r2 == 0) goto L_0x0395
            org.bytedeco.javacpp.tools.Type r0 = r1.type
            java.lang.String r0 = r0.annotations
            java.lang.String r14 = "(nullValue = "
            boolean r0 = r0.contains(r14)
            if (r0 != 0) goto L_0x0395
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r14 = r5.list
            r0.append(r14)
            java.lang.String r14 = "/*"
            r0.append(r14)
            r0.append(r4)
            r0.append(r6)
            java.lang.String r14 = "*/"
            r0.append(r14)
            java.lang.String r0 = r0.toString()
            r5.list = r0
        L_0x0395:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r14 = r5.signature
            r0.append(r14)
            r14 = 95
            r0.append(r14)
            java.lang.String r0 = r0.toString()
            r5.signature = r0
            org.bytedeco.javacpp.tools.Type r0 = r1.type
            java.lang.String r0 = r0.javaName
            org.bytedeco.javacpp.tools.Type r14 = r1.type
            java.lang.String r14 = r14.javaName
            r26 = r3
            r3 = 32
            int r3 = r14.lastIndexOf(r3)
            r14 = 1
            int r3 = r3 + r14
            java.lang.String r0 = r0.substring(r3)
            char[] r0 = r0.toCharArray()
            int r3 = r0.length
            r14 = 0
        L_0x03c6:
            if (r14 >= r3) goto L_0x03f3
            char r17 = r0[r14]
            r27 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r28 = r3
            java.lang.String r3 = r5.signature
            r0.append(r3)
            boolean r3 = java.lang.Character.isJavaIdentifierPart(r17)
            if (r3 == 0) goto L_0x03e1
            r3 = r17
            goto L_0x03e3
        L_0x03e1:
            r3 = 95
        L_0x03e3:
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r5.signature = r0
            int r14 = r14 + 1
            r0 = r27
            r3 = r28
            goto L_0x03c6
        L_0x03f3:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r5.names
            r0.append(r3)
            r3 = 1
            if (r8 <= r3) goto L_0x0403
            java.lang.String r3 = ", "
            goto L_0x0405
        L_0x0403:
            java.lang.String r3 = ""
        L_0x0405:
            r0.append(r3)
            java.lang.String r3 = r1.javaName
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r5.names = r0
            java.lang.String r0 = r1.javaName
            java.lang.String r3 = "arg"
            boolean r0 = r0.startsWith(r3)
            if (r0 == 0) goto L_0x0437
            java.lang.String r0 = r1.javaName     // Catch:{ NumberFormatException -> 0x042b }
            r3 = 3
            java.lang.String r0 = r0.substring(r3)     // Catch:{ NumberFormatException -> 0x042b }
            int r0 = java.lang.Integer.parseInt(r0)     // Catch:{ NumberFormatException -> 0x042b }
            r3 = 1
            int r0 = r0 + r3
            goto L_0x0438
        L_0x042b:
            r0 = move-exception
            goto L_0x0437
        L_0x042d:
            r8 = r20
            r13 = r21
            r3 = r22
            r5 = r24
        L_0x0435:
            r26 = r3
        L_0x0437:
            r0 = r8
        L_0x0438:
            if (r2 == 0) goto L_0x0440
            if (r32 != 0) goto L_0x043d
            goto L_0x0440
        L_0x043d:
            r7 = r23
            goto L_0x0445
        L_0x0440:
            r7 = r23
            r7.add(r1)
        L_0x0445:
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r9.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.get()
            r8 = 2
            java.lang.Object[] r8 = new java.lang.Object[r8]
            r14 = 44
            java.lang.Character r17 = java.lang.Character.valueOf(r14)
            r18 = 0
            r8[r18] = r17
            java.lang.Character r16 = java.lang.Character.valueOf(r16)
            r14 = 1
            r8[r14] = r16
            org.bytedeco.javacpp.tools.Token r3 = r3.expect(r8)
            java.lang.Object[] r8 = new java.lang.Object[r14]
            r14 = 44
            java.lang.Character r14 = java.lang.Character.valueOf(r14)
            r8[r18] = r14
            boolean r3 = r3.match(r8)
            if (r3 == 0) goto L_0x0478
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r9.tokens
            r3.next()
        L_0x0478:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r9.tokens
            org.bytedeco.javacpp.tools.Token r2 = r1.get()
            r8 = r5
            r6 = r26
            r12 = 1
            r13 = 40
            r14 = 0
            goto L_0x003f
        L_0x0487:
            r12 = r31
            r3 = r6
            r5 = r8
        L_0x048b:
            org.bytedeco.javacpp.tools.TemplateMap r1 = r10.templateMap
            if (r1 != 0) goto L_0x04c8
            int r1 = r7.size()
            r2 = 1
            if (r1 != r2) goto L_0x04c8
            r1 = 0
            java.lang.Object r2 = r7.get(r1)
            if (r2 == 0) goto L_0x04c3
            java.lang.Object r2 = r7.get(r1)
            org.bytedeco.javacpp.tools.Declarator r2 = (org.bytedeco.javacpp.tools.Declarator) r2
            org.bytedeco.javacpp.tools.Type r2 = r2.type
            if (r2 == 0) goto L_0x04c3
            java.lang.Object r2 = r7.get(r1)
            org.bytedeco.javacpp.tools.Declarator r2 = (org.bytedeco.javacpp.tools.Declarator) r2
            org.bytedeco.javacpp.tools.Type r2 = r2.type
            java.lang.String r2 = r2.cppName
            if (r2 == 0) goto L_0x04c3
            java.lang.Object r1 = r7.get(r1)
            org.bytedeco.javacpp.tools.Declarator r1 = (org.bytedeco.javacpp.tools.Declarator) r1
            org.bytedeco.javacpp.tools.Type r1 = r1.type
            java.lang.String r1 = r1.cppName
            int r1 = r1.length()
            if (r1 != 0) goto L_0x04c8
        L_0x04c3:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r9.tokens
            r1.index = r11
            return r15
        L_0x04c8:
            int r1 = r7.size()
            org.bytedeco.javacpp.tools.Declarator[] r1 = new org.bytedeco.javacpp.tools.Declarator[r1]
            java.lang.Object[] r1 = r7.toArray(r1)
            org.bytedeco.javacpp.tools.Declarator[] r1 = (org.bytedeco.javacpp.tools.Declarator[]) r1
            r5.declarators = r1
            return r5
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.parameters(org.bytedeco.javacpp.tools.Context, int, boolean):org.bytedeco.javacpp.tools.Parameters");
    }

    static String incorporateConstAnnotation(String annotations, int constValueIndex, boolean constValue) {
        String str = annotations;
        int start = annotations.indexOf("@Const");
        int end = annotations.indexOf("@", start + 1);
        if (end == -1) {
            end = annotations.length();
        }
        String prefix = annotations.substring(0, start);
        String suffix = " " + annotations.substring(end, annotations.length());
        Matcher matcher = Pattern.compile("(true|false)").matcher(annotations.substring(start, end));
        boolean[] constArray = {true, false, false};
        int index = 0;
        while (matcher.find()) {
            constArray[index] = Boolean.parseBoolean(matcher.group(1));
            index++;
        }
        constArray[constValueIndex] = constValue;
        return prefix + ("@Const({" + constArray[0] + ", " + constArray[1] + ", " + constArray[2] + "})") + suffix;
    }

    /* JADX WARNING: type inference failed for: r11v10 */
    /* JADX WARNING: type inference failed for: r11v11 */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:336:0x09bd, code lost:
        r0 = r53;
     */
    /* JADX WARNING: Incorrect type for immutable var: ssa=int, code=?, for r11v3, types: [boolean, int] */
    /* JADX WARNING: Removed duplicated region for block: B:126:0x0443  */
    /* JADX WARNING: Removed duplicated region for block: B:127:0x0452  */
    /* JADX WARNING: Removed duplicated region for block: B:131:0x045f  */
    /* JADX WARNING: Removed duplicated region for block: B:144:0x048c  */
    /* JADX WARNING: Removed duplicated region for block: B:213:0x064e A[LOOP:7: B:196:0x05d7->B:213:0x064e, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:234:0x06fb  */
    /* JADX WARNING: Removed duplicated region for block: B:244:0x075b  */
    /* JADX WARNING: Removed duplicated region for block: B:245:0x075f  */
    /* JADX WARNING: Removed duplicated region for block: B:264:0x07de  */
    /* JADX WARNING: Removed duplicated region for block: B:265:0x07e8  */
    /* JADX WARNING: Removed duplicated region for block: B:270:0x0805  */
    /* JADX WARNING: Removed duplicated region for block: B:279:0x082c  */
    /* JADX WARNING: Removed duplicated region for block: B:290:0x0863 A[LOOP:9: B:289:0x0861->B:290:0x0863, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:298:0x0913  */
    /* JADX WARNING: Removed duplicated region for block: B:304:0x0929  */
    /* JADX WARNING: Removed duplicated region for block: B:32:0x0105 A[LOOP:1: B:15:0x009b->B:32:0x0105, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:341:0x09ec  */
    /* JADX WARNING: Removed duplicated region for block: B:344:0x0a1a  */
    /* JADX WARNING: Removed duplicated region for block: B:350:0x0a3c A[LOOP:11: B:346:0x0a27->B:350:0x0a3c, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:351:0x0a43 A[EDGE_INSN: B:417:0x0a43->B:351:0x0a43 ?: BREAK  
    EDGE_INSN: B:418:0x0a43->B:351:0x0a43 ?: BREAK  ] */
    /* JADX WARNING: Removed duplicated region for block: B:353:0x0a59  */
    /* JADX WARNING: Removed duplicated region for block: B:373:0x0ae9  */
    /* JADX WARNING: Removed duplicated region for block: B:374:0x0aee A[LOOP:13: B:374:0x0aee->B:376:0x0b09, LOOP_START] */
    /* JADX WARNING: Removed duplicated region for block: B:383:0x010e A[EDGE_INSN: B:383:0x010e->B:33:0x010e ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:391:0x0486 A[EDGE_INSN: B:391:0x0486->B:142:0x0486 ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:397:0x0930 A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:409:0x0662 A[EDGE_INSN: B:409:0x0662->B:216:0x0662 ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:412:0x0744 A[SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean function(org.bytedeco.javacpp.tools.Context r59, org.bytedeco.javacpp.tools.DeclarationList r60) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r58 = this;
            r8 = r58
            r9 = r59
            r10 = r60
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            int r11 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r12 = r0.spacing
            java.lang.String r13 = "public native "
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            int r14 = r0.index
            org.bytedeco.javacpp.tools.Type r15 = r58.type(r59)
            r7 = 0
            org.bytedeco.javacpp.tools.Parameters r6 = r8.parameters(r9, r7, r7)
            org.bytedeco.javacpp.tools.Declarator r0 = new org.bytedeco.javacpp.tools.Declarator
            r0.<init>()
            r5 = r0
            org.bytedeco.javacpp.tools.Declaration r0 = new org.bytedeco.javacpp.tools.Declaration
            r0.<init>()
            r4 = r0
            java.lang.String r0 = r15.javaName
            int r0 = r0.length()
            if (r0 != 0) goto L_0x003a
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            return r7
        L_0x003a:
            java.lang.String r0 = r9.javaName
            r3 = 4
            r16 = 5
            r17 = 41
            r18 = 40
            r19 = 58
            r20 = 59
            r21 = 123(0x7b, float:1.72E-43)
            r2 = 1
            if (r0 != 0) goto L_0x014d
            boolean r0 = r15.operator
            if (r0 != 0) goto L_0x014d
            if (r6 == 0) goto L_0x014d
        L_0x0052:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r1 = new java.lang.Object[r3]
            java.lang.Character r24 = java.lang.Character.valueOf(r19)
            r1[r7] = r24
            java.lang.Character r24 = java.lang.Character.valueOf(r21)
            r1[r2] = r24
            java.lang.Character r24 = java.lang.Character.valueOf(r20)
            r23 = 2
            r1[r23] = r24
            org.bytedeco.javacpp.tools.Token r24 = org.bytedeco.javacpp.tools.Token.EOF
            r22 = 3
            r1[r22] = r24
            boolean r0 = r0.match(r1)
            if (r0 != 0) goto L_0x0080
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x0052
        L_0x0080:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r1 = new java.lang.Object[r2]
            java.lang.Character r3 = java.lang.Character.valueOf(r19)
            r1[r7] = r3
            boolean r0 = r0.match(r1)
            if (r0 == 0) goto L_0x010e
            r0 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            org.bytedeco.javacpp.tools.Token r1 = r1.next()
        L_0x009b:
            java.lang.Object[] r3 = new java.lang.Object[r2]
            org.bytedeco.javacpp.tools.Token r19 = org.bytedeco.javacpp.tools.Token.EOF
            r3[r7] = r19
            boolean r3 = r1.match(r3)
            if (r3 != 0) goto L_0x010e
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Character r19 = java.lang.Character.valueOf(r18)
            r3[r7] = r19
            boolean r3 = r1.match(r3)
            if (r3 == 0) goto L_0x00b8
            int r0 = r0 + 1
            goto L_0x00c8
        L_0x00b8:
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Character r19 = java.lang.Character.valueOf(r17)
            r3[r7] = r19
            boolean r3 = r1.match(r3)
            if (r3 == 0) goto L_0x00c8
            int r0 = r0 + -1
        L_0x00c8:
            if (r0 != 0) goto L_0x00f4
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Integer r19 = java.lang.Integer.valueOf(r16)
            r3[r7] = r19
            boolean r3 = r1.match(r3)
            if (r3 != 0) goto L_0x00f4
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.get(r2)
            r25 = r0
            java.lang.Object[] r0 = new java.lang.Object[r2]
            java.lang.Character r19 = java.lang.Character.valueOf(r21)
            r0[r7] = r19
            boolean r0 = r3.match(r0)
            if (r0 == 0) goto L_0x00f6
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x010e
        L_0x00f4:
            r25 = r0
        L_0x00f6:
            java.lang.Object[] r0 = new java.lang.Object[r2]
            java.lang.Character r3 = java.lang.Character.valueOf(r20)
            r0[r7] = r3
            boolean r0 = r1.match(r0)
            if (r0 == 0) goto L_0x0105
            goto L_0x010e
        L_0x0105:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r1 = r0.next()
            r0 = r25
            goto L_0x009b
        L_0x010e:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r1 = new java.lang.Object[r2]
            java.lang.Character r3 = java.lang.Character.valueOf(r21)
            r1[r7] = r3
            boolean r0 = r0.match(r1)
            if (r0 == 0) goto L_0x0126
            r58.body()
            goto L_0x0145
        L_0x0126:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r1 = 2
            java.lang.Object[] r3 = new java.lang.Object[r1]
            java.lang.Character r16 = java.lang.Character.valueOf(r20)
            r3[r7] = r16
            org.bytedeco.javacpp.tools.Token r16 = org.bytedeco.javacpp.tools.Token.EOF
            r3[r2] = r16
            boolean r0 = r0.match(r3)
            if (r0 != 0) goto L_0x0145
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x0126
        L_0x0145:
            r4.text = r12
            r4.function = r2
            r10.add((org.bytedeco.javacpp.tools.Declaration) r4)
            return r2
        L_0x014d:
            r1 = 2
            r22 = 3
            boolean r0 = r15.constructor
            r26 = r4
            r4 = 32
            if (r0 != 0) goto L_0x0160
            boolean r0 = r15.destructor
            if (r0 != 0) goto L_0x0160
            boolean r0 = r15.operator
            if (r0 == 0) goto L_0x01fc
        L_0x0160:
            if (r6 == 0) goto L_0x01fc
            r5.type = r15
            r5.parameters = r6
            java.lang.String r0 = r15.cppName
            r5.cppName = r0
            java.lang.String r0 = r15.javaName
            java.lang.String r1 = r15.javaName
            int r1 = r1.lastIndexOf(r4)
            int r1 = r1 + r2
            java.lang.String r0 = r0.substring(r1)
            r5.javaName = r0
            boolean r0 = r15.operator
            if (r0 == 0) goto L_0x01df
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = "operator "
            r0.append(r1)
            org.bytedeco.javacpp.tools.Type r1 = r5.type
            boolean r1 = r1.constValue
            if (r1 == 0) goto L_0x0190
            java.lang.String r1 = "const "
            goto L_0x0192
        L_0x0190:
            java.lang.String r1 = ""
        L_0x0192:
            r0.append(r1)
            org.bytedeco.javacpp.tools.Type r1 = r5.type
            java.lang.String r1 = r1.cppName
            r0.append(r1)
            org.bytedeco.javacpp.tools.Type r1 = r5.type
            int r1 = r1.indirections
            if (r1 <= 0) goto L_0x01a5
            java.lang.String r1 = "*"
            goto L_0x01b0
        L_0x01a5:
            org.bytedeco.javacpp.tools.Type r1 = r5.type
            boolean r1 = r1.reference
            if (r1 == 0) goto L_0x01ae
            java.lang.String r1 = "&"
            goto L_0x01b0
        L_0x01ae:
            java.lang.String r1 = ""
        L_0x01b0:
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r5.cppName = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = "as"
            r0.append(r1)
            java.lang.String r1 = r5.javaName
            char r1 = r1.charAt(r7)
            char r1 = java.lang.Character.toUpperCase(r1)
            r0.append(r1)
            java.lang.String r1 = r5.javaName
            java.lang.String r1 = r1.substring(r2)
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r5.javaName = r0
        L_0x01df:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r5.javaName
            r0.append(r1)
            java.lang.String r1 = r6.signature
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r5.signature = r0
            r23 = r6
            r36 = r13
            r34 = r26
            r13 = 0
            goto L_0x022b
        L_0x01fc:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r14
            r23 = 0
            r24 = 0
            r25 = 0
            r28 = 0
            r29 = 0
            r30 = 0
            r0 = r58
            r1 = r59
            r2 = r23
            r3 = r24
            r34 = r26
            r4 = r25
            r22 = r5
            r5 = r28
            r23 = r6
            r6 = r29
            r36 = r13
            r13 = 0
            r7 = r30
            org.bytedeco.javacpp.tools.Declarator r5 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            org.bytedeco.javacpp.tools.Type r15 = r5.type
        L_0x022b:
            java.lang.String r0 = r5.cppName
            if (r0 == 0) goto L_0x0b19
            java.lang.String r0 = r15.javaName
            int r0 = r0.length()
            if (r0 == 0) goto L_0x0b19
            org.bytedeco.javacpp.tools.Parameters r0 = r5.parameters
            if (r0 != 0) goto L_0x0241
            r43 = r11
            r0 = r34
            goto L_0x0b1d
        L_0x0241:
            java.lang.String r0 = r5.cppName
            java.lang.String r1 = "::"
            int r0 = r0.lastIndexOf(r1)
            java.lang.String r1 = r9.namespace
            if (r1 == 0) goto L_0x0269
            if (r0 >= 0) goto L_0x0269
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = r9.namespace
            r1.append(r2)
            java.lang.String r2 = "::"
            r1.append(r2)
            java.lang.String r2 = r5.cppName
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r5.cppName = r1
        L_0x0269:
            r1 = 0
            r2 = 0
            java.lang.String r3 = r5.cppName
            java.lang.String r4 = r5.cppName
            org.bytedeco.javacpp.tools.Parameters r6 = r5.parameters
            if (r6 == 0) goto L_0x03e0
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r3)
            java.lang.String r7 = "("
            r6.append(r7)
            java.lang.String r3 = r6.toString()
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r4)
            java.lang.String r7 = "("
            r6.append(r7)
            java.lang.String r4 = r6.toString()
            java.lang.String r6 = ""
            org.bytedeco.javacpp.tools.Parameters r7 = r5.parameters
            org.bytedeco.javacpp.tools.Declarator[] r7 = r7.declarators
            int r13 = r7.length
            r37 = r0
            r0 = r4
            r4 = r3
            r3 = 0
        L_0x02a1:
            if (r3 >= r13) goto L_0x03a2
            r38 = r1
            r1 = r7[r3]
            if (r1 == 0) goto L_0x038c
            r39 = r2
            org.bytedeco.javacpp.tools.Type r2 = r1.type
            java.lang.String r2 = r2.cppName
            r40 = r7
            org.bytedeco.javacpp.tools.Type r7 = r1.type
            java.lang.String r7 = r7.cppName
            r41 = r7
            org.bytedeco.javacpp.tools.Type r7 = r1.type
            boolean r7 = r7.constValue
            if (r7 == 0) goto L_0x02d9
            java.lang.String r7 = "const "
            boolean r7 = r2.startsWith(r7)
            if (r7 != 0) goto L_0x02d9
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r42 = r13
            java.lang.String r13 = "const "
            r7.append(r13)
            r7.append(r2)
            java.lang.String r2 = r7.toString()
            goto L_0x02db
        L_0x02d9:
            r42 = r13
        L_0x02db:
            org.bytedeco.javacpp.tools.Type r7 = r1.type
            boolean r7 = r7.constPointer
            if (r7 == 0) goto L_0x02fa
            java.lang.String r7 = " const"
            boolean r7 = r2.endsWith(r7)
            if (r7 != 0) goto L_0x02fa
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r2)
            java.lang.String r13 = " const"
            r7.append(r13)
            java.lang.String r2 = r7.toString()
        L_0x02fa:
            int r7 = r1.indirections
            if (r7 <= 0) goto L_0x033a
            r7 = r2
            r13 = r41
            r2 = 0
        L_0x0302:
            r43 = r11
            int r11 = r1.indirections
            if (r2 >= r11) goto L_0x0335
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r7)
            r44 = r7
            java.lang.String r7 = "*"
            r11.append(r7)
            java.lang.String r7 = r11.toString()
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r13)
            r45 = r7
            java.lang.String r7 = "*"
            r11.append(r7)
            java.lang.String r13 = r11.toString()
            int r2 = r2 + 1
            r11 = r43
            r7 = r45
            goto L_0x0302
        L_0x0335:
            r44 = r7
            r2 = r44
            goto L_0x033e
        L_0x033a:
            r43 = r11
            r13 = r41
        L_0x033e:
            boolean r7 = r1.reference
            if (r7 == 0) goto L_0x0364
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r2)
            java.lang.String r11 = "&"
            r7.append(r11)
            java.lang.String r2 = r7.toString()
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r13)
            java.lang.String r11 = "&"
            r7.append(r11)
            java.lang.String r13 = r7.toString()
        L_0x0364:
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r4)
            r7.append(r6)
            r7.append(r2)
            java.lang.String r4 = r7.toString()
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r0)
            r7.append(r6)
            r7.append(r13)
            java.lang.String r0 = r7.toString()
            java.lang.String r1 = ", "
            r6 = r1
            goto L_0x0394
        L_0x038c:
            r39 = r2
            r40 = r7
            r43 = r11
            r42 = r13
        L_0x0394:
            int r3 = r3 + 1
            r1 = r38
            r2 = r39
            r7 = r40
            r13 = r42
            r11 = r43
            goto L_0x02a1
        L_0x03a2:
            r38 = r1
            r39 = r2
            r43 = r11
            org.bytedeco.javacpp.tools.InfoMap r1 = r8.infoMap
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r2.append(r4)
            java.lang.String r3 = ")"
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r3 = r2
            org.bytedeco.javacpp.tools.Info r1 = r1.getFirst(r2)
            r2 = r1
            if (r1 != 0) goto L_0x03db
            org.bytedeco.javacpp.tools.InfoMap r4 = r8.infoMap
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r0)
            java.lang.String r11 = ")"
            r7.append(r11)
            java.lang.String r7 = r7.toString()
            r0 = r7
            org.bytedeco.javacpp.tools.Info r1 = r4.getFirst(r7)
        L_0x03db:
            r22 = r0
            r13 = r2
            r11 = r3
            goto L_0x03ed
        L_0x03e0:
            r37 = r0
            r38 = r1
            r39 = r2
            r43 = r11
            r11 = r3
            r22 = r4
            r13 = r39
        L_0x03ed:
            if (r1 != 0) goto L_0x0426
            org.bytedeco.javacpp.tools.InfoMap r0 = r8.infoMap
            java.lang.String r2 = r5.cppName
            org.bytedeco.javacpp.tools.Info r1 = r0.getFirst(r2)
            boolean r0 = r15.constructor
            if (r0 != 0) goto L_0x0426
            boolean r0 = r15.destructor
            if (r0 != 0) goto L_0x0426
            boolean r0 = r15.operator
            if (r0 != 0) goto L_0x0426
            org.bytedeco.javacpp.tools.InfoMap r0 = r8.infoMap
            if (r1 == 0) goto L_0x0417
            org.bytedeco.javacpp.tools.Info r2 = new org.bytedeco.javacpp.tools.Info
            r2.<init>((org.bytedeco.javacpp.tools.Info) r1)
            r7 = 1
            java.lang.String[] r3 = new java.lang.String[r7]
            r4 = 0
            r3[r4] = r11
            org.bytedeco.javacpp.tools.Info r2 = r2.cppNames(r3)
            goto L_0x0422
        L_0x0417:
            r4 = 0
            r7 = 1
            org.bytedeco.javacpp.tools.Info r2 = new org.bytedeco.javacpp.tools.Info
            java.lang.String[] r3 = new java.lang.String[r7]
            r3[r4] = r11
            r2.<init>((java.lang.String[]) r3)
        L_0x0422:
            r0.put(r2)
            goto L_0x0427
        L_0x0426:
            r7 = 1
        L_0x0427:
            r6 = r1
            java.lang.String r0 = r5.cppName
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = r9.namespace
            r1.append(r2)
            java.lang.String r2 = "::"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            boolean r1 = r0.startsWith(r1)
            if (r1 == 0) goto L_0x0452
            java.lang.String r1 = r5.cppName
            java.lang.String r2 = r9.namespace
            int r2 = r2.length()
            r4 = 2
            int r2 = r2 + r4
            java.lang.String r0 = r1.substring(r2)
            goto L_0x0453
        L_0x0452:
            r4 = 2
        L_0x0453:
            r3 = r0
            r0 = 0
            r1 = 0
            r24 = r1
            r1 = 0
        L_0x0459:
            int r2 = r3.length()
            if (r1 >= r2) goto L_0x0486
            char r2 = r3.charAt(r1)
            r4 = 60
            if (r2 != r4) goto L_0x046a
            int r24 = r24 + 1
            goto L_0x0481
        L_0x046a:
            r4 = 62
            if (r2 != r4) goto L_0x0471
            int r24 = r24 + -1
            goto L_0x0481
        L_0x0471:
            if (r24 != 0) goto L_0x0481
            java.lang.String r4 = r3.substring(r1)
            java.lang.String r7 = "::"
            boolean r4 = r4.startsWith(r7)
            if (r4 == 0) goto L_0x0481
            r0 = r1
            goto L_0x0486
        L_0x0481:
            int r1 = r1 + 1
            r4 = 2
            r7 = 1
            goto L_0x0459
        L_0x0486:
            r25 = r0
            boolean r0 = r15.friend
            if (r0 != 0) goto L_0x09ec
            java.lang.String r0 = r9.javaName
            if (r0 != 0) goto L_0x049b
            if (r25 > 0) goto L_0x0493
            goto L_0x049b
        L_0x0493:
            r35 = r3
            r4 = r6
            r54 = r11
        L_0x0498:
            r2 = 1
            goto L_0x09f2
        L_0x049b:
            if (r6 == 0) goto L_0x04a2
            boolean r0 = r6.skip
            if (r0 == 0) goto L_0x04a2
            goto L_0x0493
        L_0x04a2:
            boolean r0 = r15.staticMember
            if (r0 != 0) goto L_0x04ae
            java.lang.String r0 = r9.javaName
            if (r0 != 0) goto L_0x04ab
            goto L_0x04ae
        L_0x04ab:
            r0 = r36
            goto L_0x04c7
        L_0x04ae:
            java.lang.String r0 = "public static native "
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            boolean r1 = r1.isCFile
            if (r1 == 0) goto L_0x04c7
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "@NoException "
            r1.append(r2)
            r1.append(r0)
            java.lang.String r0 = r1.toString()
        L_0x04c7:
            java.util.ArrayList r1 = new java.util.ArrayList
            r1.<init>()
            r7 = r1
            r1 = 1
            r2 = -2
            r27 = r0
            r28 = r1
            r26 = r5
        L_0x04d5:
            r5 = r2
            r0 = 2147483647(0x7fffffff, float:NaN)
            if (r5 >= r0) goto L_0x09d8
            org.bytedeco.javacpp.tools.Declaration r0 = new org.bytedeco.javacpp.tools.Declaration
            r0.<init>()
            r4 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r14
            if (r6 == 0) goto L_0x04eb
            boolean r0 = r6.skipDefaults
            if (r0 != 0) goto L_0x04f1
        L_0x04eb:
            int r0 = r5 % 2
            if (r0 == 0) goto L_0x04f1
            r0 = 1
            goto L_0x04f2
        L_0x04f1:
            r0 = 0
        L_0x04f2:
            r2 = r0
            boolean r0 = r15.constructor
            if (r0 != 0) goto L_0x0509
            boolean r0 = r15.destructor
            if (r0 != 0) goto L_0x0509
            boolean r0 = r15.operator
            if (r0 == 0) goto L_0x0500
            goto L_0x0509
        L_0x0500:
            r46 = r2
            r47 = r3
            r48 = r7
            r7 = 1
            goto L_0x0680
        L_0x0509:
            if (r23 == 0) goto L_0x0679
            org.bytedeco.javacpp.tools.Type r0 = r58.type(r59)
            int r1 = r5 / 2
            org.bytedeco.javacpp.tools.Parameters r1 = r8.parameters(r9, r1, r2)
            org.bytedeco.javacpp.tools.Declarator r15 = new org.bytedeco.javacpp.tools.Declarator
            r15.<init>()
            r15.type = r0
            r15.parameters = r1
            r46 = r2
            java.lang.String r2 = r0.cppName
            r15.cppName = r2
            java.lang.String r2 = r0.javaName
            r47 = r3
            java.lang.String r3 = r0.javaName
            r48 = r7
            r7 = 32
            int r3 = r3.lastIndexOf(r7)
            r23 = 1
            int r3 = r3 + 1
            java.lang.String r2 = r2.substring(r3)
            r15.javaName = r2
            boolean r2 = r0.operator
            if (r2 == 0) goto L_0x05a4
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "operator "
            r2.append(r3)
            org.bytedeco.javacpp.tools.Type r3 = r15.type
            boolean r3 = r3.constValue
            if (r3 == 0) goto L_0x0553
            java.lang.String r3 = "const "
            goto L_0x0555
        L_0x0553:
            java.lang.String r3 = ""
        L_0x0555:
            r2.append(r3)
            org.bytedeco.javacpp.tools.Type r3 = r15.type
            java.lang.String r3 = r3.cppName
            r2.append(r3)
            org.bytedeco.javacpp.tools.Type r3 = r15.type
            int r3 = r3.indirections
            if (r3 <= 0) goto L_0x0568
            java.lang.String r3 = "*"
            goto L_0x0573
        L_0x0568:
            org.bytedeco.javacpp.tools.Type r3 = r15.type
            boolean r3 = r3.reference
            if (r3 == 0) goto L_0x0571
            java.lang.String r3 = "&"
            goto L_0x0573
        L_0x0571:
            java.lang.String r3 = ""
        L_0x0573:
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r15.cppName = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "as"
            r2.append(r3)
            java.lang.String r3 = r15.javaName
            r7 = 0
            char r3 = r3.charAt(r7)
            char r3 = java.lang.Character.toUpperCase(r3)
            r2.append(r3)
            java.lang.String r3 = r15.javaName
            r7 = 1
            java.lang.String r3 = r3.substring(r7)
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r15.javaName = r2
        L_0x05a4:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = r15.javaName
            r2.append(r3)
            java.lang.String r3 = r1.signature
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r15.signature = r2
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.get()
            r3 = 1
            java.lang.Object[] r7 = new java.lang.Object[r3]
            java.lang.Character r3 = java.lang.Character.valueOf(r19)
            r23 = 0
            r7[r23] = r3
            boolean r2 = r2.match(r7)
            if (r2 == 0) goto L_0x065d
            r2 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.next()
        L_0x05d7:
            r49 = r0
            r7 = 1
            java.lang.Object[] r0 = new java.lang.Object[r7]
            org.bytedeco.javacpp.tools.Token r23 = org.bytedeco.javacpp.tools.Token.EOF
            r26 = 0
            r0[r26] = r23
            boolean r0 = r3.match(r0)
            if (r0 != 0) goto L_0x065a
            java.lang.Object[] r0 = new java.lang.Object[r7]
            java.lang.Character r23 = java.lang.Character.valueOf(r18)
            r0[r26] = r23
            boolean r0 = r3.match(r0)
            if (r0 == 0) goto L_0x05f9
            int r2 = r2 + 1
            goto L_0x0609
        L_0x05f9:
            java.lang.Object[] r0 = new java.lang.Object[r7]
            java.lang.Character r7 = java.lang.Character.valueOf(r17)
            r0[r26] = r7
            boolean r0 = r3.match(r0)
            if (r0 == 0) goto L_0x0609
            int r2 = r2 + -1
        L_0x0609:
            if (r2 != 0) goto L_0x063a
            r0 = 1
            java.lang.Object[] r7 = new java.lang.Object[r0]
            java.lang.Integer r23 = java.lang.Integer.valueOf(r16)
            r26 = 0
            r7[r26] = r23
            boolean r7 = r3.match(r7)
            if (r7 != 0) goto L_0x063a
            org.bytedeco.javacpp.tools.TokenIndexer r7 = r8.tokens
            org.bytedeco.javacpp.tools.Token r7 = r7.get(r0)
            r50 = r1
            java.lang.Object[] r1 = new java.lang.Object[r0]
            java.lang.Character r0 = java.lang.Character.valueOf(r21)
            r1[r26] = r0
            boolean r0 = r7.match(r1)
            if (r0 == 0) goto L_0x063c
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            r7 = 1
            goto L_0x065c
        L_0x063a:
            r50 = r1
        L_0x063c:
            r7 = 1
            java.lang.Object[] r0 = new java.lang.Object[r7]
            java.lang.Character r1 = java.lang.Character.valueOf(r20)
            r23 = 0
            r0[r23] = r1
            boolean r0 = r3.match(r0)
            if (r0 == 0) goto L_0x064e
            goto L_0x065c
        L_0x064e:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r0.next()
            r0 = r49
            r1 = r50
            goto L_0x05d7
        L_0x065a:
            r50 = r1
        L_0x065c:
            goto L_0x0662
        L_0x065d:
            r49 = r0
            r50 = r1
            r7 = 1
        L_0x0662:
            r51 = r4
            r29 = r5
            r52 = r6
            r54 = r11
            r33 = r46
            r35 = r47
            r53 = r48
            r1 = r49
            r0 = r50
            r11 = 1
            r30 = 32
            goto L_0x06e8
        L_0x0679:
            r46 = r2
            r47 = r3
            r48 = r7
            r7 = 1
        L_0x0680:
            r2 = 0
            int r3 = r5 / 2
            if (r6 == 0) goto L_0x0689
            boolean r0 = r6.skipDefaults
            if (r0 != 0) goto L_0x0690
        L_0x0689:
            int r0 = r5 % 2
            if (r0 == 0) goto L_0x0690
            r29 = 1
            goto L_0x0692
        L_0x0690:
            r29 = 0
        L_0x0692:
            r30 = 0
            r31 = 0
            r32 = 0
            r0 = r58
            r1 = r59
            r33 = r46
            r35 = r47
            r51 = r4
            r4 = r29
            r29 = r5
            r5 = r30
            r52 = r6
            r6 = r31
            r54 = r11
            r53 = r48
            r11 = 1
            r30 = 32
            r7 = r32
            org.bytedeco.javacpp.tools.Declarator r0 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            org.bytedeco.javacpp.tools.Type r1 = r0.type
            java.lang.String r2 = r0.cppName
            java.lang.String r3 = "::"
            int r2 = r2.lastIndexOf(r3)
            java.lang.String r3 = r9.namespace
            if (r3 == 0) goto L_0x06e3
            if (r2 >= 0) goto L_0x06e3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r9.namespace
            r3.append(r4)
            java.lang.String r4 = "::"
            r3.append(r4)
            java.lang.String r4 = r0.cppName
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r0.cppName = r3
        L_0x06e3:
            r15 = r0
            r37 = r2
            r0 = r23
        L_0x06e8:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.get()
        L_0x06ee:
            java.lang.Object[] r3 = new java.lang.Object[r11]
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.EOF
            r5 = 0
            r3[r5] = r4
            boolean r3 = r2.match(r3)
            if (r3 != 0) goto L_0x0744
            r3 = r51
            boolean r4 = r3.constMember
            r6 = 3
            java.lang.Object[] r7 = new java.lang.Object[r6]
            org.bytedeco.javacpp.tools.Token r23 = org.bytedeco.javacpp.tools.Token.CONST
            r7[r5] = r23
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.__CONST
            r7[r11] = r5
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.CONSTEXPR
            r6 = 2
            r7[r6] = r5
            boolean r5 = r2.match(r7)
            r4 = r4 | r5
            r3.constMember = r4
            org.bytedeco.javacpp.tools.Attribute r4 = r58.attribute()
            if (r4 == 0) goto L_0x0738
            boolean r5 = r4.annotation
            if (r5 == 0) goto L_0x0738
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            org.bytedeco.javacpp.tools.Type r7 = r15.type
            java.lang.String r6 = r7.annotations
            r5.append(r6)
            java.lang.String r6 = r4.javaName
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r7.annotations = r5
            goto L_0x073b
        L_0x0738:
            if (r4 != 0) goto L_0x073b
            goto L_0x0746
        L_0x073b:
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r4.get()
            r51 = r3
            goto L_0x06ee
        L_0x0744:
            r3 = r51
        L_0x0746:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.get()
            java.lang.Object[] r4 = new java.lang.Object[r11]
            java.lang.Character r5 = java.lang.Character.valueOf(r21)
            r6 = 0
            r4[r6] = r5
            boolean r2 = r2.match(r4)
            if (r2 == 0) goto L_0x075f
            r58.body()
            goto L_0x07c8
        L_0x075f:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.get()
            java.lang.Object[] r4 = new java.lang.Object[r11]
            r5 = 61
            java.lang.Character r5 = java.lang.Character.valueOf(r5)
            r6 = 0
            r4[r6] = r5
            boolean r2 = r2.match(r4)
            if (r2 == 0) goto L_0x07c3
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.next()
            r4 = 3
            java.lang.Object[] r5 = new java.lang.Object[r4]
            java.lang.String r4 = "0"
            r5[r6] = r4
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.DELETE
            r5[r11] = r4
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.DEFAULT
            r7 = 2
            r5[r7] = r4
            org.bytedeco.javacpp.tools.Token r2 = r2.expect(r5)
            java.lang.Object[] r4 = new java.lang.Object[r11]
            java.lang.String r5 = "0"
            r4[r6] = r5
            boolean r4 = r2.match(r4)
            if (r4 == 0) goto L_0x079f
            r3.abstractMember = r11
            goto L_0x07b1
        L_0x079f:
            java.lang.Object[] r4 = new java.lang.Object[r11]
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.DELETE
            r4[r6] = r5
            boolean r4 = r2.match(r4)
            if (r4 == 0) goto L_0x07b1
            r3.text = r12
            r10.add((org.bytedeco.javacpp.tools.Declaration) r3)
            return r11
        L_0x07b1:
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r8.tokens
            org.bytedeco.javacpp.tools.Token r4 = r4.next()
            java.lang.Object[] r5 = new java.lang.Object[r11]
            java.lang.Character r6 = java.lang.Character.valueOf(r20)
            r7 = 0
            r5[r7] = r6
            r4.expect(r5)
        L_0x07c3:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            r2.next()
        L_0x07c8:
            boolean r2 = r3.constMember
            if (r2 == 0) goto L_0x07fd
            boolean r2 = r1.virtual
            if (r2 == 0) goto L_0x07fd
            boolean r2 = r9.virtualize
            if (r2 == 0) goto L_0x07fd
            java.lang.String r2 = r1.annotations
            java.lang.String r4 = "@Const"
            boolean r2 = r2.contains(r4)
            if (r2 == 0) goto L_0x07e8
            java.lang.String r2 = r1.annotations
            r4 = 2
            java.lang.String r2 = incorporateConstAnnotation(r2, r4, r11)
            r1.annotations = r2
            goto L_0x07fd
        L_0x07e8:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r4 = r1.annotations
            r2.append(r4)
            java.lang.String r4 = "@Const({false, false, true}) "
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            r1.annotations = r2
        L_0x07fd:
            boolean r2 = r1.virtual
            if (r2 == 0) goto L_0x082c
            boolean r2 = r9.virtualize
            if (r2 == 0) goto L_0x082c
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r4 = "@Virtual"
            r2.append(r4)
            boolean r4 = r3.abstractMember
            if (r4 == 0) goto L_0x0816
            java.lang.String r4 = "(true) "
            goto L_0x0818
        L_0x0816:
            java.lang.String r4 = " "
        L_0x0818:
            r2.append(r4)
            boolean r4 = r9.inaccessible
            if (r4 == 0) goto L_0x0822
            java.lang.String r4 = "protected native "
            goto L_0x0824
        L_0x0822:
            java.lang.String r4 = "public native "
        L_0x0824:
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            goto L_0x082e
        L_0x082c:
            r2 = r27
        L_0x082e:
            r3.declarator = r15
            java.lang.String r4 = r9.namespace
            if (r4 == 0) goto L_0x0857
            java.lang.String r4 = r9.javaName
            if (r4 != 0) goto L_0x0857
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r3.text
            r4.append(r5)
            java.lang.String r5 = "@Namespace(\""
            r4.append(r5)
            java.lang.String r5 = r9.namespace
            r4.append(r5)
            java.lang.String r5 = "\") "
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.text = r4
        L_0x0857:
            if (r13 == 0) goto L_0x0885
            java.lang.String[] r4 = r13.annotations
            if (r4 == 0) goto L_0x0885
            java.lang.String[] r4 = r13.annotations
            int r5 = r4.length
            r6 = 0
        L_0x0861:
            if (r6 >= r5) goto L_0x0885
            r7 = r4[r6]
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r55 = r4
            java.lang.String r4 = r3.text
            r11.append(r4)
            r11.append(r7)
            java.lang.String r4 = " "
            r11.append(r4)
            java.lang.String r4 = r11.toString()
            r3.text = r4
            int r6 = r6 + 1
            r4 = r55
            r11 = 1
            goto L_0x0861
        L_0x0885:
            boolean r4 = r1.constructor
            if (r4 == 0) goto L_0x08d6
            if (r0 == 0) goto L_0x08d6
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r3.text
            r4.append(r5)
            java.lang.String r5 = "public "
            r4.append(r5)
            java.lang.String r5 = r9.javaName
            java.lang.String r5 = r9.shorten(r5)
            r4.append(r5)
            org.bytedeco.javacpp.tools.Parameters r5 = r15.parameters
            java.lang.String r5 = r5.list
            r4.append(r5)
            java.lang.String r5 = " { super((Pointer)null); allocate"
            r4.append(r5)
            java.lang.String r5 = r0.names
            r4.append(r5)
            java.lang.String r5 = "; }\nprivate native "
            r4.append(r5)
            java.lang.String r5 = r1.annotations
            r4.append(r5)
            java.lang.String r5 = "void allocate"
            r4.append(r5)
            org.bytedeco.javacpp.tools.Parameters r5 = r15.parameters
            java.lang.String r5 = r5.list
            r4.append(r5)
            java.lang.String r5 = ";\n"
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.text = r4
            goto L_0x090d
        L_0x08d6:
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r3.text
            r4.append(r5)
            r4.append(r2)
            java.lang.String r5 = r1.annotations
            r4.append(r5)
            java.lang.String r5 = r1.javaName
            java.lang.String r5 = r9.shorten(r5)
            r4.append(r5)
            java.lang.String r5 = " "
            r4.append(r5)
            java.lang.String r5 = r15.javaName
            r4.append(r5)
            org.bytedeco.javacpp.tools.Parameters r5 = r15.parameters
            java.lang.String r5 = r5.list
            r4.append(r5)
            java.lang.String r5 = ";\n"
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.text = r4
        L_0x090d:
            java.lang.String r4 = r15.signature
            r3.signature = r4
            if (r33 == 0) goto L_0x091f
            java.lang.String r4 = r3.text
            java.lang.String r5 = "@Override "
            java.lang.String r6 = ""
            java.lang.String r4 = r4.replaceAll(r5, r6)
            r3.text = r4
        L_0x091f:
            r4 = r52
            if (r4 == 0) goto L_0x0936
            java.lang.String r5 = r4.javaText
            if (r5 == 0) goto L_0x0936
            if (r28 == 0) goto L_0x0930
            java.lang.String r5 = r4.javaText
            r3.text = r5
            r3.signature = r5
            goto L_0x0936
        L_0x0930:
            r56 = r0
            r57 = r2
            goto L_0x09bd
        L_0x0936:
            java.lang.String r5 = r58.commentAfter()
            if (r28 == 0) goto L_0x0951
            r10.spacing = r12
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r5)
            java.lang.String r7 = r3.text
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r3.text = r6
        L_0x0951:
            r6 = 1
            r3.function = r6
            r6 = 0
            java.util.Iterator r7 = r53.iterator()
        L_0x0959:
            boolean r11 = r7.hasNext()
            if (r11 == 0) goto L_0x0978
            java.lang.Object r11 = r7.next()
            org.bytedeco.javacpp.tools.Declarator r11 = (org.bytedeco.javacpp.tools.Declarator) r11
            r56 = r0
            java.lang.String r0 = r15.signature
            r57 = r2
            java.lang.String r2 = r11.signature
            boolean r0 = r0.equals(r2)
            r6 = r6 | r0
            r0 = r56
            r2 = r57
            goto L_0x0959
        L_0x0978:
            r56 = r0
            r57 = r2
            java.lang.String r0 = r15.javaName
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x09a4
            if (r6 != 0) goto L_0x09a4
            boolean r0 = r1.destructor
            if (r0 == 0) goto L_0x0990
            if (r4 == 0) goto L_0x09a4
            java.lang.String r0 = r4.javaText
            if (r0 == 0) goto L_0x09a4
        L_0x0990:
            boolean r0 = r10.add((org.bytedeco.javacpp.tools.Declaration) r3)
            if (r0 == 0) goto L_0x0998
            r28 = 0
        L_0x0998:
            boolean r0 = r1.virtual
            if (r0 == 0) goto L_0x09c0
            boolean r0 = r9.virtualize
            if (r0 == 0) goto L_0x09c0
            r0 = r53
            goto L_0x09e7
        L_0x09a4:
            if (r6 == 0) goto L_0x09c0
            int r0 = r29 / 2
            if (r0 <= 0) goto L_0x09c0
            int r0 = r29 % 2
            if (r0 != 0) goto L_0x09c0
            int r0 = r29 / 2
            int r2 = r15.infoNumber
            org.bytedeco.javacpp.tools.Parameters r7 = r15.parameters
            int r7 = r7.infoNumber
            int r2 = java.lang.Math.max(r2, r7)
            if (r0 <= r2) goto L_0x09c0
        L_0x09bd:
            r0 = r53
            goto L_0x09e7
        L_0x09c0:
            r0 = r53
            r0.add(r15)
            int r2 = r29 + 1
            r7 = r0
            r34 = r3
            r6 = r4
            r26 = r15
            r3 = r35
            r11 = r54
            r23 = r56
            r27 = r57
            r15 = r1
            goto L_0x04d5
        L_0x09d8:
            r35 = r3
            r4 = r6
            r0 = r7
            r54 = r11
            r1 = r15
            r56 = r23
            r15 = r26
            r57 = r27
            r3 = r34
        L_0x09e7:
            r2 = 0
            r10.spacing = r2
            r2 = 1
            return r2
        L_0x09ec:
            r35 = r3
            r4 = r6
            r54 = r11
            r2 = 1
        L_0x09f2:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r1 = 4
            java.lang.Object[] r3 = new java.lang.Object[r1]
            java.lang.Character r6 = java.lang.Character.valueOf(r19)
            r7 = 0
            r3[r7] = r6
            java.lang.Character r6 = java.lang.Character.valueOf(r21)
            r3[r2] = r6
            java.lang.Character r2 = java.lang.Character.valueOf(r20)
            r6 = 2
            r3[r6] = r2
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.EOF
            r6 = 3
            r3[r6] = r2
            boolean r0 = r0.match(r3)
            if (r0 != 0) goto L_0x0a21
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x0498
        L_0x0a21:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
        L_0x0a27:
            r1 = 1
            java.lang.Object[] r2 = new java.lang.Object[r1]
            org.bytedeco.javacpp.tools.Token r1 = org.bytedeco.javacpp.tools.Token.EOF
            r3 = 0
            r2[r3] = r1
            boolean r1 = r0.match(r2)
            if (r1 != 0) goto L_0x0a43
            org.bytedeco.javacpp.tools.Attribute r1 = r58.attribute()
            if (r1 != 0) goto L_0x0a3c
            goto L_0x0a43
        L_0x0a3c:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r1.get()
            goto L_0x0a27
        L_0x0a43:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r1 = 1
            java.lang.Object[] r2 = new java.lang.Object[r1]
            java.lang.Character r1 = java.lang.Character.valueOf(r19)
            r3 = 0
            r2[r3] = r1
            boolean r0 = r0.match(r2)
            if (r0 == 0) goto L_0x0ad3
            r0 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            org.bytedeco.javacpp.tools.Token r1 = r1.next()
        L_0x0a60:
            r2 = 1
            java.lang.Object[] r3 = new java.lang.Object[r2]
            org.bytedeco.javacpp.tools.Token r6 = org.bytedeco.javacpp.tools.Token.EOF
            r7 = 0
            r3[r7] = r6
            boolean r3 = r1.match(r3)
            if (r3 != 0) goto L_0x0ad3
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Character r6 = java.lang.Character.valueOf(r18)
            r3[r7] = r6
            boolean r3 = r1.match(r3)
            if (r3 == 0) goto L_0x0a7f
            int r0 = r0 + 1
            goto L_0x0a8f
        L_0x0a7f:
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Character r2 = java.lang.Character.valueOf(r17)
            r3[r7] = r2
            boolean r2 = r1.match(r3)
            if (r2 == 0) goto L_0x0a8f
            int r0 = r0 + -1
        L_0x0a8f:
            if (r0 != 0) goto L_0x0abb
            r2 = 1
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Integer r6 = java.lang.Integer.valueOf(r16)
            r7 = 0
            r3[r7] = r6
            boolean r3 = r1.match(r3)
            if (r3 != 0) goto L_0x0abb
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.get(r2)
            java.lang.Object[] r6 = new java.lang.Object[r2]
            java.lang.Character r2 = java.lang.Character.valueOf(r21)
            r6[r7] = r2
            boolean r2 = r3.match(r6)
            if (r2 == 0) goto L_0x0abb
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            r2.next()
            goto L_0x0ad3
        L_0x0abb:
            r2 = 1
            java.lang.Object[] r3 = new java.lang.Object[r2]
            java.lang.Character r2 = java.lang.Character.valueOf(r20)
            r6 = 0
            r3[r6] = r2
            boolean r2 = r1.match(r3)
            if (r2 == 0) goto L_0x0acc
            goto L_0x0ad3
        L_0x0acc:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r1 = r2.next()
            goto L_0x0a60
        L_0x0ad3:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r1 = 1
            java.lang.Object[] r2 = new java.lang.Object[r1]
            java.lang.Character r1 = java.lang.Character.valueOf(r21)
            r3 = 0
            r2[r3] = r1
            boolean r0 = r0.match(r2)
            if (r0 == 0) goto L_0x0aee
            r58.body()
            r6 = 1
            goto L_0x0b0f
        L_0x0aee:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r1 = 2
            java.lang.Object[] r2 = new java.lang.Object[r1]
            java.lang.Character r3 = java.lang.Character.valueOf(r20)
            r6 = 0
            r2[r6] = r3
            org.bytedeco.javacpp.tools.Token r3 = org.bytedeco.javacpp.tools.Token.EOF
            r6 = 1
            r2[r6] = r3
            boolean r0 = r0.match(r2)
            if (r0 != 0) goto L_0x0b0f
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x0aee
        L_0x0b0f:
            r0 = r34
            r0.text = r12
            r0.function = r6
            r10.add((org.bytedeco.javacpp.tools.Declaration) r0)
            return r6
        L_0x0b19:
            r43 = r11
            r0 = r34
        L_0x0b1d:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            r2 = r43
            r1.index = r2
            r1 = 0
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.function(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):boolean");
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:165:0x061e  */
    /* JADX WARNING: Removed duplicated region for block: B:201:0x022f A[EDGE_INSN: B:201:0x022f->B:67:0x022f ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:63:0x01f9  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean variable(org.bytedeco.javacpp.tools.Context r44, org.bytedeco.javacpp.tools.DeclarationList r45) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r43 = this;
            r8 = r43
            r9 = r44
            r10 = r45
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            int r11 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r12 = r0.spacing
            java.lang.String r13 = "public static native "
            java.lang.String r14 = "void "
            r2 = 0
            r3 = 0
            r4 = 0
            r5 = 0
            r6 = 0
            r7 = 1
            r0 = r43
            r1 = r44
            org.bytedeco.javacpp.tools.Declarator r0 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            org.bytedeco.javacpp.tools.Declaration r1 = new org.bytedeco.javacpp.tools.Declaration
            r1.<init>()
            java.lang.String r2 = r0.cppName
            java.lang.String r3 = r0.javaName
            org.bytedeco.javacpp.tools.Attribute r15 = r43.attribute()
            if (r15 == 0) goto L_0x004e
            boolean r4 = r15.annotation
            if (r4 == 0) goto L_0x004e
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r6 = r5.annotations
            r4.append(r6)
            java.lang.String r6 = r15.javaName
            r4.append(r6)
            java.lang.String r4 = r4.toString()
            r5.annotations = r4
        L_0x004e:
            r16 = 0
            if (r2 == 0) goto L_0x073e
            if (r3 == 0) goto L_0x073e
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r8.tokens
            org.bytedeco.javacpp.tools.Token r4 = r4.get()
            r5 = 7
            java.lang.Object[] r5 = new java.lang.Object[r5]
            r6 = 40
            java.lang.Character r6 = java.lang.Character.valueOf(r6)
            r5[r16] = r6
            r6 = 91
            java.lang.Character r6 = java.lang.Character.valueOf(r6)
            r7 = 1
            r5[r7] = r6
            r6 = 61
            java.lang.Character r6 = java.lang.Character.valueOf(r6)
            r17 = r3
            r3 = 2
            r5[r3] = r6
            r6 = 3
            r18 = 44
            java.lang.Character r18 = java.lang.Character.valueOf(r18)
            r5[r6] = r18
            r6 = 4
            r18 = 58
            java.lang.Character r18 = java.lang.Character.valueOf(r18)
            r5[r6] = r18
            r6 = 5
            r18 = 59
            java.lang.Character r19 = java.lang.Character.valueOf(r18)
            r5[r6] = r19
            r6 = 6
            r19 = 123(0x7b, float:1.72E-43)
            java.lang.Character r19 = java.lang.Character.valueOf(r19)
            r5[r6] = r19
            boolean r4 = r4.match(r5)
            if (r4 != 0) goto L_0x00a9
            r21 = r0
            r31 = r15
            goto L_0x0744
        L_0x00a9:
            org.bytedeco.javacpp.tools.Type r4 = r0.type
            boolean r4 = r4.staticMember
            if (r4 != 0) goto L_0x00cc
            java.lang.String r4 = r9.javaName
            if (r4 == 0) goto L_0x00cc
            java.lang.String r13 = "public native "
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r9.javaName
            java.lang.String r5 = r9.shorten(r5)
            r4.append(r5)
            java.lang.String r5 = " "
            r4.append(r5)
            java.lang.String r14 = r4.toString()
        L_0x00cc:
            java.lang.String r4 = "::"
            int r4 = r2.lastIndexOf(r4)
            java.lang.String r5 = r9.namespace
            if (r5 == 0) goto L_0x00ee
            if (r4 >= 0) goto L_0x00ee
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r9.namespace
            r5.append(r6)
            java.lang.String r6 = "::"
            r5.append(r6)
            r5.append(r2)
            java.lang.String r2 = r5.toString()
        L_0x00ee:
            org.bytedeco.javacpp.tools.InfoMap r5 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r5 = r5.getFirst(r2)
            java.lang.String r6 = r0.cppName
            int r6 = r6.length()
            if (r6 == 0) goto L_0x0710
            if (r5 == 0) goto L_0x0108
            boolean r6 = r5.skip
            if (r6 == 0) goto L_0x0108
            r21 = r0
            r31 = r15
            goto L_0x0714
        L_0x0108:
            if (r5 != 0) goto L_0x0139
            org.bytedeco.javacpp.tools.InfoMap r6 = r8.infoMap
            java.lang.String r3 = r0.cppName
            org.bytedeco.javacpp.tools.Info r3 = r6.getFirst(r3)
            org.bytedeco.javacpp.tools.InfoMap r6 = r8.infoMap
            if (r3 == 0) goto L_0x0128
            r21 = r0
            org.bytedeco.javacpp.tools.Info r0 = new org.bytedeco.javacpp.tools.Info
            r0.<init>((org.bytedeco.javacpp.tools.Info) r3)
            r22 = r3
            java.lang.String[] r3 = new java.lang.String[r7]
            r3[r16] = r2
            org.bytedeco.javacpp.tools.Info r0 = r0.cppNames(r3)
            goto L_0x0135
        L_0x0128:
            r21 = r0
            r22 = r3
            org.bytedeco.javacpp.tools.Info r0 = new org.bytedeco.javacpp.tools.Info
            java.lang.String[] r3 = new java.lang.String[r7]
            r3[r16] = r2
            r0.<init>((java.lang.String[]) r3)
        L_0x0135:
            r6.put(r0)
            goto L_0x013b
        L_0x0139:
            r21 = r0
        L_0x013b:
            r0 = 1
            org.bytedeco.javacpp.tools.Declarator r6 = r9.variable
            r19 = r0
            r22 = r2
            r24 = r5
            r23 = r17
            r0 = 0
            r17 = r4
        L_0x0149:
            r5 = r0
            r0 = 2147483647(0x7fffffff, float:NaN)
            r4 = 0
            if (r5 >= r0) goto L_0x0708
            org.bytedeco.javacpp.tools.Declaration r0 = new org.bytedeco.javacpp.tools.Declaration
            r0.<init>()
            r3 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            r2 = 0
            r25 = -1
            r26 = 0
            r27 = 0
            r28 = 1
            r0 = r43
            r1 = r44
            r29 = r3
            r3 = r25
            r4 = r26
            r20 = r5
            r30 = r6
            r6 = r27
            r31 = r15
            r15 = 1
            r7 = r28
            org.bytedeco.javacpp.tools.Declarator r7 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            if (r7 == 0) goto L_0x06ff
            java.lang.String r0 = r7.cppName
            if (r0 != 0) goto L_0x018c
            r21 = r7
            r2 = r29
            r26 = r30
            r4 = 0
            goto L_0x0706
        L_0x018c:
            r6 = r29
            r6.declarator = r7
            java.lang.String r0 = r7.cppName
            java.lang.String r1 = "::"
            int r1 = r0.lastIndexOf(r1)
            java.lang.String r2 = r9.namespace
            if (r2 == 0) goto L_0x01b4
            if (r1 >= 0) goto L_0x01b4
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = r9.namespace
            r2.append(r3)
            java.lang.String r3 = "::"
            r2.append(r3)
            r2.append(r0)
            java.lang.String r0 = r2.toString()
        L_0x01b4:
            r5 = r0
            org.bytedeco.javacpp.tools.InfoMap r0 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r4 = r0.getFirst(r5)
            java.lang.String r0 = "::"
            int r17 = r5.lastIndexOf(r0)
            r0 = r5
            if (r17 < 0) goto L_0x01ca
            int r1 = r17 + 2
            java.lang.String r0 = r5.substring(r1)
        L_0x01ca:
            r3 = r0
            java.lang.String r0 = r7.javaName
            r2 = r30
            if (r2 == 0) goto L_0x01e3
            int r1 = r2.indices
            if (r1 == 0) goto L_0x01e3
            int r1 = r7.indices
            if (r1 != 0) goto L_0x01da
            goto L_0x01e3
        L_0x01da:
            r15 = r0
            r35 = r2
            r33 = r4
            r34 = r5
            goto L_0x0458
        L_0x01e3:
            java.lang.String r1 = ""
            r15 = r1
            r1 = 0
        L_0x01e7:
            if (r2 == 0) goto L_0x01f3
            r32 = r0
            int r0 = r2.indices
            if (r0 != 0) goto L_0x01f0
            goto L_0x01f5
        L_0x01f0:
            int r0 = r2.indices
            goto L_0x01f7
        L_0x01f3:
            r32 = r0
        L_0x01f5:
            int r0 = r7.indices
        L_0x01f7:
            if (r1 >= r0) goto L_0x022f
            if (r1 <= 0) goto L_0x020f
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r15)
            r33 = r4
            java.lang.String r4 = ", "
            r0.append(r4)
            java.lang.String r15 = r0.toString()
            goto L_0x0211
        L_0x020f:
            r33 = r4
        L_0x0211:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r15)
            java.lang.String r4 = "int "
            r0.append(r4)
            int r4 = r1 + 105
            char r4 = (char) r4
            r0.append(r4)
            java.lang.String r15 = r0.toString()
            int r1 = r1 + 1
            r0 = r32
            r4 = r33
            goto L_0x01e7
        L_0x022f:
            r33 = r4
            java.lang.String r0 = r9.namespace
            if (r0 == 0) goto L_0x0258
            java.lang.String r0 = r9.javaName
            if (r0 != 0) goto L_0x0258
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r6.text
            r0.append(r1)
            java.lang.String r1 = "@Namespace(\""
            r0.append(r1)
            java.lang.String r1 = r9.namespace
            r0.append(r1)
            java.lang.String r1 = "\") "
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r6.text = r0
        L_0x0258:
            if (r2 == 0) goto L_0x02dd
            java.lang.String r0 = r2.cppName
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x02dd
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r6.text
            r0.append(r1)
            int r1 = r2.indices
            if (r1 != 0) goto L_0x0291
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = "@Name(\""
            r1.append(r4)
            java.lang.String r4 = r2.cppName
            r1.append(r4)
            java.lang.String r4 = "."
            r1.append(r4)
            r1.append(r3)
            java.lang.String r4 = "\") "
        L_0x0289:
            r1.append(r4)
            java.lang.String r1 = r1.toString()
            goto L_0x02ab
        L_0x0291:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = "@Name({\""
            r1.append(r4)
            java.lang.String r4 = r2.cppName
            r1.append(r4)
            java.lang.String r4 = "\", \"."
            r1.append(r4)
            r1.append(r3)
            java.lang.String r4 = "\"}) "
            goto L_0x0289
        L_0x02ab:
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r6.text = r0
            org.bytedeco.javacpp.tools.Type r0 = r7.type
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            java.lang.String r1 = r1.annotations
            java.lang.String r4 = "@Name\\(.*\\) "
            r34 = r5
            java.lang.String r5 = ""
            java.lang.String r1 = r1.replaceAll(r4, r5)
            r0.annotations = r1
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r2.javaName
            r0.append(r1)
            java.lang.String r1 = "_"
            r0.append(r1)
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            goto L_0x02e1
        L_0x02dd:
            r34 = r5
            r0 = r32
        L_0x02e1:
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            boolean r1 = r1.constValue
            if (r1 != 0) goto L_0x02eb
            boolean r1 = r7.constPointer
            if (r1 == 0) goto L_0x0300
        L_0x02eb:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = r6.text
            r1.append(r4)
            java.lang.String r4 = "@MemberGetter "
            r1.append(r4)
            java.lang.String r1 = r1.toString()
            r6.text = r1
        L_0x0300:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = r6.text
            r1.append(r4)
            r1.append(r13)
            org.bytedeco.javacpp.tools.Type r4 = r7.type
            java.lang.String r4 = r4.annotations
            java.lang.String r5 = "@ByVal "
            r35 = r2
            java.lang.String r2 = "@ByRef "
            java.lang.String r2 = r4.replace(r5, r2)
            r1.append(r2)
            org.bytedeco.javacpp.tools.Type r2 = r7.type
            java.lang.String r2 = r2.javaName
            r1.append(r2)
            java.lang.String r2 = " "
            r1.append(r2)
            r1.append(r0)
            java.lang.String r2 = "("
            r1.append(r2)
            r1.append(r15)
            java.lang.String r2 = ");"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r6.text = r1
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            boolean r1 = r1.constValue
            if (r1 != 0) goto L_0x03ab
            boolean r1 = r7.constPointer
            if (r1 != 0) goto L_0x03ab
            int r1 = r15.length()
            if (r1 <= 0) goto L_0x0361
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r15)
            java.lang.String r2 = ", "
            r1.append(r2)
            java.lang.String r15 = r1.toString()
        L_0x0361:
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            java.lang.String r1 = r1.javaName
            org.bytedeco.javacpp.tools.Type r2 = r7.type
            java.lang.String r2 = r2.javaName
            java.lang.String r4 = " "
            int r2 = r2.lastIndexOf(r4)
            r4 = 1
            int r2 = r2 + r4
            java.lang.String r1 = r1.substring(r2)
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r4 = r6.text
            r2.append(r4)
            java.lang.String r4 = " "
            r2.append(r4)
            r2.append(r13)
            r2.append(r14)
            r2.append(r0)
            java.lang.String r4 = "("
            r2.append(r4)
            r2.append(r15)
            r2.append(r1)
            java.lang.String r4 = " "
            r2.append(r4)
            r2.append(r0)
            java.lang.String r4 = ");"
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            r6.text = r2
        L_0x03ab:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = r6.text
            r1.append(r2)
            java.lang.String r2 = "\n"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r6.text = r1
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            boolean r1 = r1.constValue
            if (r1 != 0) goto L_0x03ca
            boolean r1 = r7.constPointer
            if (r1 == 0) goto L_0x0457
        L_0x03ca:
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            boolean r1 = r1.staticMember
            if (r1 == 0) goto L_0x0457
            int r1 = r15.length()
            if (r1 != 0) goto L_0x0457
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            java.lang.String r1 = r1.javaName
            org.bytedeco.javacpp.tools.Type r2 = r7.type
            java.lang.String r2 = r2.javaName
            r4 = 32
            int r2 = r2.lastIndexOf(r4)
            r4 = 1
            int r2 = r2 + r4
            java.lang.String r1 = r1.substring(r2)
            java.lang.String r2 = "byte"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "short"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "int"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "long"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "float"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "double"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "char"
            boolean r2 = r2.equals(r1)
            if (r2 != 0) goto L_0x042a
            java.lang.String r2 = "boolean"
            boolean r2 = r2.equals(r1)
            if (r2 == 0) goto L_0x0457
        L_0x042a:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r4 = r6.text
            r2.append(r4)
            java.lang.String r4 = "public static final "
            r2.append(r4)
            r2.append(r1)
            java.lang.String r4 = " "
            r2.append(r4)
            r2.append(r0)
            java.lang.String r4 = " = "
            r2.append(r4)
            r2.append(r0)
            java.lang.String r4 = "();\n"
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            r6.text = r2
        L_0x0457:
            r15 = r0
        L_0x0458:
            int r0 = r7.indices
            if (r0 <= 0) goto L_0x067f
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            r2 = 0
            r4 = -1
            r5 = 0
            r21 = 1
            r22 = 0
            r0 = r43
            r1 = r44
            r36 = r35
            r37 = r3
            r3 = r4
            r38 = r33
            r4 = r5
            r23 = r34
            r5 = r20
            r39 = r6
            r6 = r21
            r21 = r7
            r7 = r22
            org.bytedeco.javacpp.tools.Declarator r7 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            java.lang.String r0 = ""
            r6 = r0
            r0 = 0
        L_0x0487:
            r5 = r36
            if (r5 != 0) goto L_0x048d
            r1 = 0
            goto L_0x048f
        L_0x048d:
            int r1 = r5.indices
        L_0x048f:
            if (r0 >= r1) goto L_0x04c0
            if (r0 <= 0) goto L_0x04a4
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r6)
            java.lang.String r2 = ", "
            r1.append(r2)
            java.lang.String r6 = r1.toString()
        L_0x04a4:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r6)
            java.lang.String r2 = "int "
            r1.append(r2)
            int r2 = r0 + 105
            char r2 = (char) r2
            r1.append(r2)
            java.lang.String r6 = r1.toString()
            int r0 = r0 + 1
            r36 = r5
            goto L_0x0487
        L_0x04c0:
            java.lang.String r0 = r9.namespace
            if (r0 == 0) goto L_0x04ea
            java.lang.String r0 = r9.javaName
            if (r0 != 0) goto L_0x04ea
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r4 = r39
            java.lang.String r1 = r4.text
            r0.append(r1)
            java.lang.String r1 = "@Namespace(\""
            r0.append(r1)
            java.lang.String r1 = r9.namespace
            r0.append(r1)
            java.lang.String r1 = "\") "
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r4.text = r0
            goto L_0x04ec
        L_0x04ea:
            r4 = r39
        L_0x04ec:
            if (r5 == 0) goto L_0x0575
            java.lang.String r0 = r5.cppName
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x0575
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r4.text
            r0.append(r1)
            int r1 = r5.indices
            if (r1 != 0) goto L_0x0527
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "@Name(\""
            r1.append(r2)
            java.lang.String r2 = r5.cppName
            r1.append(r2)
            java.lang.String r2 = "."
            r1.append(r2)
            r3 = r37
            r1.append(r3)
            java.lang.String r2 = "\") "
        L_0x051f:
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            goto L_0x0543
        L_0x0527:
            r3 = r37
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "@Name({\""
            r1.append(r2)
            java.lang.String r2 = r5.cppName
            r1.append(r2)
            java.lang.String r2 = "\", \"."
            r1.append(r2)
            r1.append(r3)
            java.lang.String r2 = "\"}) "
            goto L_0x051f
        L_0x0543:
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r4.text = r0
            org.bytedeco.javacpp.tools.Type r0 = r7.type
            org.bytedeco.javacpp.tools.Type r1 = r7.type
            java.lang.String r1 = r1.annotations
            java.lang.String r2 = "@Name\\(.*\\) "
            r40 = r4
            java.lang.String r4 = ""
            java.lang.String r1 = r1.replaceAll(r2, r4)
            r0.annotations = r1
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = r5.javaName
            r0.append(r1)
            java.lang.String r1 = "_"
            r0.append(r1)
            r0.append(r3)
            java.lang.String r15 = r0.toString()
            goto L_0x0579
        L_0x0575:
            r40 = r4
            r3 = r37
        L_0x0579:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            r2 = 0
            r4 = -1
            r21 = 0
            r22 = 0
            r24 = 0
            r0 = r43
            r1 = r44
            r25 = r3
            r3 = r4
            r41 = r40
            r4 = r21
            r26 = r5
            r5 = r20
            r42 = r6
            r6 = r22
            r9 = r7
            r7 = r24
            org.bytedeco.javacpp.tools.Declarator r0 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            org.bytedeco.javacpp.tools.Type r1 = r0.type
            boolean r1 = r1.constValue
            if (r1 != 0) goto L_0x05b2
            boolean r1 = r0.constPointer
            if (r1 != 0) goto L_0x05b2
            int r1 = r0.indirections
            r3 = 2
            if (r1 >= r3) goto L_0x05af
            goto L_0x05b3
        L_0x05af:
            r2 = r41
            goto L_0x05ca
        L_0x05b2:
            r3 = 2
        L_0x05b3:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r2 = r41
            java.lang.String r4 = r2.text
            r1.append(r4)
            java.lang.String r4 = "@MemberGetter "
            r1.append(r4)
            java.lang.String r1 = r1.toString()
            r2.text = r1
        L_0x05ca:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = r2.text
            r1.append(r4)
            r1.append(r13)
            org.bytedeco.javacpp.tools.Type r4 = r9.type
            java.lang.String r4 = r4.annotations
            java.lang.String r5 = "@ByVal "
            java.lang.String r6 = "@ByRef "
            java.lang.String r4 = r4.replace(r5, r6)
            r1.append(r4)
            org.bytedeco.javacpp.tools.Type r4 = r9.type
            java.lang.String r4 = r4.javaName
            r1.append(r4)
            java.lang.String r4 = " "
            r1.append(r4)
            r1.append(r15)
            java.lang.String r4 = "("
            r1.append(r4)
            r6 = r42
            r1.append(r6)
            java.lang.String r4 = ");"
            r1.append(r4)
            java.lang.String r1 = r1.toString()
            r2.text = r1
            org.bytedeco.javacpp.tools.Type r1 = r9.type
            boolean r1 = r1.constValue
            if (r1 != 0) goto L_0x0669
            boolean r1 = r9.constPointer
            if (r1 != 0) goto L_0x0669
            int r1 = r0.indirections
            if (r1 < r3) goto L_0x0669
            int r1 = r6.length()
            if (r1 <= 0) goto L_0x062f
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r6)
            java.lang.String r4 = ", "
            r1.append(r4)
            java.lang.String r6 = r1.toString()
        L_0x062f:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = r2.text
            r1.append(r4)
            java.lang.String r4 = " "
            r1.append(r4)
            r1.append(r13)
            r1.append(r14)
            r1.append(r15)
            java.lang.String r4 = "("
            r1.append(r4)
            r1.append(r6)
            org.bytedeco.javacpp.tools.Type r4 = r9.type
            java.lang.String r4 = r4.javaName
            r1.append(r4)
            java.lang.String r4 = " "
            r1.append(r4)
            r1.append(r15)
            java.lang.String r4 = ");"
            r1.append(r4)
            java.lang.String r1 = r1.toString()
            r2.text = r1
        L_0x0669:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r4 = r2.text
            r1.append(r4)
            java.lang.String r4 = "\n"
            r1.append(r4)
            java.lang.String r1 = r1.toString()
            r2.text = r1
            goto L_0x068d
        L_0x067f:
            r25 = r3
            r2 = r6
            r21 = r7
            r38 = r33
            r23 = r34
            r26 = r35
            r3 = 2
            r9 = r21
        L_0x068d:
            java.lang.String r0 = r9.signature
            r2.signature = r0
            r0 = r38
            if (r0 == 0) goto L_0x06a2
            java.lang.String r1 = r0.javaText
            if (r1 == 0) goto L_0x06a2
            java.lang.String r1 = r0.javaText
            r2.text = r1
            r2.signature = r1
            r4 = 0
            r2.declarator = r4
        L_0x06a2:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            org.bytedeco.javacpp.tools.Token r1 = r1.get()
            java.lang.Object[] r4 = new java.lang.Object[r3]
            org.bytedeco.javacpp.tools.Token r5 = org.bytedeco.javacpp.tools.Token.EOF
            r4[r16] = r5
            java.lang.Character r5 = java.lang.Character.valueOf(r18)
            r6 = 1
            r4[r6] = r5
            boolean r1 = r1.match(r4)
            if (r1 != 0) goto L_0x06c1
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            r1.next()
            goto L_0x06a2
        L_0x06c1:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            r1.next()
            java.lang.String r1 = r43.commentAfter()
            if (r19 == 0) goto L_0x06e4
            r4 = 0
            r10.spacing = r12
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r1)
            java.lang.String r6 = r2.text
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r2.text = r5
            r19 = r4
        L_0x06e4:
            r4 = 1
            r2.variable = r4
            r10.add((org.bytedeco.javacpp.tools.Declaration) r2)
            int r1 = r20 + 1
            r24 = r0
            r0 = r1
            r1 = r2
            r21 = r9
            r22 = r23
            r6 = r26
            r7 = 1
            r9 = r44
            r23 = r15
            r15 = r31
            goto L_0x0149
        L_0x06ff:
            r21 = r7
            r2 = r29
            r26 = r30
            r4 = 0
        L_0x0706:
            r1 = r2
            goto L_0x070c
        L_0x0708:
            r26 = r6
            r31 = r15
        L_0x070c:
            r10.spacing = r4
            r0 = 1
            return r0
        L_0x0710:
            r21 = r0
            r31 = r15
        L_0x0714:
            r1.text = r12
            r10.add((org.bytedeco.javacpp.tools.Declaration) r1)
        L_0x0719:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r6 = new java.lang.Object[r3]
            org.bytedeco.javacpp.tools.Token r7 = org.bytedeco.javacpp.tools.Token.EOF
            r6[r16] = r7
            java.lang.Character r7 = java.lang.Character.valueOf(r18)
            r9 = 1
            r6[r9] = r7
            boolean r0 = r0.match(r6)
            if (r0 != 0) goto L_0x0738
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x0719
        L_0x0738:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            return r9
        L_0x073e:
            r21 = r0
            r17 = r3
            r31 = r15
        L_0x0744:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            return r16
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.variable(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):boolean");
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:190:0x05af, code lost:
        if (r0.match("L") != false) goto L_0x05b2;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:34:0x0101, code lost:
        r32 = r9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:46:0x011c, code lost:
        r21 = r10;
        r10 = new java.lang.String[r4];
        r10[0] = r0;
        r7 = new org.bytedeco.javacpp.tools.Info(r10).cppText("");
        r1.tokens.index = r3;
        r10 = r1.tokens.get();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:48:0x013d, code lost:
        if (r1.tokens.index >= r9) goto L_0x017f;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:49:0x013f, code lost:
        r14 = new java.lang.StringBuilder();
        r22 = r3;
        r14.append(r7.cppText);
        r3 = new java.lang.Object[r4];
        r3[0] = "\n";
     */
    /* JADX WARNING: Code restructure failed: missing block: B:50:0x0157, code lost:
        if (r10.match(r3) == false) goto L_0x015b;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:51:0x0159, code lost:
        r3 = r10;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:52:0x015b, code lost:
        r3 = r10.spacing + r10;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:53:0x016c, code lost:
        r14.append(r3);
        r7.cppText = r14.toString();
        r10 = r1.tokens.next();
        r3 = r22;
        r4 = 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:54:0x017f, code lost:
        r22 = r3;
        r1.infoMap.put(r7);
     */
    /* JADX WARNING: Removed duplicated region for block: B:24:0x00d6  */
    /* JADX WARNING: Removed duplicated region for block: B:25:0x00da  */
    /* JADX WARNING: Removed duplicated region for block: B:264:0x07a5 A[LOOP:2: B:27:0x00ea->B:264:0x07a5, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:268:0x07c3  */
    /* JADX WARNING: Removed duplicated region for block: B:279:0x0859  */
    /* JADX WARNING: Removed duplicated region for block: B:285:0x07b7 A[EDGE_INSN: B:285:0x07b7->B:265:0x07b7 ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:290:0x079e A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:29:0x00f0  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean macro(org.bytedeco.javacpp.tools.Context r47, org.bytedeco.javacpp.tools.DeclarationList r48) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r46 = this;
            r1 = r46
            r2 = r48
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r3 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r4 = 1
            java.lang.Object[] r5 = new java.lang.Object[r4]
            r6 = 35
            java.lang.Character r6 = java.lang.Character.valueOf(r6)
            r7 = 0
            r5[r7] = r6
            boolean r0 = r0.match(r5)
            if (r0 != 0) goto L_0x0021
            return r7
        L_0x0021:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.raw = r4
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r5 = r0.spacing
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r6 = r0.next()
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r8 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
        L_0x0042:
            java.lang.Object[] r9 = new java.lang.Object[r4]
            org.bytedeco.javacpp.tools.Token r10 = org.bytedeco.javacpp.tools.Token.EOF
            r9[r7] = r10
            boolean r9 = r0.match(r9)
            r10 = 10
            if (r9 != 0) goto L_0x0060
            java.lang.String r9 = r0.spacing
            int r9 = r9.indexOf(r10)
            if (r9 < 0) goto L_0x0059
            goto L_0x0060
        L_0x0059:
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.next()
            goto L_0x0042
        L_0x0060:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r0 = r0.index
        L_0x0064:
            r9 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r11 = -1
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r11)
            java.lang.Object[] r12 = new java.lang.Object[r4]
            r13 = 4
            java.lang.Integer r13 = java.lang.Integer.valueOf(r13)
            r12[r7] = r13
            boolean r0 = r0.match(r12)
            if (r0 == 0) goto L_0x0084
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r11 = r0.index
            int r11 = r11 - r4
            r0.index = r11
            r0 = r9
            goto L_0x0064
        L_0x0084:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r12 = r0.index
            org.bytedeco.javacpp.tools.Declaration r0 = new org.bytedeco.javacpp.tools.Declaration
            r0.<init>()
            r13 = r0
            java.lang.Object[] r0 = new java.lang.Object[r4]
            org.bytedeco.javacpp.tools.Token r14 = org.bytedeco.javacpp.tools.Token.DEFINE
            r0[r7] = r14
            boolean r0 = r6.match(r0)
            r14 = 0
            if (r0 == 0) goto L_0x07b7
            if (r8 >= r9) goto L_0x07b7
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.index = r8
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r0 = r0.value
            org.bytedeco.javacpp.tools.TokenIndexer r15 = r1.tokens
            org.bytedeco.javacpp.tools.Token r15 = r15.next()
            java.lang.String r11 = r15.spacing
            int r11 = r11.length()
            r17 = 40
            if (r11 != 0) goto L_0x00c9
            java.lang.Object[] r11 = new java.lang.Object[r4]
            java.lang.Character r18 = java.lang.Character.valueOf(r17)
            r11[r7] = r18
            boolean r11 = r15.match(r11)
            if (r11 == 0) goto L_0x00c9
            r11 = 1
            goto L_0x00ca
        L_0x00c9:
            r11 = 0
        L_0x00ca:
            org.bytedeco.javacpp.tools.InfoMap r10 = r1.infoMap
            java.util.List r10 = r10.get(r0)
            int r18 = r10.size()
            if (r18 <= 0) goto L_0x00da
            r19 = r0
            r0 = r10
            goto L_0x00e4
        L_0x00da:
            r19 = r0
            org.bytedeco.javacpp.tools.Info[] r0 = new org.bytedeco.javacpp.tools.Info[r4]
            r0[r7] = r14
            java.util.List r0 = java.util.Arrays.asList(r0)
        L_0x00e4:
            java.util.Iterator r18 = r0.iterator()
            r0 = r19
        L_0x00ea:
            boolean r19 = r18.hasNext()
            if (r19 == 0) goto L_0x07b7
            java.lang.Object r19 = r18.next()
            r14 = r19
            org.bytedeco.javacpp.tools.Info r14 = (org.bytedeco.javacpp.tools.Info) r14
            if (r14 == 0) goto L_0x0105
            boolean r7 = r14.skip
            if (r7 == 0) goto L_0x0105
            r22 = r3
        L_0x0101:
            r32 = r9
            goto L_0x07bb
        L_0x0105:
            if (r14 != 0) goto L_0x010d
            if (r11 != 0) goto L_0x011c
            int r7 = r8 + 1
            if (r7 == r9) goto L_0x011c
        L_0x010d:
            if (r14 == 0) goto L_0x0188
            java.lang.String r7 = r14.cppText
            if (r7 != 0) goto L_0x0188
            java.lang.String[] r7 = r14.cppTypes
            if (r7 == 0) goto L_0x0188
            java.lang.String[] r7 = r14.cppTypes
            int r7 = r7.length
            if (r7 != 0) goto L_0x0188
        L_0x011c:
            org.bytedeco.javacpp.tools.Info r7 = new org.bytedeco.javacpp.tools.Info
            r21 = r10
            java.lang.String[] r10 = new java.lang.String[r4]
            r16 = 0
            r10[r16] = r0
            r7.<init>((java.lang.String[]) r10)
            java.lang.String r10 = ""
            org.bytedeco.javacpp.tools.Info r7 = r7.cppText(r10)
            org.bytedeco.javacpp.tools.TokenIndexer r10 = r1.tokens
            r10.index = r3
            org.bytedeco.javacpp.tools.TokenIndexer r10 = r1.tokens
            org.bytedeco.javacpp.tools.Token r10 = r10.get()
        L_0x0139:
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r1.tokens
            int r14 = r14.index
            if (r14 >= r9) goto L_0x017f
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r22 = r3
            java.lang.String r3 = r7.cppText
            r14.append(r3)
            java.lang.Object[] r3 = new java.lang.Object[r4]
            java.lang.String r16 = "\n"
            r17 = 0
            r3[r17] = r16
            boolean r3 = r10.match(r3)
            if (r3 == 0) goto L_0x015b
            r3 = r10
            goto L_0x016c
        L_0x015b:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r10.spacing
            r3.append(r4)
            r3.append(r10)
            java.lang.String r3 = r3.toString()
        L_0x016c:
            r14.append(r3)
            java.lang.String r3 = r14.toString()
            r7.cppText = r3
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            org.bytedeco.javacpp.tools.Token r10 = r3.next()
            r3 = r22
            r4 = 1
            goto L_0x0139
        L_0x017f:
            r22 = r3
            org.bytedeco.javacpp.tools.InfoMap r3 = r1.infoMap
            r3.put(r7)
            goto L_0x0101
        L_0x0188:
            r22 = r3
            r21 = r10
            if (r14 == 0) goto L_0x03bb
            java.lang.String r4 = r14.cppText
            if (r4 != 0) goto L_0x03bb
            java.lang.String[] r4 = r14.cppTypes
            if (r4 == 0) goto L_0x03bb
            java.lang.String[] r4 = r14.cppTypes
            int r4 = r4.length
            if (r11 == 0) goto L_0x019d
            r7 = 0
            goto L_0x019e
        L_0x019d:
            r7 = 1
        L_0x019e:
            if (r4 <= r7) goto L_0x03bb
            java.util.ArrayList r4 = new java.util.ArrayList
            r4.<init>()
            r7 = r0
            r0 = -1
        L_0x01a7:
            r10 = 2147483647(0x7fffffff, float:NaN)
            if (r0 >= r10) goto L_0x03b2
            r10 = 1
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            r32 = r9
            int r9 = r8 + 2
            r3.index = r9
            java.lang.String r3 = "("
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r9.get()
        L_0x01bd:
            if (r11 == 0) goto L_0x0265
            r33 = r11
            org.bytedeco.javacpp.tools.TokenIndexer r11 = r1.tokens
            int r11 = r11.index
            if (r11 >= r12) goto L_0x0260
            java.lang.String[] r11 = r14.cppTypes
            int r11 = r11.length
            if (r10 >= r11) goto L_0x0260
            r34 = r15
            r11 = 1
            java.lang.Object[] r15 = new java.lang.Object[r11]
            r11 = 5
            java.lang.Integer r19 = java.lang.Integer.valueOf(r11)
            r11 = 0
            r15[r11] = r19
            boolean r11 = r9.match(r15)
            if (r11 == 0) goto L_0x023b
            java.lang.String[] r11 = r14.cppTypes
            r11 = r11[r10]
            java.lang.String r15 = r9.value
            java.lang.String r2 = "..."
            boolean r2 = r15.equals(r2)
            if (r2 == 0) goto L_0x0201
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r35 = r15
            java.lang.String r15 = "arg"
            r2.append(r15)
            r2.append(r10)
            java.lang.String r15 = r2.toString()
            goto L_0x0203
        L_0x0201:
            r35 = r15
        L_0x0203:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r2.append(r3)
            r2.append(r11)
            r36 = r3
            java.lang.String r3 = " "
            r2.append(r3)
            r2.append(r15)
            java.lang.String r2 = r2.toString()
            int r10 = r10 + 1
            java.lang.String[] r3 = r14.cppTypes
            int r3 = r3.length
            if (r10 >= r3) goto L_0x0237
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r2)
            r37 = r2
            java.lang.String r2 = ", "
            r3.append(r2)
            java.lang.String r2 = r3.toString()
            goto L_0x0239
        L_0x0237:
            r37 = r2
        L_0x0239:
            r3 = r2
            goto L_0x0252
        L_0x023b:
            r36 = r3
            r2 = 1
            java.lang.Object[] r3 = new java.lang.Object[r2]
            r2 = 41
            java.lang.Character r2 = java.lang.Character.valueOf(r2)
            r11 = 0
            r3[r11] = r2
            boolean r2 = r9.match(r3)
            if (r2 == 0) goto L_0x0250
            goto L_0x026b
        L_0x0250:
            r3 = r36
        L_0x0252:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r2.next()
            r11 = r33
            r15 = r34
            r2 = r48
            goto L_0x01bd
        L_0x0260:
            r36 = r3
            r34 = r15
            goto L_0x026b
        L_0x0265:
            r36 = r3
            r33 = r11
            r34 = r15
        L_0x026b:
            r2 = r36
        L_0x026d:
            java.lang.String[] r3 = r14.cppTypes
            int r3 = r3.length
            if (r10 >= r3) goto L_0x02b7
            java.lang.String[] r3 = r14.cppTypes
            r3 = r3[r10]
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String r11 = "arg"
            r9.append(r11)
            r9.append(r10)
            java.lang.String r9 = r9.toString()
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r2)
            r11.append(r3)
            java.lang.String r15 = " "
            r11.append(r15)
            r11.append(r9)
            java.lang.String r2 = r11.toString()
            int r10 = r10 + 1
            java.lang.String[] r11 = r14.cppTypes
            int r11 = r11.length
            if (r10 >= r11) goto L_0x02b6
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r2)
            java.lang.String r15 = ", "
            r11.append(r15)
            java.lang.String r2 = r11.toString()
        L_0x02b6:
            goto L_0x026d
        L_0x02b7:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r3.append(r2)
            java.lang.String r9 = ")"
            r3.append(r9)
            java.lang.String r2 = r3.toString()
            org.bytedeco.javacpp.tools.Parser r3 = new org.bytedeco.javacpp.tools.Parser
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            java.lang.String[] r11 = r14.cppTypes
            r15 = 0
            r11 = r11[r15]
            r9.append(r11)
            java.lang.String r11 = " "
            r9.append(r11)
            r9.append(r7)
            r9.append(r2)
            java.lang.String r9 = r9.toString()
            r3.<init>((org.bytedeco.javacpp.tools.Parser) r1, (java.lang.String) r9)
            r26 = 0
            r28 = 0
            r29 = 0
            r30 = 0
            r31 = 0
            r24 = r3
            r25 = r47
            r27 = r0
            org.bytedeco.javacpp.tools.Declarator r3 = r24.declarator(r25, r26, r27, r28, r29, r30, r31)
            r9 = 0
        L_0x02fe:
            java.lang.String[] r11 = r14.cppNames
            int r11 = r11.length
            if (r9 >= r11) goto L_0x0338
            java.lang.String[] r11 = r14.cppNames
            r11 = r11[r9]
            boolean r11 = r7.equals(r11)
            if (r11 == 0) goto L_0x0335
            java.lang.String[] r11 = r14.javaNames
            if (r11 == 0) goto L_0x0335
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r15 = "@Name(\""
            r11.append(r15)
            java.lang.String[] r15 = r14.cppNames
            r19 = 0
            r15 = r15[r19]
            r11.append(r15)
            java.lang.String r15 = "\") "
            r11.append(r15)
            java.lang.String[] r15 = r14.javaNames
            r15 = r15[r9]
            r11.append(r15)
            java.lang.String r7 = r11.toString()
            goto L_0x0338
        L_0x0335:
            int r9 = r9 + 1
            goto L_0x02fe
        L_0x0338:
            r9 = 0
            java.util.Iterator r11 = r4.iterator()
        L_0x033d:
            boolean r15 = r11.hasNext()
            if (r15 == 0) goto L_0x035c
            java.lang.Object r15 = r11.next()
            org.bytedeco.javacpp.tools.Declarator r15 = (org.bytedeco.javacpp.tools.Declarator) r15
            r38 = r2
            java.lang.String r2 = r3.signature
            r39 = r10
            java.lang.String r10 = r15.signature
            boolean r2 = r2.equals(r10)
            r9 = r9 | r2
            r2 = r38
            r10 = r39
            goto L_0x033d
        L_0x035c:
            r38 = r2
            r39 = r10
            if (r9 != 0) goto L_0x039e
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r10 = r13.text
            r2.append(r10)
            java.lang.String r10 = "public static native "
            r2.append(r10)
            org.bytedeco.javacpp.tools.Type r10 = r3.type
            java.lang.String r10 = r10.annotations
            r2.append(r10)
            org.bytedeco.javacpp.tools.Type r10 = r3.type
            java.lang.String r10 = r10.javaName
            r2.append(r10)
            java.lang.String r10 = " "
            r2.append(r10)
            r2.append(r7)
            org.bytedeco.javacpp.tools.Parameters r10 = r3.parameters
            java.lang.String r10 = r10.list
            r2.append(r10)
            java.lang.String r10 = ";\n"
            r2.append(r10)
            java.lang.String r2 = r2.toString()
            r13.text = r2
            java.lang.String r2 = r3.signature
            r13.signature = r2
            goto L_0x03a3
        L_0x039e:
            if (r9 == 0) goto L_0x03a3
            if (r0 <= 0) goto L_0x03a3
            goto L_0x03b8
        L_0x03a3:
            r4.add(r3)
            int r0 = r0 + 1
            r9 = r32
            r11 = r33
            r15 = r34
            r2 = r48
            goto L_0x01a7
        L_0x03b2:
            r32 = r9
            r33 = r11
            r34 = r15
        L_0x03b8:
            r0 = r7
            goto L_0x0798
        L_0x03bb:
            r32 = r9
            r33 = r11
            r34 = r15
            int r2 = r8 + 1
            if (r12 <= r2) goto L_0x0798
            if (r14 == 0) goto L_0x03d5
            java.lang.String r2 = r14.cppText
            if (r2 != 0) goto L_0x0798
            java.lang.String[] r2 = r14.cppTypes
            if (r2 == 0) goto L_0x03d5
            java.lang.String[] r2 = r14.cppTypes
            int r2 = r2.length
            r3 = 1
            if (r2 != r3) goto L_0x0798
        L_0x03d5:
            java.lang.String r2 = ""
            java.lang.String r3 = "int"
            java.lang.String r4 = "int"
            java.lang.String r7 = ""
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            int r10 = r8 + 1
            r9.index = r10
            org.bytedeco.javacpp.tools.Token r9 = new org.bytedeco.javacpp.tools.Token
            r9.<init>()
            r10 = 1
            org.bytedeco.javacpp.tools.TokenIndexer r11 = r1.tokens
            org.bytedeco.javacpp.tools.Token r11 = r11.get()
        L_0x03ef:
            org.bytedeco.javacpp.tools.TokenIndexer r15 = r1.tokens
            int r15 = r15.index
            if (r15 >= r12) goto L_0x04d3
            r40 = r2
            r15 = 1
            java.lang.Object[] r2 = new java.lang.Object[r15]
            r15 = 3
            java.lang.Integer r15 = java.lang.Integer.valueOf(r15)
            r19 = 0
            r2[r19] = r15
            boolean r2 = r11.match(r2)
            if (r2 == 0) goto L_0x0411
            java.lang.String r3 = "const char*"
            java.lang.String r4 = "String"
            java.lang.String r7 = " + "
            goto L_0x04d7
        L_0x0411:
            r2 = 1
            java.lang.Object[] r15 = new java.lang.Object[r2]
            r2 = 2
            java.lang.Integer r19 = java.lang.Integer.valueOf(r2)
            r20 = 0
            r15[r20] = r19
            boolean r15 = r11.match(r15)
            if (r15 == 0) goto L_0x042b
            java.lang.String r3 = "double"
            java.lang.String r4 = "double"
            java.lang.String r7 = ""
            goto L_0x04d7
        L_0x042b:
            r15 = 1
            java.lang.Object[] r2 = new java.lang.Object[r15]
            java.lang.Integer r19 = java.lang.Integer.valueOf(r15)
            r2[r20] = r19
            boolean r2 = r11.match(r2)
            if (r2 == 0) goto L_0x044c
            java.lang.String r2 = r11.value
            java.lang.String r15 = "L"
            boolean r2 = r2.endsWith(r15)
            if (r2 == 0) goto L_0x044c
            java.lang.String r3 = "long long"
            java.lang.String r4 = "long"
            java.lang.String r7 = ""
            goto L_0x04d7
        L_0x044c:
            r2 = 2
            java.lang.Object[] r15 = new java.lang.Object[r2]
            r19 = 5
            java.lang.Integer r24 = java.lang.Integer.valueOf(r19)
            r20 = 0
            r15[r20] = r24
            r24 = 62
            java.lang.Character r24 = java.lang.Character.valueOf(r24)
            r23 = 1
            r15[r23] = r24
            boolean r15 = r9.match(r15)
            if (r15 == 0) goto L_0x0482
            java.lang.Object[] r15 = new java.lang.Object[r2]
            java.lang.Integer r24 = java.lang.Integer.valueOf(r19)
            r15[r20] = r24
            java.lang.Character r19 = java.lang.Character.valueOf(r17)
            r15[r23] = r19
            boolean r15 = r11.match(r15)
            if (r15 != 0) goto L_0x047e
            goto L_0x0482
        L_0x047e:
            r41 = r3
            r3 = 1
            goto L_0x049f
        L_0x0482:
            java.lang.Object[] r2 = new java.lang.Object[r2]
            r15 = 123(0x7b, float:1.72E-43)
            java.lang.Character r15 = java.lang.Character.valueOf(r15)
            r19 = 0
            r2[r19] = r15
            r15 = 125(0x7d, float:1.75E-43)
            java.lang.Character r15 = java.lang.Character.valueOf(r15)
            r41 = r3
            r3 = 1
            r2[r3] = r15
            boolean r2 = r11.match(r2)
            if (r2 == 0) goto L_0x04a3
        L_0x049f:
            r2 = 0
            r10 = r2
            r3 = 5
            goto L_0x04c6
        L_0x04a3:
            java.lang.Object[] r2 = new java.lang.Object[r3]
            r3 = 5
            java.lang.Integer r15 = java.lang.Integer.valueOf(r3)
            r19 = 0
            r2[r19] = r15
            boolean r2 = r11.match(r2)
            if (r2 == 0) goto L_0x04c6
            org.bytedeco.javacpp.tools.InfoMap r2 = r1.infoMap
            java.lang.String r15 = r11.value
            org.bytedeco.javacpp.tools.Info r2 = r2.getFirst(r15)
            if (r14 != 0) goto L_0x04c6
            if (r2 == 0) goto L_0x04c6
            java.lang.String[] r15 = r2.cppTypes
            if (r15 == 0) goto L_0x04c6
            r14 = r2
        L_0x04c6:
            r9 = r11
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r1.tokens
            org.bytedeco.javacpp.tools.Token r11 = r2.next()
            r2 = r40
            r3 = r41
            goto L_0x03ef
        L_0x04d3:
            r40 = r2
            r41 = r3
        L_0x04d7:
            if (r14 == 0) goto L_0x0570
            java.lang.String[] r2 = r14.cppTypes
            if (r2 == 0) goto L_0x0533
            java.lang.String[] r2 = r14.cppTypes
            int r2 = r2.length
            if (r2 <= 0) goto L_0x0533
            org.bytedeco.javacpp.tools.Parser r2 = new org.bytedeco.javacpp.tools.Parser
            java.lang.String[] r11 = r14.cppTypes
            r15 = 0
            r11 = r11[r15]
            r2.<init>((org.bytedeco.javacpp.tools.Parser) r1, (java.lang.String) r11)
            r26 = 0
            r27 = -1
            r28 = 0
            r29 = 0
            r30 = 0
            r31 = 1
            r24 = r2
            r25 = r47
            org.bytedeco.javacpp.tools.Declarator r2 = r24.declarator(r25, r26, r27, r28, r29, r30, r31)
            org.bytedeco.javacpp.tools.Type r11 = r2.type
            java.lang.String r11 = r11.javaName
            java.lang.String r15 = "int"
            boolean r11 = r11.equals(r15)
            if (r11 != 0) goto L_0x0533
            org.bytedeco.javacpp.tools.Type r11 = r2.type
            java.lang.String r3 = r11.cppName
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            org.bytedeco.javacpp.tools.Type r15 = r2.type
            java.lang.String r15 = r15.annotations
            r11.append(r15)
            java.lang.String[] r15 = r14.pointerTypes
            if (r15 == 0) goto L_0x0527
            java.lang.String[] r15 = r14.pointerTypes
            r19 = 0
            r15 = r15[r19]
            goto L_0x052b
        L_0x0527:
            org.bytedeco.javacpp.tools.Type r15 = r2.type
            java.lang.String r15 = r15.javaName
        L_0x052b:
            r11.append(r15)
            java.lang.String r2 = r11.toString()
            r4 = r2
        L_0x0533:
            r2 = 0
        L_0x0534:
            java.lang.String[] r11 = r14.cppNames
            int r11 = r11.length
            if (r2 >= r11) goto L_0x056e
            java.lang.String[] r11 = r14.cppNames
            r11 = r11[r2]
            boolean r11 = r0.equals(r11)
            if (r11 == 0) goto L_0x056b
            java.lang.String[] r11 = r14.javaNames
            if (r11 == 0) goto L_0x056b
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r15 = "@Name(\""
            r11.append(r15)
            java.lang.String[] r15 = r14.cppNames
            r19 = 0
            r15 = r15[r19]
            r11.append(r15)
            java.lang.String r15 = "\") "
            r11.append(r15)
            java.lang.String[] r15 = r14.javaNames
            r15 = r15[r2]
            r11.append(r15)
            java.lang.String r0 = r11.toString()
            goto L_0x056e
        L_0x056b:
            int r2 = r2 + 1
            goto L_0x0534
        L_0x056e:
            boolean r10 = r14.translate
        L_0x0570:
            r2 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r11 = r8 + 1
            r0.index = r11
            if (r10 == 0) goto L_0x06af
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r15 = r40
        L_0x0581:
            org.bytedeco.javacpp.tools.TokenIndexer r11 = r1.tokens
            int r11 = r11.index
            if (r11 >= r12) goto L_0x05ea
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r15)
            r42 = r3
            java.lang.String r3 = r0.spacing
            r11.append(r3)
            java.lang.String r3 = r11.toString()
            java.lang.String r11 = "String"
            boolean r11 = r4.equals(r11)
            if (r11 == 0) goto L_0x05b4
            r11 = 1
            java.lang.Object[] r15 = new java.lang.Object[r11]
            java.lang.String r11 = "L"
            r19 = 0
            r15[r19] = r11
            boolean r11 = r0.match(r15)
            if (r11 == 0) goto L_0x05b4
        L_0x05b2:
            r15 = r3
            goto L_0x05e1
        L_0x05b4:
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r11.append(r3)
            r11.append(r0)
            org.bytedeco.javacpp.tools.TokenIndexer r15 = r1.tokens
            int r15 = r15.index
            r19 = 1
            int r15 = r15 + 1
            if (r15 >= r12) goto L_0x05d7
            java.lang.String r15 = r0.value
            java.lang.String r15 = r15.trim()
            int r15 = r15.length()
            if (r15 <= 0) goto L_0x05d7
            r15 = r7
            goto L_0x05d9
        L_0x05d7:
            java.lang.String r15 = ""
        L_0x05d9:
            r11.append(r15)
            java.lang.String r3 = r11.toString()
            goto L_0x05b2
        L_0x05e1:
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r3.next()
            r3 = r42
            goto L_0x0581
        L_0x05ea:
            r42 = r3
            java.lang.String r3 = r1.translate(r15)
            java.lang.String r0 = "int"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x06ac
            java.lang.String r0 = "(String)"
            boolean r0 = r3.contains(r0)
            if (r0 == 0) goto L_0x060a
            java.lang.String r0 = "const char*"
            java.lang.String r4 = "String"
        L_0x0604:
            r42 = r0
            r43 = r7
            goto L_0x071b
        L_0x060a:
            java.lang.String r0 = "(float)"
            boolean r0 = r3.contains(r0)
            if (r0 != 0) goto L_0x06a3
            java.lang.String r0 = "(double)"
            boolean r0 = r3.contains(r0)
            if (r0 == 0) goto L_0x061e
            r43 = r7
            goto L_0x06a5
        L_0x061e:
            java.lang.String r0 = "(long)"
            boolean r0 = r3.contains(r0)
            if (r0 == 0) goto L_0x062b
            java.lang.String r0 = "long long"
            java.lang.String r4 = "long"
            goto L_0x0604
        L_0x062b:
            java.lang.String r0 = r3.trim()     // Catch:{ NumberFormatException -> 0x069e }
            long r24 = java.lang.Long.parseLong(r0)     // Catch:{ NumberFormatException -> 0x069e }
            r26 = 2147483647(0x7fffffff, double:1.060997895E-314)
            int r11 = (r24 > r26 ? 1 : (r24 == r26 ? 0 : -1))
            if (r11 <= 0) goto L_0x0670
            r11 = 32
            long r28 = r24 >>> r11
            r30 = 0
            int r11 = (r28 > r30 ? 1 : (r28 == r30 ? 0 : -1))
            if (r11 != 0) goto L_0x0670
            java.lang.StringBuilder r11 = new java.lang.StringBuilder     // Catch:{ NumberFormatException -> 0x069e }
            r11.<init>()     // Catch:{ NumberFormatException -> 0x069e }
            int r15 = r3.length()     // Catch:{ NumberFormatException -> 0x069e }
            int r19 = r0.length()     // Catch:{ NumberFormatException -> 0x069e }
            int r15 = r15 - r19
            r43 = r7
            r7 = 0
            java.lang.String r15 = r3.substring(r7, r15)     // Catch:{ NumberFormatException -> 0x069c }
            r11.append(r15)     // Catch:{ NumberFormatException -> 0x069c }
            java.lang.String r7 = "(int)"
            r11.append(r7)     // Catch:{ NumberFormatException -> 0x069c }
            r11.append(r0)     // Catch:{ NumberFormatException -> 0x069c }
            java.lang.String r7 = "L"
            r11.append(r7)     // Catch:{ NumberFormatException -> 0x069c }
            java.lang.String r7 = r11.toString()     // Catch:{ NumberFormatException -> 0x069c }
            r3 = r7
            goto L_0x06a1
        L_0x0670:
            r43 = r7
            int r7 = (r24 > r26 ? 1 : (r24 == r26 ? 0 : -1))
            if (r7 > 0) goto L_0x067d
            r26 = -2147483648(0xffffffff80000000, double:NaN)
            int r7 = (r24 > r26 ? 1 : (r24 == r26 ? 0 : -1))
            if (r7 >= 0) goto L_0x06a1
        L_0x067d:
            java.lang.String r7 = "long long"
            java.lang.String r11 = "long"
            r4 = r11
            java.lang.StringBuilder r11 = new java.lang.StringBuilder     // Catch:{ NumberFormatException -> 0x0698 }
            r11.<init>()     // Catch:{ NumberFormatException -> 0x0698 }
            r11.append(r3)     // Catch:{ NumberFormatException -> 0x0698 }
            java.lang.String r15 = "L"
            r11.append(r15)     // Catch:{ NumberFormatException -> 0x0698 }
            java.lang.String r11 = r11.toString()     // Catch:{ NumberFormatException -> 0x0698 }
            r0 = r11
            r3 = r0
            r42 = r7
            goto L_0x06a1
        L_0x0698:
            r0 = move-exception
            r42 = r7
            goto L_0x06a1
        L_0x069c:
            r0 = move-exception
            goto L_0x06a1
        L_0x069e:
            r0 = move-exception
            r43 = r7
        L_0x06a1:
            goto L_0x071b
        L_0x06a3:
            r43 = r7
        L_0x06a5:
            java.lang.String r0 = "double"
            java.lang.String r4 = "double"
            r42 = r0
            goto L_0x071b
        L_0x06ac:
            r43 = r7
            goto L_0x071b
        L_0x06af:
            r42 = r3
            r43 = r7
            if (r14 == 0) goto L_0x06e0
            java.lang.String[] r0 = r14.annotations
            if (r0 == 0) goto L_0x06e0
            java.lang.String[] r0 = r14.annotations
            int r3 = r0.length
            r7 = 0
        L_0x06bd:
            if (r7 >= r3) goto L_0x06e0
            r11 = r0[r7]
            java.lang.StringBuilder r15 = new java.lang.StringBuilder
            r15.<init>()
            r44 = r0
            java.lang.String r0 = r13.text
            r15.append(r0)
            r15.append(r11)
            java.lang.String r0 = " "
            r15.append(r0)
            java.lang.String r0 = r15.toString()
            r13.text = r0
            int r7 = r7 + 1
            r0 = r44
            goto L_0x06bd
        L_0x06e0:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = r13.text
            r0.append(r3)
            java.lang.String r3 = "public static native @MemberGetter "
            r0.append(r3)
            r0.append(r4)
            java.lang.String r3 = " "
            r0.append(r3)
            r0.append(r2)
            java.lang.String r3 = "();\n"
            r0.append(r3)
            java.lang.String r0 = r0.toString()
            r13.text = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r3 = " "
            r0.append(r3)
            r0.append(r2)
            java.lang.String r3 = "()"
            r0.append(r3)
            java.lang.String r3 = r0.toString()
        L_0x071b:
            r7 = 32
            int r0 = r4.lastIndexOf(r7)
            if (r0 < 0) goto L_0x0729
            int r7 = r0 + 1
            java.lang.String r4 = r4.substring(r7)
        L_0x0729:
            int r7 = r3.length()
            if (r7 <= 0) goto L_0x075c
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r11 = r13.text
            r7.append(r11)
            java.lang.String r11 = "public static final "
            r7.append(r11)
            r7.append(r4)
            java.lang.String r11 = " "
            r7.append(r11)
            r7.append(r2)
            java.lang.String r11 = " ="
            r7.append(r11)
            r7.append(r3)
            java.lang.String r11 = ";\n"
            r7.append(r11)
            java.lang.String r7 = r7.toString()
            r13.text = r7
        L_0x075c:
            r13.signature = r2
            if (r14 == 0) goto L_0x076c
            java.lang.String[] r7 = r14.cppNames
            java.util.List r7 = java.util.Arrays.asList(r7)
            boolean r7 = r7.contains(r2)
            if (r7 != 0) goto L_0x0797
        L_0x076c:
            org.bytedeco.javacpp.tools.InfoMap r7 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r11 = new org.bytedeco.javacpp.tools.Info
            r45 = r0
            r15 = 1
            java.lang.String[] r0 = new java.lang.String[r15]
            r19 = 0
            r0[r19] = r2
            r11.<init>((java.lang.String[]) r0)
            org.bytedeco.javacpp.tools.Info r0 = r11.define(r15)
            java.lang.String[] r11 = new java.lang.String[r15]
            r11[r19] = r42
            org.bytedeco.javacpp.tools.Info r0 = r0.cppTypes(r11)
            java.lang.String[] r11 = new java.lang.String[r15]
            r11[r19] = r4
            org.bytedeco.javacpp.tools.Info r0 = r0.pointerTypes(r11)
            org.bytedeco.javacpp.tools.Info r0 = r0.translate(r10)
            r7.put(r0)
        L_0x0797:
            r0 = r2
        L_0x0798:
            if (r14 == 0) goto L_0x07a5
            java.lang.String r2 = r14.javaText
            if (r2 == 0) goto L_0x07a5
            java.lang.String r2 = r14.javaText
            r13.text = r2
            r13.signature = r2
            goto L_0x07bb
        L_0x07a5:
            r10 = r21
            r3 = r22
            r9 = r32
            r11 = r33
            r15 = r34
            r2 = r48
            r4 = 1
            r7 = 0
            r14 = 0
            goto L_0x00ea
        L_0x07b7:
            r22 = r3
            r32 = r9
        L_0x07bb:
            java.lang.String r0 = r13.text
            int r0 = r0.length()
            if (r0 != 0) goto L_0x0851
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.index = r8
            r2 = 10
            int r0 = r5.lastIndexOf(r2)
            r2 = 1
            int r0 = r0 + r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = r13.text
            r2.append(r3)
            java.lang.String r3 = "// "
            r2.append(r3)
            java.lang.String r3 = r5.substring(r0)
            r2.append(r3)
            java.lang.String r3 = "#"
            r2.append(r3)
            java.lang.String r3 = r6.spacing
            r2.append(r3)
            r2.append(r6)
            java.lang.String r2 = r2.toString()
            r13.text = r2
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r1.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.get()
        L_0x07fe:
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            int r3 = r3.index
            if (r3 >= r12) goto L_0x084c
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r13.text
            r3.append(r4)
            r4 = 1
            java.lang.Object[] r7 = new java.lang.Object[r4]
            java.lang.String r4 = "\n"
            r9 = 0
            r7[r9] = r4
            boolean r4 = r2.match(r7)
            if (r4 == 0) goto L_0x081f
            java.lang.String r4 = "\n// "
            goto L_0x083c
        L_0x081f:
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r7 = r2.spacing
            r4.append(r7)
            java.lang.String r7 = r2.toString()
            java.lang.String r9 = "\n"
            java.lang.String r10 = "\n//"
            java.lang.String r7 = r7.replace(r9, r10)
            r4.append(r7)
            java.lang.String r4 = r4.toString()
        L_0x083c:
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r13.text = r3
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            org.bytedeco.javacpp.tools.Token r2 = r3.next()
            goto L_0x07fe
        L_0x084c:
            r2 = 0
            java.lang.String r5 = r5.substring(r2, r0)
        L_0x0851:
            java.lang.String r0 = r13.text
            int r0 = r0.length()
            if (r0 <= 0) goto L_0x0874
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.index = r12
            java.lang.String r0 = r46.commentAfter()
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r2.append(r0)
            java.lang.String r3 = r13.text
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r13.text = r2
        L_0x0874:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r2 = 0
            r0.raw = r2
            r2 = r48
            r2.spacing = r5
            r2.add((org.bytedeco.javacpp.tools.Declaration) r13)
            r3 = 0
            r2.spacing = r3
            r3 = 1
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.macro(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):boolean");
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:10:0x006b  */
    /* JADX WARNING: Removed duplicated region for block: B:143:0x041a A[SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:16:0x0087  */
    /* JADX WARNING: Removed duplicated region for block: B:20:0x00ba  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean typedef(org.bytedeco.javacpp.tools.Context r21, org.bytedeco.javacpp.tools.DeclarationList r22) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r20 = this;
            r8 = r20
            r9 = r21
            r10 = r22
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r11 = r0.spacing
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r12 = 1
            java.lang.Object[] r1 = new java.lang.Object[r12]
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.USING
            r13 = 0
            r1[r13] = r2
            boolean r0 = r0.match(r1)
            r1 = 61
            r14 = 2
            r2 = 5
            if (r0 == 0) goto L_0x0057
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r12)
            java.lang.Object[] r3 = new java.lang.Object[r12]
            java.lang.Integer r4 = java.lang.Integer.valueOf(r2)
            r3[r13] = r4
            boolean r0 = r0.match(r3)
            if (r0 == 0) goto L_0x0057
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r14)
            java.lang.Object[] r3 = new java.lang.Object[r12]
            java.lang.Character r4 = java.lang.Character.valueOf(r1)
            r3[r13] = r4
            boolean r0 = r0.match(r3)
            if (r0 == 0) goto L_0x0057
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r12)
            java.lang.String r0 = r0.value
            goto L_0x0058
        L_0x0057:
            r0 = 0
        L_0x0058:
            r7 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r3 = new java.lang.Object[r12]
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.TYPEDEF
            r3[r13] = r4
            boolean r0 = r0.match(r3)
            if (r0 != 0) goto L_0x0080
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r12)
            java.lang.Object[] r3 = new java.lang.Object[r12]
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.TYPEDEF
            r3[r13] = r4
            boolean r0 = r0.match(r3)
            if (r0 != 0) goto L_0x0080
            if (r7 != 0) goto L_0x0080
            return r13
        L_0x0080:
            org.bytedeco.javacpp.tools.Declaration r0 = new org.bytedeco.javacpp.tools.Declaration
            r0.<init>()
            if (r7 == 0) goto L_0x00ae
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.next()
            java.lang.Object[] r4 = new java.lang.Object[r12]
            java.lang.Integer r2 = java.lang.Integer.valueOf(r2)
            r4[r13] = r2
            r3.expect(r4)
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.next()
            java.lang.Object[] r3 = new java.lang.Object[r12]
            java.lang.Character r1 = java.lang.Character.valueOf(r1)
            r3[r13] = r1
            r2.expect(r3)
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            r1.next()
        L_0x00ae:
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            int r6 = r1.index
            r1 = r0
            r0 = 0
        L_0x00b4:
            r5 = r0
            r0 = 2147483647(0x7fffffff, float:NaN)
            if (r5 >= r0) goto L_0x041a
            org.bytedeco.javacpp.tools.Declaration r0 = new org.bytedeco.javacpp.tools.Declaration
            r0.<init>()
            r4 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r6
            r2 = 0
            r3 = 0
            r16 = 0
            r17 = 1
            r18 = 0
            r0 = r20
            r1 = r21
            r15 = r4
            r4 = r16
            r16 = r5
            r19 = r6
            r6 = r17
            r14 = r7
            r7 = r18
            org.bytedeco.javacpp.tools.Declarator r0 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            if (r0 != 0) goto L_0x00e4
            goto L_0x041e
        L_0x00e4:
            if (r14 == 0) goto L_0x00e8
            r0.cppName = r14
        L_0x00e8:
            org.bytedeco.javacpp.tools.Attribute r1 = r20.attribute()
            if (r1 != 0) goto L_0x00f3
            org.bytedeco.javacpp.tools.TokenIndexer r1 = r8.tokens
            r1.next()
        L_0x00f3:
            org.bytedeco.javacpp.tools.Type r1 = r0.type
            java.lang.String r1 = r1.cppName
            java.lang.String r2 = r0.cppName
            if (r2 != 0) goto L_0x00fe
            r2 = r1
            r0.cppName = r1
        L_0x00fe:
            java.lang.String r3 = r0.javaName
            if (r3 != 0) goto L_0x0106
            java.lang.String r3 = r0.cppName
            r0.javaName = r3
        L_0x0106:
            java.lang.String r3 = "::"
            int r3 = r2.lastIndexOf(r3)
            java.lang.String r4 = r9.namespace
            if (r4 == 0) goto L_0x0128
            if (r3 >= 0) goto L_0x0128
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r9.namespace
            r4.append(r5)
            java.lang.String r5 = "::"
            r4.append(r5)
            r4.append(r2)
            java.lang.String r2 = r4.toString()
        L_0x0128:
            org.bytedeco.javacpp.tools.InfoMap r4 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r4 = r4.getFirst(r2)
            org.bytedeco.javacpp.tools.Declaration r5 = r0.definition
            if (r5 == 0) goto L_0x01b2
            org.bytedeco.javacpp.tools.Declaration r5 = r0.definition
            java.lang.String r6 = r0.javaName
            int r6 = r6.length()
            if (r6 <= 0) goto L_0x015a
            java.lang.String r6 = r9.javaName
            if (r6 == 0) goto L_0x015a
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r9.javaName
            r6.append(r7)
            java.lang.String r7 = "."
            r6.append(r7)
            java.lang.String r7 = r0.javaName
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r0.javaName = r6
        L_0x015a:
            if (r4 == 0) goto L_0x0165
            boolean r6 = r4.skip
            if (r6 != 0) goto L_0x0161
            goto L_0x0165
        L_0x0161:
            r15 = r5
        L_0x0162:
            r7 = 0
            goto L_0x03e4
        L_0x0165:
            if (r4 == 0) goto L_0x0175
            org.bytedeco.javacpp.tools.Info r6 = new org.bytedeco.javacpp.tools.Info
            r6.<init>((org.bytedeco.javacpp.tools.Info) r4)
            java.lang.String[] r7 = new java.lang.String[r12]
            r7[r13] = r2
            org.bytedeco.javacpp.tools.Info r6 = r6.cppNames(r7)
            goto L_0x017e
        L_0x0175:
            org.bytedeco.javacpp.tools.Info r6 = new org.bytedeco.javacpp.tools.Info
            java.lang.String[] r7 = new java.lang.String[r12]
            r7[r13] = r2
            r6.<init>((java.lang.String[]) r7)
        L_0x017e:
            r4 = r6
            org.bytedeco.javacpp.tools.InfoMap r6 = r8.infoMap
            java.lang.String[] r7 = new java.lang.String[r12]
            java.lang.String r15 = r0.javaName
            r7[r13] = r15
            org.bytedeco.javacpp.tools.Info r7 = r4.valueTypes(r7)
            java.lang.String[] r15 = new java.lang.String[r12]
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            int r13 = r0.indirections
            if (r13 <= 0) goto L_0x0199
            java.lang.String r13 = "@ByPtrPtr "
            goto L_0x019b
        L_0x0199:
            java.lang.String r13 = ""
        L_0x019b:
            r12.append(r13)
            java.lang.String r13 = r0.javaName
            r12.append(r13)
            java.lang.String r12 = r12.toString()
            r13 = 0
            r15[r13] = r12
            org.bytedeco.javacpp.tools.Info r7 = r7.pointerTypes(r15)
            r6.put(r7)
            goto L_0x0161
        L_0x01b2:
            java.lang.String r5 = "void"
            boolean r5 = r1.equals(r5)
            if (r5 == 0) goto L_0x0292
            if (r4 == 0) goto L_0x01c0
            boolean r5 = r4.skip
            if (r5 != 0) goto L_0x0162
        L_0x01c0:
            java.lang.String r5 = r0.javaName
            java.lang.String r6 = "Pointer"
            boolean r5 = r5.equals(r6)
            if (r5 != 0) goto L_0x0162
            int r5 = r0.indirections
            if (r5 <= 0) goto L_0x022d
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r15.text
            r5.append(r6)
            java.lang.String r6 = "@Namespace @Name(\"void\") "
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r15.text = r5
            if (r4 == 0) goto L_0x01f5
            org.bytedeco.javacpp.tools.Info r5 = new org.bytedeco.javacpp.tools.Info
            r5.<init>((org.bytedeco.javacpp.tools.Info) r4)
            r6 = 1
            java.lang.String[] r7 = new java.lang.String[r6]
            r12 = 0
            r7[r12] = r2
            org.bytedeco.javacpp.tools.Info r5 = r5.cppNames(r7)
            goto L_0x0200
        L_0x01f5:
            r6 = 1
            r12 = 0
            org.bytedeco.javacpp.tools.Info r5 = new org.bytedeco.javacpp.tools.Info
            java.lang.String[] r7 = new java.lang.String[r6]
            r7[r12] = r2
            r5.<init>((java.lang.String[]) r7)
        L_0x0200:
            r4 = r5
            org.bytedeco.javacpp.tools.InfoMap r5 = r8.infoMap
            java.lang.String[] r7 = new java.lang.String[r6]
            java.lang.String r13 = r0.javaName
            r7[r12] = r13
            org.bytedeco.javacpp.tools.Info r7 = r4.valueTypes(r7)
            java.lang.String[] r13 = new java.lang.String[r6]
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r12 = "@ByPtrPtr "
            r6.append(r12)
            java.lang.String r12 = r0.javaName
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r12 = 0
            r13[r12] = r6
            org.bytedeco.javacpp.tools.Info r6 = r7.pointerTypes(r13)
            r5.put(r6)
            goto L_0x0254
        L_0x022d:
            java.lang.String r5 = r9.namespace
            if (r5 == 0) goto L_0x0254
            java.lang.String r5 = r9.javaName
            if (r5 != 0) goto L_0x0254
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r15.text
            r5.append(r6)
            java.lang.String r6 = "@Namespace(\""
            r5.append(r6)
            java.lang.String r6 = r9.namespace
            r5.append(r6)
            java.lang.String r6 = "\") "
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r15.text = r5
        L_0x0254:
            org.bytedeco.javacpp.tools.Type r5 = new org.bytedeco.javacpp.tools.Type
            java.lang.String r6 = r0.javaName
            r5.<init>(r6)
            r15.type = r5
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r15.text
            r5.append(r6)
            java.lang.String r6 = "@Opaque public static class "
            r5.append(r6)
            java.lang.String r6 = r0.javaName
            r5.append(r6)
            java.lang.String r6 = " extends Pointer {\n    /** Empty constructor. Calls {@code super((Pointer)null)}. */\n    public "
            r5.append(r6)
            java.lang.String r6 = r0.javaName
            r5.append(r6)
            java.lang.String r6 = "() { super((Pointer)null); }\n    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */\n    public "
            r5.append(r6)
            java.lang.String r6 = r0.javaName
            r5.append(r6)
            java.lang.String r6 = "(Pointer p) { super(p); }\n}"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r15.text = r5
            goto L_0x0162
        L_0x0292:
            org.bytedeco.javacpp.tools.InfoMap r5 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r4 = r5.getFirst(r1)
            if (r4 == 0) goto L_0x029e
            boolean r5 = r4.skip
            if (r5 != 0) goto L_0x0162
        L_0x029e:
            if (r4 == 0) goto L_0x02b0
            org.bytedeco.javacpp.tools.Info r5 = new org.bytedeco.javacpp.tools.Info
            r5.<init>((org.bytedeco.javacpp.tools.Info) r4)
            r6 = 1
            java.lang.String[] r7 = new java.lang.String[r6]
            r12 = 0
            r7[r12] = r2
            org.bytedeco.javacpp.tools.Info r5 = r5.cppNames(r7)
            goto L_0x02bb
        L_0x02b0:
            r6 = 1
            r12 = 0
            org.bytedeco.javacpp.tools.Info r5 = new org.bytedeco.javacpp.tools.Info
            java.lang.String[] r7 = new java.lang.String[r6]
            r7[r12] = r2
            r5.<init>((java.lang.String[]) r7)
        L_0x02bb:
            r4 = r5
            java.lang.String[] r5 = r4.cppTypes
            if (r5 != 0) goto L_0x0351
            java.lang.String[] r5 = r4.annotations
            if (r5 == 0) goto L_0x0351
            r5 = r1
            org.bytedeco.javacpp.tools.Type r6 = r0.type
            boolean r6 = r6.constValue
            if (r6 == 0) goto L_0x02e4
            java.lang.String r6 = "const "
            boolean r6 = r5.startsWith(r6)
            if (r6 != 0) goto L_0x02e4
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = "const "
            r6.append(r7)
            r6.append(r5)
            java.lang.String r5 = r6.toString()
        L_0x02e4:
            org.bytedeco.javacpp.tools.Type r6 = r0.type
            boolean r6 = r6.constPointer
            if (r6 == 0) goto L_0x0303
            java.lang.String r6 = " const"
            boolean r6 = r5.endsWith(r6)
            if (r6 != 0) goto L_0x0303
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r5)
            java.lang.String r7 = " const"
            r6.append(r7)
            java.lang.String r5 = r6.toString()
        L_0x0303:
            org.bytedeco.javacpp.tools.Type r6 = r0.type
            int r6 = r6.indirections
            if (r6 <= 0) goto L_0x0326
            r6 = r5
            r5 = 0
        L_0x030b:
            org.bytedeco.javacpp.tools.Type r7 = r0.type
            int r7 = r7.indirections
            if (r5 >= r7) goto L_0x0325
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r6)
            java.lang.String r12 = "*"
            r7.append(r12)
            java.lang.String r6 = r7.toString()
            int r5 = r5 + 1
            goto L_0x030b
        L_0x0325:
            r5 = r6
        L_0x0326:
            org.bytedeco.javacpp.tools.Type r6 = r0.type
            boolean r6 = r6.reference
            if (r6 == 0) goto L_0x033d
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r5)
            java.lang.String r7 = "&"
            r6.append(r7)
            java.lang.String r5 = r6.toString()
        L_0x033d:
            r6 = 2
            java.lang.String[] r7 = new java.lang.String[r6]
            r12 = 0
            r7[r12] = r2
            r13 = 1
            r7[r13] = r5
            org.bytedeco.javacpp.tools.Info r7 = r4.cppNames(r7)
            java.lang.String[] r6 = new java.lang.String[r13]
            r6[r12] = r5
            r7.cppTypes(r6)
        L_0x0351:
            java.lang.String[] r5 = r4.valueTypes
            if (r5 != 0) goto L_0x0376
            int r5 = r0.indirections
            if (r5 <= 0) goto L_0x0376
            java.lang.String[] r5 = r4.pointerTypes
            if (r5 == 0) goto L_0x0363
            java.lang.String[] r5 = r4.pointerTypes
            r6 = r5
            r5 = 1
            r7 = 0
            goto L_0x0369
        L_0x0363:
            r5 = 1
            java.lang.String[] r6 = new java.lang.String[r5]
            r7 = 0
            r6[r7] = r1
        L_0x0369:
            r4.valueTypes(r6)
            java.lang.String[] r6 = new java.lang.String[r5]
            java.lang.String r12 = "PointerPointer"
            r6[r7] = r12
            r4.pointerTypes(r6)
            goto L_0x0383
        L_0x0376:
            r5 = 1
            r7 = 0
            java.lang.String[] r6 = r4.pointerTypes
            if (r6 != 0) goto L_0x0383
            java.lang.String[] r6 = new java.lang.String[r5]
            r6[r7] = r1
            r4.pointerTypes(r6)
        L_0x0383:
            java.lang.String[] r5 = r4.annotations
            if (r5 != 0) goto L_0x03de
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r5 = r5.annotations
            if (r5 == 0) goto L_0x03cd
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r5 = r5.annotations
            int r5 = r5.length()
            if (r5 <= 0) goto L_0x03cd
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r5 = r5.annotations
            java.lang.String r6 = "@ByVal "
            boolean r5 = r5.startsWith(r6)
            if (r5 != 0) goto L_0x03cd
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r5 = r5.annotations
            java.lang.String r6 = "@Cast("
            boolean r5 = r5.startsWith(r6)
            if (r5 != 0) goto L_0x03cd
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r5 = r5.annotations
            java.lang.String r6 = "@Const "
            boolean r5 = r5.startsWith(r6)
            if (r5 != 0) goto L_0x03cd
            r5 = 1
            java.lang.String[] r6 = new java.lang.String[r5]
            org.bytedeco.javacpp.tools.Type r5 = r0.type
            java.lang.String r5 = r5.annotations
            java.lang.String r5 = r5.trim()
            r7 = 0
            r6[r7] = r5
            r4.annotations(r6)
            goto L_0x03df
        L_0x03cd:
            r7 = 0
            java.lang.String r5 = r0.cppName
            java.lang.String[] r6 = r4.pointerTypes
            r6 = r6[r7]
            boolean r5 = r5.equals(r6)
            r6 = 1
            r5 = r5 ^ r6
            r4.cast(r5)
            goto L_0x03df
        L_0x03de:
            r7 = 0
        L_0x03df:
            org.bytedeco.javacpp.tools.InfoMap r5 = r8.infoMap
            r5.put(r4)
        L_0x03e4:
            if (r4 == 0) goto L_0x03f0
            java.lang.String r5 = r4.javaText
            if (r5 == 0) goto L_0x03f0
            java.lang.String r5 = r4.javaText
            r15.text = r5
            r15.signature = r5
        L_0x03f0:
            java.lang.String r5 = r20.commentAfter()
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r6.append(r5)
            java.lang.String r12 = r15.text
            r6.append(r12)
            java.lang.String r6 = r6.toString()
            r15.text = r6
            r10.spacing = r11
            r10.add((org.bytedeco.javacpp.tools.Declaration) r15)
            r6 = 0
            r10.spacing = r6
            int r0 = r16 + 1
            r7 = r14
            r1 = r15
            r6 = r19
            r12 = 1
            r13 = 0
            r14 = 2
            goto L_0x00b4
        L_0x041a:
            r19 = r6
            r14 = r7
            r15 = r1
        L_0x041e:
            r0 = 1
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.typedef(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):boolean");
    }

    /* access modifiers changed from: package-private */
    public boolean using(Context context, DeclarationList declList) throws ParserException {
        if (!this.tokens.get().match(Token.USING)) {
            return false;
        }
        String spacing = this.tokens.get().spacing;
        boolean namespace = this.tokens.get(1).match(Token.NAMESPACE);
        Declarator dcl = declarator(context, (String) null, 0, false, 0, true, false);
        this.tokens.next();
        List<String> list = context.usingList;
        StringBuilder sb = new StringBuilder();
        sb.append(dcl.type.cppName);
        sb.append(namespace ? "::" : "");
        list.add(sb.toString());
        Declaration decl = new Declaration();
        if (dcl.definition != null) {
            decl = dcl.definition;
        }
        Info info = this.infoMap.getFirst(dcl.type.cppName);
        if (!(context.inaccessible || info == null || info.javaText == null)) {
            String str = info.javaText;
            decl.text = str;
            decl.signature = str;
            decl.declarator = dcl;
        }
        String comment = commentAfter();
        decl.text = comment + decl.text;
        declList.spacing = spacing;
        declList.add(decl);
        declList.spacing = null;
        return true;
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:30:0x0125, code lost:
        if (r8.tokens.get(2).match(';') == false) goto L_0x0127;
     */
    /* JADX WARNING: Removed duplicated region for block: B:20:0x00b3  */
    /* JADX WARNING: Removed duplicated region for block: B:22:0x00b8  */
    /* JADX WARNING: Removed duplicated region for block: B:240:0x085d  */
    /* JADX WARNING: Removed duplicated region for block: B:301:0x09b7  */
    /* JADX WARNING: Removed duplicated region for block: B:352:0x0bb0  */
    /* JADX WARNING: Removed duplicated region for block: B:356:0x0bc8  */
    /* JADX WARNING: Removed duplicated region for block: B:379:0x0ca8  */
    /* JADX WARNING: Removed duplicated region for block: B:397:0x0d0a  */
    /* JADX WARNING: Removed duplicated region for block: B:400:0x0d17  */
    /* JADX WARNING: Removed duplicated region for block: B:405:0x0d46  */
    /* JADX WARNING: Removed duplicated region for block: B:406:0x0d4d  */
    /* JADX WARNING: Removed duplicated region for block: B:410:0x0d88  */
    /* JADX WARNING: Removed duplicated region for block: B:414:0x0dbc  */
    /* JADX WARNING: Removed duplicated region for block: B:422:0x0df4  */
    /* JADX WARNING: Removed duplicated region for block: B:423:0x0dfb A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:432:0x00af A[EDGE_INSN: B:432:0x00af->B:18:0x00af ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:441:0x024d A[EDGE_INSN: B:441:0x024d->B:68:0x024d ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:482:0x0de7 A[EDGE_INSN: B:482:0x0de7->B:418:0x0de7 ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:67:0x0247 A[LOOP:2: B:65:0x022c->B:67:0x0247, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:70:0x0273  */
    /* JADX WARNING: Removed duplicated region for block: B:72:0x0278  */
    /* JADX WARNING: Removed duplicated region for block: B:9:0x0075  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean group(org.bytedeco.javacpp.tools.Context r84, org.bytedeco.javacpp.tools.DeclarationList r85) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r83 = this;
            r8 = r83
            r9 = r84
            r10 = r85
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            int r11 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r12 = r0.spacing
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r13 = 1
            java.lang.Object[] r1 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.TYPEDEF
            r14 = 0
            r1[r14] = r2
            boolean r0 = r0.match(r1)
            if (r0 != 0) goto L_0x003b
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r13)
            java.lang.Object[] r1 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.TYPEDEF
            r1[r14] = r2
            boolean r0 = r0.match(r1)
            if (r0 == 0) goto L_0x0039
            goto L_0x003b
        L_0x0039:
            r0 = 0
            goto L_0x003c
        L_0x003b:
            r0 = 1
        L_0x003c:
            r15 = r0
            r0 = 0
            r1 = 0
            org.bytedeco.javacpp.tools.Context r2 = new org.bytedeco.javacpp.tools.Context
            r2.<init>(r9)
            r7 = r2
            r2 = 5
            org.bytedeco.javacpp.tools.Token[] r3 = new org.bytedeco.javacpp.tools.Token[r2]
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.CLASS
            r3[r14] = r4
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.INTERFACE
            r3[r13] = r4
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.__INTERFACE
            r6 = 2
            r3[r6] = r4
            org.bytedeco.javacpp.tools.Token r4 = org.bytedeco.javacpp.tools.Token.STRUCT
            r5 = 3
            r3[r5] = r4
            r4 = 4
            org.bytedeco.javacpp.tools.Token r16 = org.bytedeco.javacpp.tools.Token.UNION
            r3[r4] = r16
            r4 = r3
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.get()
            r16 = r1
        L_0x0068:
            r1 = r3
            java.lang.Object[] r3 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r17 = org.bytedeco.javacpp.tools.Token.EOF
            r3[r14] = r17
            boolean r3 = r1.match(r3)
            if (r3 != 0) goto L_0x00af
            boolean r3 = r1.match(r4)
            if (r3 == 0) goto L_0x0089
            r0 = 1
            java.lang.Object[] r3 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r17 = org.bytedeco.javacpp.tools.Token.CLASS
            r3[r14] = r17
            boolean r3 = r1.match(r3)
            r7.inaccessible = r3
            goto L_0x00af
        L_0x0089:
            java.lang.Object[] r3 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r17 = org.bytedeco.javacpp.tools.Token.FRIEND
            r3[r14] = r17
            boolean r3 = r1.match(r3)
            if (r3 == 0) goto L_0x0099
            r3 = 1
            r16 = r3
            goto L_0x00a8
        L_0x0099:
            java.lang.Object[] r3 = new java.lang.Object[r13]
            java.lang.Integer r17 = java.lang.Integer.valueOf(r2)
            r3[r14] = r17
            boolean r3 = r1.match(r3)
            if (r3 != 0) goto L_0x00a8
            goto L_0x00af
        L_0x00a8:
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.next()
            goto L_0x0068
        L_0x00af:
            r17 = r0
            if (r17 != 0) goto L_0x00b8
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            return r14
        L_0x00b8:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            java.lang.Object[] r1 = new java.lang.Object[r5]
            java.lang.Integer r3 = java.lang.Integer.valueOf(r2)
            r1[r14] = r3
            r3 = 123(0x7b, float:1.72E-43)
            java.lang.Character r18 = java.lang.Character.valueOf(r3)
            r1[r13] = r18
            java.lang.String r18 = "::"
            r1[r6] = r18
            r0.expect(r1)
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.Object[] r1 = new java.lang.Object[r13]
            java.lang.Character r18 = java.lang.Character.valueOf(r3)
            r1[r14] = r18
            boolean r0 = r0.match(r1)
            r18 = 59
            if (r0 != 0) goto L_0x012c
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r13)
            java.lang.Object[] r1 = new java.lang.Object[r13]
            java.lang.Integer r19 = java.lang.Integer.valueOf(r2)
            r1[r14] = r19
            boolean r0 = r0.match(r1)
            if (r0 == 0) goto L_0x012c
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r13)
            java.lang.Object[] r1 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r19 = org.bytedeco.javacpp.tools.Token.FINAL
            r1[r14] = r19
            boolean r0 = r0.match(r1)
            if (r0 != 0) goto L_0x012c
            if (r15 != 0) goto L_0x0127
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r6)
            java.lang.Object[] r1 = new java.lang.Object[r13]
            java.lang.Character r19 = java.lang.Character.valueOf(r18)
            r1[r14] = r19
            boolean r0 = r0.match(r1)
            if (r0 != 0) goto L_0x012c
        L_0x0127:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
        L_0x012c:
            org.bytedeco.javacpp.tools.Type r1 = r8.type(r9, r13)
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            org.bytedeco.javacpp.tools.Declaration r19 = new org.bytedeco.javacpp.tools.Declaration
            r19.<init>()
            r20 = r19
            java.lang.String r2 = r1.annotations
            r22 = r7
            r7 = r20
            r7.text = r2
            java.lang.String r2 = r1.javaName
            if (r15 != 0) goto L_0x0152
            java.lang.String r3 = r1.cppName
            int r3 = r3.length()
            if (r3 != 0) goto L_0x0152
            r3 = 1
            goto L_0x0153
        L_0x0152:
            r3 = 0
        L_0x0153:
            r19 = 0
            r20 = 0
            java.lang.String r6 = r1.cppName
            int r6 = r6.length()
            r25 = 44
            if (r6 <= 0) goto L_0x0222
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r8.tokens
            org.bytedeco.javacpp.tools.Token r6 = r6.get()
            java.lang.Object[] r5 = new java.lang.Object[r13]
            r27 = 58
            java.lang.Character r27 = java.lang.Character.valueOf(r27)
            r5[r14] = r27
            boolean r5 = r6.match(r5)
            if (r5 == 0) goto L_0x0222
            r19 = 1
            org.bytedeco.javacpp.tools.TokenIndexer r5 = r8.tokens
            org.bytedeco.javacpp.tools.Token r5 = r5.next()
        L_0x017f:
            java.lang.Object[] r6 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r27 = org.bytedeco.javacpp.tools.Token.EOF
            r6[r14] = r27
            boolean r6 = r5.match(r6)
            if (r6 != 0) goto L_0x0222
            r6 = 0
            r28 = r2
            java.lang.Object[] r2 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r27 = org.bytedeco.javacpp.tools.Token.VIRTUAL
            r2[r14] = r27
            boolean r2 = r5.match(r2)
            if (r2 == 0) goto L_0x019f
            r30 = r0
            goto L_0x0214
        L_0x019f:
            r2 = 3
            java.lang.Object[] r13 = new java.lang.Object[r2]
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.PRIVATE
            r13[r14] = r2
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.PROTECTED
            r14 = 1
            r13[r14] = r2
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.PUBLIC
            r24 = 2
            r13[r24] = r2
            boolean r2 = r5.match(r13)
            if (r2 == 0) goto L_0x01c7
            java.lang.Object[] r2 = new java.lang.Object[r14]
            org.bytedeco.javacpp.tools.Token r13 = org.bytedeco.javacpp.tools.Token.PUBLIC
            r14 = 0
            r2[r14] = r13
            boolean r6 = r5.match(r2)
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r8.tokens
            r2.next()
        L_0x01c7:
            org.bytedeco.javacpp.tools.Type r2 = r83.type(r84)
            org.bytedeco.javacpp.tools.InfoMap r13 = r8.infoMap
            java.lang.String r14 = r2.cppName
            org.bytedeco.javacpp.tools.Info r13 = r13.getFirst(r14)
            if (r13 == 0) goto L_0x01dc
            boolean r14 = r13.skip
            if (r14 == 0) goto L_0x01dc
            r14 = 1
            r20 = r14
        L_0x01dc:
            if (r6 == 0) goto L_0x01e1
            r0.add(r2)
        L_0x01e1:
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r8.tokens
            org.bytedeco.javacpp.tools.Token r14 = r14.get()
            r30 = r0
            r31 = r2
            r0 = 2
            java.lang.Object[] r2 = new java.lang.Object[r0]
            java.lang.Character r0 = java.lang.Character.valueOf(r25)
            r27 = 0
            r2[r27] = r0
            r0 = 123(0x7b, float:1.72E-43)
            java.lang.Character r23 = java.lang.Character.valueOf(r0)
            r0 = 1
            r2[r0] = r23
            org.bytedeco.javacpp.tools.Token r2 = r14.expect(r2)
            java.lang.Object[] r14 = new java.lang.Object[r0]
            r0 = 123(0x7b, float:1.72E-43)
            java.lang.Character r29 = java.lang.Character.valueOf(r0)
            r14[r27] = r29
            boolean r0 = r2.match(r14)
            if (r0 == 0) goto L_0x0214
            goto L_0x0226
        L_0x0214:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r5 = r0.next()
            r2 = r28
            r0 = r30
            r13 = 1
            r14 = 0
            goto L_0x017f
        L_0x0222:
            r30 = r0
            r28 = r2
        L_0x0226:
            if (r15 == 0) goto L_0x024d
            int r0 = r1.indirections
            if (r0 <= 0) goto L_0x024d
        L_0x022c:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r2 = 2
            java.lang.Object[] r5 = new java.lang.Object[r2]
            java.lang.Character r2 = java.lang.Character.valueOf(r18)
            r6 = 0
            r5[r6] = r2
            org.bytedeco.javacpp.tools.Token r2 = org.bytedeco.javacpp.tools.Token.EOF
            r6 = 1
            r5[r6] = r2
            boolean r0 = r0.match(r5)
            if (r0 != 0) goto L_0x024d
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.next()
            goto L_0x022c
        L_0x024d:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r2 = 3
            java.lang.Object[] r2 = new java.lang.Object[r2]
            r5 = 123(0x7b, float:1.72E-43)
            java.lang.Character r6 = java.lang.Character.valueOf(r5)
            r13 = 0
            r2[r13] = r6
            java.lang.Character r6 = java.lang.Character.valueOf(r25)
            r14 = 1
            r2[r14] = r6
            java.lang.Character r6 = java.lang.Character.valueOf(r18)
            r14 = 2
            r2[r14] = r6
            boolean r0 = r0.match(r2)
            if (r0 != 0) goto L_0x0278
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            return r13
        L_0x0278:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            int r13 = r0.index
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r14 = r0
            java.lang.String r6 = r1.cppName
            java.lang.String r23 = r83.body()
            if (r23 == 0) goto L_0x0292
            int r0 = r23.length()
            if (r0 <= 0) goto L_0x0292
            r0 = 1
            goto L_0x0293
        L_0x0292:
            r0 = 0
        L_0x0293:
            r25 = r0
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r2 = 1
            java.lang.Object[] r5 = new java.lang.Object[r2]
            java.lang.Character r2 = java.lang.Character.valueOf(r18)
            r26 = 0
            r5[r26] = r2
            boolean r0 = r0.match(r5)
            if (r0 != 0) goto L_0x0439
            if (r15 == 0) goto L_0x03ab
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r2 = r28
        L_0x02b6:
            r33 = r4
            r4 = 2
            java.lang.Object[] r5 = new java.lang.Object[r4]
            java.lang.Character r24 = java.lang.Character.valueOf(r18)
            r26 = 0
            r5[r26] = r24
            org.bytedeco.javacpp.tools.Token r24 = org.bytedeco.javacpp.tools.Token.EOF
            r4 = 1
            r5[r4] = r24
            boolean r5 = r0.match(r5)
            if (r5 != 0) goto L_0x037c
            r5 = r0
            r0 = 0
        L_0x02d0:
            r34 = r3
            java.lang.Object[] r3 = new java.lang.Object[r4]
            r4 = 42
            java.lang.Character r4 = java.lang.Character.valueOf(r4)
            r24 = 0
            r3[r24] = r4
            boolean r3 = r5.match(r3)
            if (r3 == 0) goto L_0x02f0
            int r0 = r0 + 1
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r5 = r3.next()
            r3 = r34
            r4 = 1
            goto L_0x02d0
        L_0x02f0:
            r3 = 1
            java.lang.Object[] r4 = new java.lang.Object[r3]
            r3 = 5
            java.lang.Integer r21 = java.lang.Integer.valueOf(r3)
            r24 = 0
            r4[r24] = r21
            boolean r4 = r5.match(r4)
            if (r4 == 0) goto L_0x0365
            java.lang.String r4 = r5.value
            if (r0 <= 0) goto L_0x0332
            org.bytedeco.javacpp.tools.InfoMap r3 = r8.infoMap
            r35 = r0
            org.bytedeco.javacpp.tools.Info r0 = new org.bytedeco.javacpp.tools.Info
            r36 = r11
            r37 = r15
            r11 = 1
            java.lang.String[] r15 = new java.lang.String[r11]
            r15[r24] = r4
            r0.<init>((java.lang.String[]) r15)
            org.bytedeco.javacpp.tools.Info r0 = r0.cast()
            java.lang.String[] r15 = new java.lang.String[r11]
            r15[r24] = r2
            org.bytedeco.javacpp.tools.Info r0 = r0.valueTypes(r15)
            java.lang.String[] r15 = new java.lang.String[r11]
            java.lang.String r11 = "PointerPointer"
            r15[r24] = r11
            org.bytedeco.javacpp.tools.Info r0 = r0.pointerTypes(r15)
            r3.put(r0)
            goto L_0x036b
        L_0x0332:
            r35 = r0
            r36 = r11
            r37 = r15
            java.lang.String r0 = r1.cppName
            boolean r0 = r0.equals(r6)
            if (r0 == 0) goto L_0x0347
            java.lang.String r0 = r5.value
            r1.cppName = r0
            r1.javaName = r0
            r2 = r0
        L_0x0347:
            org.bytedeco.javacpp.tools.InfoMap r0 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r3 = new org.bytedeco.javacpp.tools.Info
            r11 = 1
            java.lang.String[] r15 = new java.lang.String[r11]
            r21 = 0
            r15[r21] = r4
            r3.<init>((java.lang.String[]) r15)
            org.bytedeco.javacpp.tools.Info r3 = r3.cast()
            java.lang.String[] r15 = new java.lang.String[r11]
            r15[r21] = r2
            org.bytedeco.javacpp.tools.Info r3 = r3.pointerTypes(r15)
            r0.put(r3)
            goto L_0x036b
        L_0x0365:
            r35 = r0
            r36 = r11
            r37 = r15
        L_0x036b:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            r4 = r33
            r3 = r34
            r11 = r36
            r15 = r37
            goto L_0x02b6
        L_0x037c:
            r34 = r3
            r36 = r11
            r37 = r15
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r7.text
            r3.append(r4)
            java.lang.String r4 = r0.spacing
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r7.text = r3
            r40 = r1
            r27 = r2
            r41 = r6
            r11 = r7
            r43 = r22
            r26 = r30
            r39 = r33
            r0 = 10
            r38 = 123(0x7b, float:1.72E-43)
            goto L_0x0450
        L_0x03ab:
            r34 = r3
            r33 = r4
            r36 = r11
            r37 = r15
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            int r0 = r0.index
            r2 = 1
            int r11 = r0 + -1
            r0 = 0
        L_0x03bb:
            r15 = r0
            r0 = 2147483647(0x7fffffff, float:NaN)
            if (r15 >= r0) goto L_0x0407
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r8.tokens
            r0.index = r11
            r2 = 0
            r3 = -1
            r4 = 0
            r21 = 0
            r24 = 1
            r26 = r30
            r0 = r83
            r5 = r1
            r1 = r84
            r27 = r28
            r38 = 123(0x7b, float:1.72E-43)
            r39 = r33
            r28 = 2
            r40 = r5
            r5 = r15
            r41 = r6
            r6 = r21
            r42 = r11
            r43 = r22
            r11 = r7
            r7 = r24
            org.bytedeco.javacpp.tools.Declarator r0 = r0.declarator(r1, r2, r3, r4, r5, r6, r7)
            if (r0 != 0) goto L_0x03f0
            goto L_0x0418
        L_0x03f0:
            r34 = 1
            r14.add(r0)
            int r0 = r15 + 1
            r7 = r11
            r30 = r26
            r28 = r27
            r33 = r39
            r1 = r40
            r6 = r41
            r11 = r42
            r22 = r43
            goto L_0x03bb
        L_0x0407:
            r40 = r1
            r41 = r6
            r42 = r11
            r43 = r22
            r27 = r28
            r26 = r30
            r39 = r33
            r38 = 123(0x7b, float:1.72E-43)
            r11 = r7
        L_0x0418:
            r0 = 10
            int r1 = r12.lastIndexOf(r0)
            if (r1 < 0) goto L_0x0450
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = r11.text
            r2.append(r3)
            r3 = 0
            java.lang.String r4 = r12.substring(r3, r1)
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            r11.text = r2
            goto L_0x0450
        L_0x0439:
            r40 = r1
            r34 = r3
            r39 = r4
            r41 = r6
            r36 = r11
            r37 = r15
            r43 = r22
            r27 = r28
            r26 = r30
            r0 = 10
            r38 = 123(0x7b, float:1.72E-43)
            r11 = r7
        L_0x0450:
            r1 = r40
            java.lang.String r2 = r1.cppName
            java.lang.String r3 = "::"
            int r2 = r2.lastIndexOf(r3)
            java.lang.String r3 = r9.namespace
            if (r3 == 0) goto L_0x0494
            if (r2 >= 0) goto L_0x0494
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r9.namespace
            r3.append(r4)
            java.lang.String r4 = "::"
            r3.append(r4)
            java.lang.String r4 = r1.cppName
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r1.cppName = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r9.namespace
            r3.append(r4)
            java.lang.String r4 = "::"
            r3.append(r4)
            r4 = r41
            r3.append(r4)
            java.lang.String r6 = r3.toString()
            r4 = r6
            goto L_0x0496
        L_0x0494:
            r4 = r41
        L_0x0496:
            org.bytedeco.javacpp.tools.InfoMap r3 = r8.infoMap
            java.lang.String r5 = r1.cppName
            org.bytedeco.javacpp.tools.Info r3 = r3.getFirst(r5)
            if (r3 == 0) goto L_0x04a4
            java.lang.String r5 = r3.base
            if (r5 != 0) goto L_0x04a6
        L_0x04a4:
            if (r20 != 0) goto L_0x0e0a
        L_0x04a6:
            if (r3 == 0) goto L_0x04be
            boolean r5 = r3.skip
            if (r5 == 0) goto L_0x04be
            r55 = r2
            r58 = r4
            r6 = r8
            r8 = r10
            r10 = r12
            r48 = r13
            r50 = r14
            r67 = r39
            r74 = r43
            r2 = 1
            goto L_0x0e1a
        L_0x04be:
            if (r3 == 0) goto L_0x04d7
            java.lang.String[] r5 = r3.pointerTypes
            if (r5 == 0) goto L_0x04d7
            java.lang.String[] r5 = r3.pointerTypes
            int r5 = r5.length
            if (r5 <= 0) goto L_0x04d7
            java.lang.String[] r5 = r3.pointerTypes
            r6 = 0
            r5 = r5[r6]
            r1.javaName = r5
            java.lang.String r5 = r1.javaName
            java.lang.String r27 = r9.shorten(r5)
            goto L_0x051d
        L_0x04d7:
            if (r3 != 0) goto L_0x051d
            java.lang.String r5 = r1.javaName
            int r5 = r5.length()
            if (r5 <= 0) goto L_0x04ff
            java.lang.String r5 = r9.javaName
            if (r5 == 0) goto L_0x04ff
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r9.javaName
            r5.append(r6)
            java.lang.String r6 = "."
            r5.append(r6)
            java.lang.String r6 = r1.javaName
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r1.javaName = r5
        L_0x04ff:
            org.bytedeco.javacpp.tools.InfoMap r5 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r6 = new org.bytedeco.javacpp.tools.Info
            r7 = 1
            java.lang.String[] r15 = new java.lang.String[r7]
            java.lang.String r0 = r1.cppName
            r21 = 0
            r15[r21] = r0
            r6.<init>((java.lang.String[]) r15)
            java.lang.String[] r0 = new java.lang.String[r7]
            java.lang.String r7 = r1.javaName
            r0[r21] = r7
            org.bytedeco.javacpp.tools.Info r0 = r6.pointerTypes(r0)
            r3 = r0
            r5.put(r0)
        L_0x051d:
            r0 = r27
            org.bytedeco.javacpp.tools.Type r5 = new org.bytedeco.javacpp.tools.Type
            java.lang.String r6 = "Pointer"
            r5.<init>(r6)
            java.util.Iterator r6 = r26.iterator()
        L_0x052a:
            boolean r7 = r6.hasNext()
            if (r7 == 0) goto L_0x0550
            java.lang.Object r7 = r6.next()
            org.bytedeco.javacpp.tools.Type r7 = (org.bytedeco.javacpp.tools.Type) r7
            org.bytedeco.javacpp.tools.InfoMap r15 = r8.infoMap
            r44 = r5
            java.lang.String r5 = r7.cppName
            org.bytedeco.javacpp.tools.Info r5 = r15.getFirst(r5)
            if (r5 == 0) goto L_0x054b
            boolean r15 = r5.flatten
            if (r15 != 0) goto L_0x0547
            goto L_0x054b
        L_0x0547:
            r5 = r44
            goto L_0x052a
        L_0x054b:
            r15 = r7
            r6.remove()
            goto L_0x0554
        L_0x0550:
            r44 = r5
            r15 = r44
        L_0x0554:
            java.lang.String r5 = ""
            int r7 = r26.size()
            if (r7 <= 0) goto L_0x05d6
            java.util.Iterator r7 = r26.iterator()
        L_0x0560:
            boolean r21 = r7.hasNext()
            if (r21 == 0) goto L_0x05d1
            java.lang.Object r21 = r7.next()
            r45 = r6
            r6 = r21
            org.bytedeco.javacpp.tools.Type r6 = (org.bytedeco.javacpp.tools.Type) r6
            r46 = r7
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r5)
            r47 = r5
            java.lang.String r5 = "    public "
            r7.append(r5)
            java.lang.String r5 = r6.javaName
            r7.append(r5)
            java.lang.String r5 = " as"
            r7.append(r5)
            java.lang.String r5 = r6.javaName
            r7.append(r5)
            java.lang.String r5 = "() { return as"
            r7.append(r5)
            java.lang.String r5 = r6.javaName
            r7.append(r5)
            java.lang.String r5 = "(this); }\n    @Namespace public static native @Name(\"static_cast<"
            r7.append(r5)
            java.lang.String r5 = r6.cppName
            r7.append(r5)
            java.lang.String r5 = "*>\") "
            r7.append(r5)
            java.lang.String r5 = r6.javaName
            r7.append(r5)
            java.lang.String r5 = " as"
            r7.append(r5)
            java.lang.String r5 = r6.javaName
            r7.append(r5)
            java.lang.String r5 = "("
            r7.append(r5)
            java.lang.String r5 = r1.javaName
            r7.append(r5)
            java.lang.String r5 = " pointer);\n"
            r7.append(r5)
            java.lang.String r5 = r7.toString()
            r6 = r45
            r7 = r46
            goto L_0x0560
        L_0x05d1:
            r47 = r5
            r45 = r6
            goto L_0x05d8
        L_0x05d6:
            r45 = r6
        L_0x05d8:
            java.lang.String r6 = r1.javaName
            r11.signature = r6
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r8.tokens
            r6.index = r13
            int r6 = r0.length()
            if (r6 <= 0) goto L_0x076e
            if (r25 != 0) goto L_0x076e
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r8.tokens
            org.bytedeco.javacpp.tools.Token r6 = r6.get()
            r48 = r13
            r7 = 1
            java.lang.Object[] r13 = new java.lang.Object[r7]
            java.lang.Character r7 = java.lang.Character.valueOf(r18)
            r18 = 0
            r13[r18] = r7
            boolean r6 = r6.match(r13)
            if (r6 != 0) goto L_0x060b
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r8.tokens
            r6.next()
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r8.tokens
            r6.next()
        L_0x060b:
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r8.tokens
            r6.next()
            if (r16 == 0) goto L_0x061b
            java.lang.String r6 = ""
            r11.text = r6
            r10.add((org.bytedeco.javacpp.tools.Declaration) r11)
            r6 = 1
            return r6
        L_0x061b:
            if (r3 == 0) goto L_0x0625
            java.lang.String r6 = r3.base
            if (r6 == 0) goto L_0x0625
            java.lang.String r6 = r3.base
            r15.javaName = r6
        L_0x0625:
            java.lang.String r6 = "Pointer"
            boolean r6 = r0.equals(r6)
            if (r6 == 0) goto L_0x062f
            r6 = 1
            return r6
        L_0x062f:
            java.lang.String r6 = r9.namespace
            if (r6 == 0) goto L_0x064a
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r7 = r9.namespace
            r6.append(r7)
            java.lang.String r7 = "::"
            r6.append(r7)
            r6.append(r0)
            java.lang.String r6 = r6.toString()
            goto L_0x064b
        L_0x064a:
            r6 = r0
        L_0x064b:
            java.lang.String r7 = r1.cppName
            r49 = r5
            r13 = r39
            int r5 = r13.length
            r50 = r14
            r14 = 0
        L_0x0655:
            if (r14 >= r5) goto L_0x06aa
            r51 = r5
            r5 = r13[r14]
            if (r3 == 0) goto L_0x0699
            r52 = r13
            java.lang.String[] r13 = r3.cppNames
            r18 = 0
            r13 = r13[r18]
            r53 = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r54 = r4
            java.lang.String r4 = r5.value
            r3.append(r4)
            java.lang.String r4 = " "
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            boolean r3 = r13.startsWith(r3)
            if (r3 == 0) goto L_0x069f
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r5.value
            r3.append(r4)
            java.lang.String r4 = " "
            r3.append(r4)
            r3.append(r7)
            java.lang.String r7 = r3.toString()
            goto L_0x06b0
        L_0x0699:
            r53 = r3
            r54 = r4
            r52 = r13
        L_0x069f:
            int r14 = r14 + 1
            r5 = r51
            r13 = r52
            r3 = r53
            r4 = r54
            goto L_0x0655
        L_0x06aa:
            r53 = r3
            r54 = r4
            r52 = r13
        L_0x06b0:
            boolean r3 = r6.equals(r7)
            if (r3 != 0) goto L_0x06e3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r11.text
            r3.append(r4)
            java.lang.String r4 = "@Name(\""
            r3.append(r4)
            java.lang.String r4 = r9.javaName
            if (r4 == 0) goto L_0x06d3
            if (r2 >= 0) goto L_0x06cc
            goto L_0x06d3
        L_0x06cc:
            int r4 = r2 + 2
            java.lang.String r4 = r7.substring(r4)
            goto L_0x06d4
        L_0x06d3:
            r4 = r7
        L_0x06d4:
            r3.append(r4)
            java.lang.String r4 = "\") "
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r11.text = r3
            goto L_0x070a
        L_0x06e3:
            java.lang.String r3 = r9.namespace
            if (r3 == 0) goto L_0x070a
            java.lang.String r3 = r9.javaName
            if (r3 != 0) goto L_0x070a
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r11.text
            r3.append(r4)
            java.lang.String r4 = "@Namespace(\""
            r3.append(r4)
            java.lang.String r4 = r9.namespace
            r3.append(r4)
            java.lang.String r4 = "\") "
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r11.text = r3
        L_0x070a:
            org.bytedeco.javacpp.tools.Type r3 = new org.bytedeco.javacpp.tools.Type
            r3.<init>(r0)
            r11.type = r3
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = r11.text
            r3.append(r4)
            java.lang.String r4 = "@Opaque public static class "
            r3.append(r4)
            r3.append(r0)
            java.lang.String r4 = " extends "
            r3.append(r4)
            java.lang.String r4 = r15.javaName
            r3.append(r4)
            java.lang.String r4 = " {\n    /** Empty constructor. Calls {@code super((Pointer)null)}. */\n    public "
            r3.append(r4)
            r3.append(r0)
            java.lang.String r4 = "() { super((Pointer)null); }\n    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */\n    public "
            r3.append(r4)
            r3.append(r0)
            java.lang.String r4 = "(Pointer p) { super(p); }\n}"
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r11.text = r3
            r11.type = r1
            r3 = 1
            r11.incomplete = r3
            java.lang.String r3 = r83.commentAfter()
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r3)
            java.lang.String r5 = r11.text
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r11.text = r4
            r10.spacing = r12
            r10.add((org.bytedeco.javacpp.tools.Declaration) r11)
            r4 = 0
            r10.spacing = r4
            r4 = 1
            return r4
        L_0x076e:
            r53 = r3
            r54 = r4
            r49 = r5
            r48 = r13
            r50 = r14
            r52 = r39
            r4 = 1
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.get()
            java.lang.Object[] r5 = new java.lang.Object[r4]
            java.lang.Character r4 = java.lang.Character.valueOf(r38)
            r6 = 0
            r5[r6] = r4
            boolean r3 = r3.match(r5)
            if (r3 == 0) goto L_0x0795
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r8.tokens
            r3.next()
        L_0x0795:
            java.lang.String r3 = r1.cppName
            int r3 = r3.length()
            if (r3 <= 0) goto L_0x07a8
            java.lang.String r3 = r1.cppName
            r4 = r43
            r4.namespace = r3
            r6 = r54
            r4.cppName = r6
            goto L_0x07ac
        L_0x07a8:
            r4 = r43
            r6 = r54
        L_0x07ac:
            if (r34 != 0) goto L_0x07b2
            java.lang.String r3 = r1.javaName
            r4.javaName = r3
        L_0x07b2:
            if (r53 == 0) goto L_0x07be
            r3 = r53
            boolean r5 = r3.virtualize
            if (r5 == 0) goto L_0x07c0
            r5 = 1
            r4.virtualize = r5
            goto L_0x07c0
        L_0x07be:
            r3 = r53
        L_0x07c0:
            org.bytedeco.javacpp.tools.DeclarationList r5 = new org.bytedeco.javacpp.tools.DeclarationList
            r5.<init>()
            int r7 = r50.size()
            if (r7 != 0) goto L_0x07d1
            r8.declarations(r4, r5)
            r55 = r2
            goto L_0x082d
        L_0x07d1:
            java.util.Iterator r7 = r50.iterator()
        L_0x07d5:
            boolean r13 = r7.hasNext()
            if (r13 == 0) goto L_0x082b
            java.lang.Object r13 = r7.next()
            org.bytedeco.javacpp.tools.Declarator r13 = (org.bytedeco.javacpp.tools.Declarator) r13
            org.bytedeco.javacpp.tools.Declarator r14 = r9.variable
            if (r14 == 0) goto L_0x0820
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r55 = r2
            org.bytedeco.javacpp.tools.Declarator r2 = r9.variable
            java.lang.String r2 = r2.cppName
            r14.append(r2)
            java.lang.String r2 = "."
            r14.append(r2)
            java.lang.String r2 = r13.cppName
            r14.append(r2)
            java.lang.String r2 = r14.toString()
            r13.cppName = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            org.bytedeco.javacpp.tools.Declarator r14 = r9.variable
            java.lang.String r14 = r14.javaName
            r2.append(r14)
            java.lang.String r14 = "_"
            r2.append(r14)
            java.lang.String r14 = r13.javaName
            r2.append(r14)
            java.lang.String r2 = r2.toString()
            r13.javaName = r2
            goto L_0x0822
        L_0x0820:
            r55 = r2
        L_0x0822:
            r4.variable = r13
            r8.declarations(r4, r5)
            r2 = r55
            goto L_0x07d5
        L_0x082b:
            r55 = r2
        L_0x082d:
            java.lang.String r2 = "public static "
            java.lang.String r7 = ""
            java.lang.String r13 = ""
            r14 = 1
            r21 = 0
            r22 = 0
            r24 = 0
            if (r3 == 0) goto L_0x0848
            r56 = r7
            boolean r7 = r3.purify
            if (r7 == 0) goto L_0x084a
            boolean r7 = r4.virtualize
            if (r7 != 0) goto L_0x084a
            r7 = 1
            goto L_0x084b
        L_0x0848:
            r56 = r7
        L_0x084a:
            r7 = 0
        L_0x084b:
            r27 = 1
            r28 = 0
            java.util.Iterator r30 = r5.iterator()
            r31 = r7
            r7 = r56
        L_0x0857:
            boolean r32 = r30.hasNext()
            if (r32 == 0) goto L_0x0996
            java.lang.Object r32 = r30.next()
            r57 = r13
            r13 = r32
            org.bytedeco.javacpp.tools.Declaration r13 = (org.bytedeco.javacpp.tools.Declaration) r13
            r58 = r6
            org.bytedeco.javacpp.tools.Declarator r6 = r13.declarator
            if (r6 == 0) goto L_0x08f6
            org.bytedeco.javacpp.tools.Declarator r6 = r13.declarator
            org.bytedeco.javacpp.tools.Type r6 = r6.type
            if (r6 == 0) goto L_0x08f6
            org.bytedeco.javacpp.tools.Declarator r6 = r13.declarator
            org.bytedeco.javacpp.tools.Type r6 = r6.type
            boolean r6 = r6.using
            if (r6 == 0) goto L_0x08f6
            java.lang.String r6 = r11.text
            if (r6 == 0) goto L_0x08f6
            java.lang.String r6 = r13.text
            r59 = r5
            java.lang.String r5 = "private native void allocate();"
            boolean r5 = r6.contains(r5)
            r5 = r21 | r5
            java.lang.String r6 = r13.text
            java.lang.String r10 = "private native void allocate(long"
            boolean r6 = r6.contains(r10)
            r6 = r22 | r6
            java.lang.String r10 = r13.text
            r60 = r12
            java.lang.String r12 = "private native void allocate(Pointer"
            boolean r10 = r10.contains(r12)
            r10 = r24 | r10
            if (r5 != 0) goto L_0x08a9
            if (r6 != 0) goto L_0x08a9
            if (r10 != 0) goto L_0x08a9
            r12 = 1
            goto L_0x08aa
        L_0x08a9:
            r12 = 0
        L_0x08aa:
            r12 = r12 & r14
            org.bytedeco.javacpp.tools.Declarator r14 = r13.declarator
            org.bytedeco.javacpp.tools.Type r14 = r14.type
            java.lang.String r14 = r14.cppName
            r61 = r5
            java.lang.String r5 = "::"
            int r5 = r14.lastIndexOf(r5)
            r62 = r6
            r6 = 0
            java.lang.String r5 = r14.substring(r6, r5)
            org.bytedeco.javacpp.tools.InfoMap r14 = r8.infoMap
            org.bytedeco.javacpp.tools.Info r14 = r14.getFirst(r5)
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r63 = r5
            java.lang.String r5 = r13.text
            r64 = r10
            java.lang.String[] r10 = r14.pointerTypes
            r21 = 0
            r10 = r10[r21]
            java.lang.String r5 = r5.replace(r10, r0)
            r6.append(r5)
            java.lang.String r5 = "\n"
            r6.append(r5)
            java.lang.String r5 = r6.toString()
            java.lang.String r6 = ""
            r13.text = r6
            r7 = r5
            r14 = r12
            r21 = r61
            r22 = r62
            r24 = r64
            goto L_0x0974
        L_0x08f6:
            r59 = r5
            r60 = r12
            org.bytedeco.javacpp.tools.Declarator r5 = r13.declarator
            if (r5 == 0) goto L_0x0974
            org.bytedeco.javacpp.tools.Declarator r5 = r13.declarator
            org.bytedeco.javacpp.tools.Type r5 = r5.type
            if (r5 == 0) goto L_0x0974
            org.bytedeco.javacpp.tools.Declarator r5 = r13.declarator
            org.bytedeco.javacpp.tools.Type r5 = r5.type
            boolean r5 = r5.constructor
            if (r5 == 0) goto L_0x0974
            r5 = 0
            org.bytedeco.javacpp.tools.Declarator r6 = r13.declarator
            org.bytedeco.javacpp.tools.Parameters r6 = r6.parameters
            org.bytedeco.javacpp.tools.Declarator[] r6 = r6.declarators
            int r10 = r6.length
            if (r10 == 0) goto L_0x0929
            int r10 = r6.length
            r12 = 1
            if (r10 != r12) goto L_0x092f
            r10 = 0
            r12 = r6[r10]
            org.bytedeco.javacpp.tools.Type r10 = r12.type
            java.lang.String r10 = r10.javaName
            java.lang.String r12 = "void"
            boolean r10 = r10.equals(r12)
            if (r10 == 0) goto L_0x092f
        L_0x0929:
            boolean r10 = r13.inaccessible
            if (r10 != 0) goto L_0x092f
            r10 = 1
            goto L_0x0930
        L_0x092f:
            r10 = 0
        L_0x0930:
            r10 = r21 | r10
            int r12 = r6.length
            r14 = 1
            if (r12 != r14) goto L_0x094b
            r12 = 0
            r14 = r6[r12]
            org.bytedeco.javacpp.tools.Type r12 = r14.type
            java.lang.String r12 = r12.javaName
            java.lang.String r14 = "long"
            boolean r12 = r12.equals(r14)
            if (r12 == 0) goto L_0x094b
            boolean r12 = r13.inaccessible
            if (r12 != 0) goto L_0x094b
            r12 = 1
            goto L_0x094c
        L_0x094b:
            r12 = 0
        L_0x094c:
            r12 = r22 | r12
            int r14 = r6.length
            r65 = r5
            r5 = 1
            if (r14 != r5) goto L_0x0969
            r5 = 0
            r14 = r6[r5]
            org.bytedeco.javacpp.tools.Type r5 = r14.type
            java.lang.String r5 = r5.javaName
            java.lang.String r14 = "Pointer"
            boolean r5 = r5.equals(r14)
            if (r5 == 0) goto L_0x0969
            boolean r5 = r13.inaccessible
            if (r5 != 0) goto L_0x0969
            r5 = 1
            goto L_0x096a
        L_0x0969:
            r5 = 0
        L_0x096a:
            r5 = r24 | r5
            r24 = r5
            r21 = r10
            r22 = r12
            r14 = r65
        L_0x0974:
            boolean r5 = r13.abstractMember
            r31 = r31 | r5
            boolean r5 = r13.constMember
            if (r5 == 0) goto L_0x0982
            boolean r5 = r13.abstractMember
            if (r5 == 0) goto L_0x0982
            r5 = 1
            goto L_0x0983
        L_0x0982:
            r5 = 0
        L_0x0983:
            r27 = r27 & r5
            boolean r5 = r13.variable
            r28 = r28 | r5
            r13 = r57
            r6 = r58
            r5 = r59
            r12 = r60
            r10 = r85
            goto L_0x0857
        L_0x0996:
            r59 = r5
            r58 = r6
            r60 = r12
            r57 = r13
            if (r27 == 0) goto L_0x09b5
            boolean r5 = r4.virtualize
            if (r5 == 0) goto L_0x09b5
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = "@Const "
            r5.append(r6)
            r5.append(r2)
            java.lang.String r2 = r5.toString()
        L_0x09b5:
            if (r34 != 0) goto L_0x0bb0
            java.lang.String r5 = r9.namespace
            if (r5 == 0) goto L_0x09d2
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r9.namespace
            r5.append(r6)
            java.lang.String r6 = "::"
            r5.append(r6)
            r5.append(r0)
            java.lang.String r5 = r5.toString()
            goto L_0x09d3
        L_0x09d2:
            r5 = r0
        L_0x09d3:
            java.lang.String r6 = r1.cppName
            r10 = r52
            int r12 = r10.length
            r13 = 0
        L_0x09d9:
            if (r13 >= r12) goto L_0x0a2a
            r66 = r12
            r12 = r10[r13]
            if (r3 == 0) goto L_0x0a1b
            r67 = r10
            java.lang.String[] r10 = r3.cppNames
            r29 = 0
            r10 = r10[r29]
            r68 = r1
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r8 = r12.value
            r1.append(r8)
            java.lang.String r8 = " "
            r1.append(r8)
            java.lang.String r1 = r1.toString()
            boolean r1 = r10.startsWith(r1)
            if (r1 == 0) goto L_0x0a1f
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r8 = r12.value
            r1.append(r8)
            java.lang.String r8 = " "
            r1.append(r8)
            r1.append(r6)
            java.lang.String r6 = r1.toString()
            goto L_0x0a2e
        L_0x0a1b:
            r68 = r1
            r67 = r10
        L_0x0a1f:
            int r13 = r13 + 1
            r12 = r66
            r10 = r67
            r1 = r68
            r8 = r83
            goto L_0x09d9
        L_0x0a2a:
            r68 = r1
            r67 = r10
        L_0x0a2e:
            boolean r1 = r5.equals(r6)
            if (r1 != 0) goto L_0x0a52
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r8 = r11.text
            r1.append(r8)
            java.lang.String r8 = "@Name(\""
            r1.append(r8)
            r1.append(r6)
            java.lang.String r8 = "\") "
            r1.append(r8)
            java.lang.String r1 = r1.toString()
            r11.text = r1
            goto L_0x0a79
        L_0x0a52:
            java.lang.String r1 = r9.namespace
            if (r1 == 0) goto L_0x0a79
            java.lang.String r1 = r9.javaName
            if (r1 != 0) goto L_0x0a79
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r8 = r11.text
            r1.append(r8)
            java.lang.String r8 = "@Namespace(\""
            r1.append(r8)
            java.lang.String r8 = r9.namespace
            r1.append(r8)
            java.lang.String r8 = "\") "
            r1.append(r8)
            java.lang.String r1 = r1.toString()
            r11.text = r1
        L_0x0a79:
            if (r14 == 0) goto L_0x0a7d
            if (r19 == 0) goto L_0x0a94
        L_0x0a7d:
            if (r28 == 0) goto L_0x0a94
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r8 = r11.text
            r1.append(r8)
            java.lang.String r8 = "@NoOffset "
            r1.append(r8)
            java.lang.String r1 = r1.toString()
            r11.text = r1
        L_0x0a94:
            if (r3 == 0) goto L_0x0a9e
            java.lang.String r1 = r3.base
            if (r1 == 0) goto L_0x0a9e
            java.lang.String r1 = r3.base
            r15.javaName = r1
        L_0x0a9e:
            r1 = 46
            int r1 = r0.lastIndexOf(r1)
            r8 = 1
            int r1 = r1 + r8
            java.lang.String r1 = r0.substring(r1)
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r10 = r11.text
            r8.append(r10)
            r8.append(r2)
            java.lang.String r10 = "class "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = " extends "
            r8.append(r10)
            java.lang.String r10 = r15.javaName
            r8.append(r10)
            java.lang.String r10 = " {\n    static { Loader.load(); }\n"
            r8.append(r10)
            java.lang.String r8 = r8.toString()
            r11.text = r8
            r13 = r7
            if (r14 == 0) goto L_0x0b1d
            if (r3 == 0) goto L_0x0add
            boolean r8 = r3.purify
            if (r8 != 0) goto L_0x0b1d
        L_0x0add:
            if (r31 == 0) goto L_0x0ae3
            boolean r8 = r4.virtualize
            if (r8 == 0) goto L_0x0b1d
        L_0x0ae3:
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r8.append(r7)
            java.lang.String r10 = "    /** Default native constructor. */\n    public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = "() { super((Pointer)null); allocate(); }\n    /** Native array allocator. Access with {@link Pointer#position(long)}. */\n    public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = "(long size) { super((Pointer)null); allocateArray(size); }\n    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */\n    public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = "(Pointer p) { super(p); }\n    private native void allocate();\n    private native void allocateArray(long size);\n    @Override public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = " position(long position) {\n        return ("
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = ")super.position(position);\n    }\n"
            r8.append(r10)
            java.lang.String r7 = r8.toString()
            goto L_0x0b71
        L_0x0b1d:
            if (r24 != 0) goto L_0x0b38
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r8.append(r7)
            java.lang.String r10 = "    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */\n    public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = "(Pointer p) { super(p); }\n"
            r8.append(r10)
            java.lang.String r7 = r8.toString()
        L_0x0b38:
            if (r21 == 0) goto L_0x0b71
            if (r3 == 0) goto L_0x0b40
            boolean r8 = r3.purify
            if (r8 != 0) goto L_0x0b71
        L_0x0b40:
            if (r31 == 0) goto L_0x0b46
            boolean r8 = r4.virtualize
            if (r8 == 0) goto L_0x0b71
        L_0x0b46:
            if (r22 != 0) goto L_0x0b71
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r8.append(r7)
            java.lang.String r10 = "    /** Native array allocator. Access with {@link Pointer#position(long)}. */\n    public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = "(long size) { super((Pointer)null); allocateArray(size); }\n    private native void allocateArray(long size);\n    @Override public "
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = " position(long position) {\n        return ("
            r8.append(r10)
            r8.append(r1)
            java.lang.String r10 = ")super.position(position);\n    }\n"
            r8.append(r10)
            java.lang.String r7 = r8.toString()
        L_0x0b71:
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            java.lang.String r10 = r11.text
            r8.append(r10)
            r8.append(r7)
            java.lang.String r8 = r8.toString()
            r11.text = r8
            r10 = r60
            r8 = r85
            r8.spacing = r10
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            r69 = r0
            java.lang.String r0 = r11.text
            r12.append(r0)
            r0 = r49
            r12.append(r0)
            r70 = r0
            java.lang.String r0 = "\n"
            r12.append(r0)
            java.lang.String r0 = r12.toString()
            java.lang.String r0 = r8.rescan(r0)
            r11.text = r0
            r0 = 0
            r8.spacing = r0
            goto L_0x0bbe
        L_0x0bb0:
            r69 = r0
            r68 = r1
            r70 = r49
            r67 = r52
            r10 = r60
            r8 = r85
            r13 = r57
        L_0x0bbe:
            java.util.Iterator r0 = r26.iterator()
        L_0x0bc2:
            boolean r1 = r0.hasNext()
            if (r1 == 0) goto L_0x0c94
            java.lang.Object r1 = r0.next()
            org.bytedeco.javacpp.tools.Type r1 = (org.bytedeco.javacpp.tools.Type) r1
            r6 = r83
            org.bytedeco.javacpp.tools.InfoMap r12 = r6.infoMap
            java.lang.String r5 = r1.cppName
            org.bytedeco.javacpp.tools.Info r5 = r12.getFirst(r5)
            if (r5 == 0) goto L_0x0c7c
            boolean r12 = r5.flatten
            if (r12 == 0) goto L_0x0c7c
            java.lang.String r12 = r5.javaText
            if (r12 == 0) goto L_0x0c7c
            java.lang.String r12 = r5.javaText
            r71 = r0
            r0 = 123(0x7b, float:1.72E-43)
            int r30 = r12.indexOf(r0)
            r0 = r30
            r30 = 0
        L_0x0bf0:
            r72 = r30
            r73 = r2
            r74 = r4
            r2 = r72
            r4 = 2
            if (r2 >= r4) goto L_0x0c1c
            char r4 = r12.charAt(r0)
            r75 = r5
            r5 = 10
            if (r4 != r5) goto L_0x0c0a
            int r72 = r2 + 1
            r30 = r72
            goto L_0x0c13
        L_0x0c0a:
            boolean r30 = java.lang.Character.isWhitespace(r4)
            if (r30 != 0) goto L_0x0c11
            r2 = 0
        L_0x0c11:
            r30 = r2
        L_0x0c13:
            int r0 = r0 + 1
            r2 = r73
            r4 = r74
            r5 = r75
            goto L_0x0bf0
        L_0x0c1c:
            r75 = r5
            r5 = 10
            r2 = 125(0x7d, float:1.75E-43)
            int r2 = r12.lastIndexOf(r2)
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r11.text
            r4.append(r5)
            java.lang.String r5 = r12.substring(r0, r2)
            r76 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r77 = r2
            java.lang.String r2 = "(\\s+)"
            r0.append(r2)
            java.lang.String r2 = r1.javaName
            r0.append(r2)
            java.lang.String r2 = "(\\s+)"
            r0.append(r2)
            java.lang.String r0 = r0.toString()
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r78 = r1
            java.lang.String r1 = "$1"
            r2.append(r1)
            r79 = r7
            r1 = r68
            java.lang.String r7 = r1.javaName
            r2.append(r7)
            java.lang.String r7 = "$2"
            r2.append(r7)
            java.lang.String r2 = r2.toString()
            java.lang.String r0 = r5.replaceAll(r0, r2)
            r4.append(r0)
            java.lang.String r0 = r4.toString()
            r11.text = r0
            goto L_0x0c86
        L_0x0c7c:
            r71 = r0
            r73 = r2
            r74 = r4
            r79 = r7
            r1 = r68
        L_0x0c86:
            r68 = r1
            r0 = r71
            r2 = r73
            r4 = r74
            r7 = r79
            r38 = 123(0x7b, float:1.72E-43)
            goto L_0x0bc2
        L_0x0c94:
            r73 = r2
            r74 = r4
            r79 = r7
            r1 = r68
            r6 = r83
            java.util.Iterator r0 = r59.iterator()
        L_0x0ca2:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x0d00
            java.lang.Object r2 = r0.next()
            org.bytedeco.javacpp.tools.Declaration r2 = (org.bytedeco.javacpp.tools.Declaration) r2
            boolean r4 = r2.inaccessible
            if (r4 != 0) goto L_0x0cff
            org.bytedeco.javacpp.tools.Declarator r4 = r2.declarator
            if (r4 == 0) goto L_0x0cc6
            org.bytedeco.javacpp.tools.Declarator r4 = r2.declarator
            org.bytedeco.javacpp.tools.Type r4 = r4.type
            if (r4 == 0) goto L_0x0cc6
            org.bytedeco.javacpp.tools.Declarator r4 = r2.declarator
            org.bytedeco.javacpp.tools.Type r4 = r4.type
            boolean r4 = r4.constructor
            if (r4 == 0) goto L_0x0cc6
            if (r31 != 0) goto L_0x0cff
        L_0x0cc6:
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r11.text
            r4.append(r5)
            java.lang.String r5 = r2.text
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r11.text = r4
            org.bytedeco.javacpp.tools.Declarator r4 = r2.declarator
            if (r4 == 0) goto L_0x0cff
            org.bytedeco.javacpp.tools.Declarator r4 = r2.declarator
            org.bytedeco.javacpp.tools.Type r4 = r4.type
            if (r4 == 0) goto L_0x0cff
            org.bytedeco.javacpp.tools.Declarator r4 = r2.declarator
            org.bytedeco.javacpp.tools.Type r4 = r4.type
            boolean r4 = r4.constructor
            if (r4 == 0) goto L_0x0cff
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r13)
            java.lang.String r5 = r2.text
            r4.append(r5)
            java.lang.String r2 = r4.toString()
            r13 = r2
        L_0x0cff:
            goto L_0x0ca2
        L_0x0d00:
            r0 = r58
            r2 = 60
            int r2 = r0.lastIndexOf(r2)
            if (r2 < 0) goto L_0x0d0f
            r4 = 0
            java.lang.String r0 = r0.substring(r4, r2)
        L_0x0d0f:
            java.lang.String r4 = "::"
            int r4 = r0.lastIndexOf(r4)
            if (r4 < 0) goto L_0x0d1d
            int r5 = r4 + 2
            java.lang.String r0 = r0.substring(r5)
        L_0x0d1d:
            org.bytedeco.javacpp.tools.InfoMap r5 = r6.infoMap
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r12 = r1.cppName
            r7.append(r12)
            java.lang.String r12 = "::"
            r7.append(r12)
            r7.append(r0)
            java.lang.String r7 = r7.toString()
            org.bytedeco.javacpp.tools.Info r5 = r5.getFirst(r7)
            org.bytedeco.javacpp.tools.TemplateMap r7 = r9.templateMap
            if (r7 == 0) goto L_0x0d4d
            org.bytedeco.javacpp.tools.TemplateMap r7 = r9.templateMap
            boolean r7 = r7.full()
            if (r7 == 0) goto L_0x0d46
            goto L_0x0d4d
        L_0x0d46:
            r80 = r2
            r81 = r4
            r82 = r5
            goto L_0x0d86
        L_0x0d4d:
            if (r5 != 0) goto L_0x0d80
            org.bytedeco.javacpp.tools.InfoMap r7 = r6.infoMap
            org.bytedeco.javacpp.tools.Info r12 = new org.bytedeco.javacpp.tools.Info
            r80 = r2
            r81 = r4
            r2 = 1
            java.lang.String[] r4 = new java.lang.String[r2]
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r82 = r5
            java.lang.String r5 = r1.cppName
            r2.append(r5)
            java.lang.String r5 = "::"
            r2.append(r5)
            r2.append(r0)
            java.lang.String r2 = r2.toString()
            r5 = 0
            r4[r5] = r2
            r12.<init>((java.lang.String[]) r4)
            org.bytedeco.javacpp.tools.Info r2 = r12.javaText(r13)
            r7.put(r2)
            goto L_0x0d86
        L_0x0d80:
            r80 = r2
            r81 = r4
            r82 = r5
        L_0x0d86:
            if (r34 != 0) goto L_0x0da8
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r4 = r11.text
            r2.append(r4)
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r6.tokens
            org.bytedeco.javacpp.tools.Token r4 = r4.get()
            java.lang.String r4 = r4.spacing
            r2.append(r4)
            r4 = 125(0x7d, float:1.75E-43)
            r2.append(r4)
            java.lang.String r2 = r2.toString()
            r11.text = r2
        L_0x0da8:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r6.tokens
            org.bytedeco.javacpp.tools.Token r2 = r2.next()
        L_0x0dae:
            r4 = 1
            java.lang.Object[] r5 = new java.lang.Object[r4]
            org.bytedeco.javacpp.tools.Token r7 = org.bytedeco.javacpp.tools.Token.EOF
            r12 = 0
            r5[r12] = r7
            boolean r5 = r2.match(r5)
            if (r5 != 0) goto L_0x0de7
            java.lang.Object[] r5 = new java.lang.Object[r4]
            java.lang.Character r4 = java.lang.Character.valueOf(r18)
            r5[r12] = r4
            boolean r4 = r2.match(r5)
            if (r4 == 0) goto L_0x0de0
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = r11.text
            r4.append(r5)
            java.lang.String r5 = r2.spacing
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r11.text = r4
            goto L_0x0de7
        L_0x0de0:
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r6.tokens
            org.bytedeco.javacpp.tools.Token r2 = r4.next()
            goto L_0x0dae
        L_0x0de7:
            org.bytedeco.javacpp.tools.TokenIndexer r2 = r6.tokens
            r2.next()
            r11.type = r1
            if (r3 == 0) goto L_0x0dfb
            java.lang.String r2 = r3.javaText
            if (r2 == 0) goto L_0x0dfb
            java.lang.String r2 = r3.javaText
            r11.text = r2
            r11.signature = r2
            goto L_0x0e05
        L_0x0dfb:
            if (r3 == 0) goto L_0x0e05
            boolean r2 = r3.flatten
            if (r2 == 0) goto L_0x0e05
            java.lang.String r2 = r11.text
            r3.javaText = r2
        L_0x0e05:
            r8.add((org.bytedeco.javacpp.tools.Declaration) r11)
            r2 = 1
            return r2
        L_0x0e0a:
            r55 = r2
            r58 = r4
            r6 = r8
            r8 = r10
            r10 = r12
            r48 = r13
            r50 = r14
            r67 = r39
            r74 = r43
            r2 = 1
        L_0x0e1a:
            java.lang.String r0 = ""
            r11.text = r0
            r8.add((org.bytedeco.javacpp.tools.Declaration) r11)
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.group(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):boolean");
    }

    /* JADX WARNING: type inference failed for: r6v4 */
    /* JADX WARNING: type inference failed for: r25v2, types: [boolean] */
    /* JADX WARNING: type inference failed for: r25v5 */
    /* JADX WARNING: type inference failed for: r25v6 */
    /* JADX WARNING: type inference failed for: r6v64 */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Incorrect type for immutable var: ssa=int, code=?, for r6v60, types: [boolean, int] */
    /* JADX WARNING: Removed duplicated region for block: B:130:0x0501  */
    /* JADX WARNING: Removed duplicated region for block: B:137:0x0527  */
    /* JADX WARNING: Removed duplicated region for block: B:138:0x0540  */
    /* JADX WARNING: Removed duplicated region for block: B:141:0x0574  */
    /* JADX WARNING: Removed duplicated region for block: B:144:0x059d  */
    /* JADX WARNING: Removed duplicated region for block: B:148:0x05ad  */
    /* JADX WARNING: Removed duplicated region for block: B:160:0x0600  */
    /* JADX WARNING: Removed duplicated region for block: B:161:0x0605  */
    /* JADX WARNING: Removed duplicated region for block: B:164:0x0669  */
    /* JADX WARNING: Removed duplicated region for block: B:171:0x06bb  */
    /* JADX WARNING: Removed duplicated region for block: B:176:0x06e6  */
    /* JADX WARNING: Removed duplicated region for block: B:178:0x06f3  */
    /* JADX WARNING: Removed duplicated region for block: B:184:0x0779  */
    /* JADX WARNING: Removed duplicated region for block: B:188:0x07bd  */
    /* JADX WARNING: Removed duplicated region for block: B:208:0x0875  */
    /* JADX WARNING: Removed duplicated region for block: B:209:0x088c  */
    /* JADX WARNING: Removed duplicated region for block: B:212:0x0895  */
    /* JADX WARNING: Removed duplicated region for block: B:213:0x0898  */
    /* JADX WARNING: Removed duplicated region for block: B:217:0x08ac A[LOOP:6: B:215:0x08a6->B:217:0x08ac, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:221:0x08c1  */
    /* JADX WARNING: Removed duplicated region for block: B:222:0x08d3  */
    /* JADX WARNING: Removed duplicated region for block: B:237:0x0909  */
    /* JADX WARNING: Removed duplicated region for block: B:238:0x0922  */
    /* JADX WARNING: Removed duplicated region for block: B:241:0x092d  */
    /* JADX WARNING: Removed duplicated region for block: B:242:0x094b  */
    /* JADX WARNING: Removed duplicated region for block: B:251:0x09ba  */
    /* JADX WARNING: Removed duplicated region for block: B:252:0x09c0  */
    /* JADX WARNING: Removed duplicated region for block: B:256:0x0a60 A[LOOP:7: B:254:0x0a5b->B:256:0x0a60, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:260:0x0aa4 A[LOOP:8: B:258:0x0a9f->B:260:0x0aa4, LOOP_END] */
    /* JADX WARNING: Removed duplicated region for block: B:280:0x086d A[EDGE_INSN: B:280:0x086d->B:206:0x086d ?: BREAK  , SYNTHETIC] */
    /* JADX WARNING: Removed duplicated region for block: B:84:0x0357  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean enumeration(org.bytedeco.javacpp.tools.Context r76, org.bytedeco.javacpp.tools.DeclarationList r77) throws org.bytedeco.javacpp.tools.ParserException {
        /*
            r75 = this;
            r1 = r75
            r2 = r76
            r3 = r77
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            int r4 = r0.index
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            java.lang.String r5 = r0.spacing
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get()
            r6 = 1
            java.lang.Object[] r7 = new java.lang.Object[r6]
            org.bytedeco.javacpp.tools.Token r8 = org.bytedeco.javacpp.tools.Token.TYPEDEF
            r9 = 0
            r7[r9] = r8
            boolean r7 = r0.match(r7)
            r0 = 0
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            org.bytedeco.javacpp.tools.Token r8 = r8.get()
        L_0x002b:
            java.lang.Object[] r10 = new java.lang.Object[r6]
            org.bytedeco.javacpp.tools.Token r11 = org.bytedeco.javacpp.tools.Token.EOF
            r10[r9] = r11
            boolean r10 = r8.match(r10)
            r11 = 5
            if (r10 != 0) goto L_0x005c
            java.lang.Object[] r10 = new java.lang.Object[r6]
            org.bytedeco.javacpp.tools.Token r12 = org.bytedeco.javacpp.tools.Token.ENUM
            r10[r9] = r12
            boolean r10 = r8.match(r10)
            if (r10 == 0) goto L_0x0046
            r0 = 1
            goto L_0x005c
        L_0x0046:
            java.lang.Object[] r10 = new java.lang.Object[r6]
            java.lang.Integer r12 = java.lang.Integer.valueOf(r11)
            r10[r9] = r12
            boolean r10 = r8.match(r10)
            if (r10 != 0) goto L_0x0055
            goto L_0x005c
        L_0x0055:
            org.bytedeco.javacpp.tools.TokenIndexer r10 = r1.tokens
            org.bytedeco.javacpp.tools.Token r8 = r10.next()
            goto L_0x002b
        L_0x005c:
            r8 = r0
            if (r8 != 0) goto L_0x0064
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.index = r4
            return r9
        L_0x0064:
            r0 = 0
            java.lang.String r10 = "enum"
            org.bytedeco.javacpp.tools.TokenIndexer r12 = r1.tokens
            org.bytedeco.javacpp.tools.Token r12 = r12.get(r6)
            r13 = 2
            java.lang.Object[] r14 = new java.lang.Object[r13]
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.CLASS
            r14[r9] = r15
            org.bytedeco.javacpp.tools.Token r15 = org.bytedeco.javacpp.tools.Token.STRUCT
            r14[r6] = r15
            boolean r12 = r12.match(r14)
            if (r12 == 0) goto L_0x0099
            r0 = 1
            java.lang.StringBuilder r12 = new java.lang.StringBuilder
            r12.<init>()
            r12.append(r10)
            java.lang.String r14 = " "
            r12.append(r14)
            org.bytedeco.javacpp.tools.TokenIndexer r14 = r1.tokens
            org.bytedeco.javacpp.tools.Token r14 = r14.next()
            r12.append(r14)
            java.lang.String r10 = r12.toString()
        L_0x0099:
            r12 = r0
            r14 = 123(0x7b, float:1.72E-43)
            if (r7 == 0) goto L_0x00cb
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r6)
            java.lang.Object[] r15 = new java.lang.Object[r6]
            java.lang.Character r16 = java.lang.Character.valueOf(r14)
            r15[r9] = r16
            boolean r0 = r0.match(r15)
            if (r0 != 0) goto L_0x00cb
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.get(r13)
            java.lang.Object[] r15 = new java.lang.Object[r6]
            java.lang.Integer r16 = java.lang.Integer.valueOf(r11)
            r15[r9] = r16
            boolean r0 = r0.match(r15)
            if (r0 == 0) goto L_0x00cb
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            r0.next()
        L_0x00cb:
            r0 = 0
            r15 = 0
            java.lang.String r16 = "int"
            java.lang.String r13 = "int"
            java.lang.String r18 = ""
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r14 = "public static final "
            r6.append(r14)
            r6.append(r13)
            java.lang.String r6 = r6.toString()
            java.lang.String r14 = ""
            java.lang.String r21 = ""
            java.lang.String r22 = ""
            java.util.HashMap r23 = new java.util.HashMap
            r23.<init>()
            r24 = r23
            java.lang.String r23 = ""
            java.lang.String r25 = ""
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r9.next()
            r27 = r6
            r6 = 4
            r28 = r0
            java.lang.Object[] r0 = new java.lang.Object[r6]
            java.lang.Integer r29 = java.lang.Integer.valueOf(r11)
            r26 = 0
            r0[r26] = r29
            r20 = 123(0x7b, float:1.72E-43)
            java.lang.Character r29 = java.lang.Character.valueOf(r20)
            r6 = 1
            r0[r6] = r29
            r29 = 58
            java.lang.Character r19 = java.lang.Character.valueOf(r29)
            r17 = 2
            r0[r17] = r19
            r30 = 59
            java.lang.Character r19 = java.lang.Character.valueOf(r30)
            r31 = 3
            r0[r31] = r19
            org.bytedeco.javacpp.tools.Token r0 = r9.expect(r0)
            java.lang.Object[] r9 = new java.lang.Object[r6]
            java.lang.Integer r19 = java.lang.Integer.valueOf(r11)
            r26 = 0
            r9[r26] = r19
            boolean r9 = r0.match(r9)
            if (r9 == 0) goto L_0x0171
        L_0x013b:
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r9.get(r6)
            r32 = r8
            java.lang.Object[] r8 = new java.lang.Object[r6]
            java.lang.Integer r19 = java.lang.Integer.valueOf(r11)
            r8[r26] = r19
            boolean r8 = r9.match(r8)
            if (r8 == 0) goto L_0x0168
            org.bytedeco.javacpp.tools.Attribute r8 = r1.attribute(r6)
            if (r8 == 0) goto L_0x0168
            boolean r6 = r8.annotation
            if (r6 == 0) goto L_0x0168
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r6.get()
            r8 = r32
            r6 = 1
            r26 = 0
            goto L_0x013b
        L_0x0168:
            java.lang.String r6 = r0.value
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r8.next()
            goto L_0x0175
        L_0x0171:
            r32 = r8
            r6 = r25
        L_0x0175:
            r8 = 1
            java.lang.Object[] r9 = new java.lang.Object[r8]
            java.lang.Character r8 = java.lang.Character.valueOf(r29)
            r25 = 0
            r9[r25] = r8
            boolean r8 = r0.match(r9)
            if (r8 == 0) goto L_0x01b2
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r8.next()
            org.bytedeco.javacpp.tools.Type r8 = r75.type(r76)
            java.lang.String r9 = r8.cppName
            java.lang.String r13 = r8.javaName
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r33 = r0
            java.lang.String r0 = "public static final "
            r11.append(r0)
            r11.append(r13)
            java.lang.String r0 = r11.toString()
            org.bytedeco.javacpp.tools.TokenIndexer r11 = r1.tokens
            org.bytedeco.javacpp.tools.Token r8 = r11.get()
            r27 = r0
            r0 = r8
            r16 = r9
        L_0x01b2:
            if (r7 != 0) goto L_0x01e1
            r11 = 1
            java.lang.Object[] r8 = new java.lang.Object[r11]
            java.lang.Character r19 = java.lang.Character.valueOf(r30)
            r25 = 0
            r8[r25] = r19
            boolean r8 = r0.match(r8)
            if (r8 == 0) goto L_0x01e4
            r35 = r4
            r45 = r5
            r38 = r6
            r36 = r7
            r44 = r10
            r40 = r12
            r8 = r21
            r6 = r22
            r11 = r23
            r4 = r24
            r74 = r18
            r18 = r14
            r14 = r74
            goto L_0x0777
        L_0x01e1:
            r11 = 1
            r25 = 0
        L_0x01e4:
            java.lang.Object[] r8 = new java.lang.Object[r11]
            r11 = 123(0x7b, float:1.72E-43)
            java.lang.Character r26 = java.lang.Character.valueOf(r11)
            r8[r25] = r26
            boolean r8 = r0.match(r8)
            if (r8 != 0) goto L_0x01f9
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            r8.index = r4
            return r25
        L_0x01f9:
            org.bytedeco.javacpp.tools.TokenIndexer r8 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r8.next()
            r8 = r21
            r34 = r22
            r11 = r23
            r74 = r18
            r18 = r14
            r14 = r74
        L_0x020b:
            r35 = r4
            r9 = 2
            java.lang.Object[] r4 = new java.lang.Object[r9]
            org.bytedeco.javacpp.tools.Token r9 = org.bytedeco.javacpp.tools.Token.EOF
            r4[r25] = r9
            r9 = 125(0x7d, float:1.75E-43)
            java.lang.Character r21 = java.lang.Character.valueOf(r9)
            r19 = 1
            r4[r19] = r21
            boolean r4 = r0.match(r4)
            if (r4 != 0) goto L_0x0767
            java.lang.String r4 = r75.commentBefore()
            boolean r21 = r75.macro(r76, r77)
            if (r21 == 0) goto L_0x029c
            int r9 = r77.size()
            int r9 = r9 + -1
            java.lang.Object r9 = r3.remove(r9)
            org.bytedeco.javacpp.tools.Declaration r9 = (org.bytedeco.javacpp.tools.Declaration) r9
            r36 = r7
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            r7.append(r11)
            r7.append(r4)
            r37 = r15
            java.lang.String r15 = r9.text
            r7.append(r15)
            java.lang.String r7 = r7.toString()
            java.lang.String r11 = ","
            boolean r11 = r14.equals(r11)
            if (r11 == 0) goto L_0x028d
            java.lang.String r11 = r9.text
            java.lang.String r11 = r11.trim()
            java.lang.String r15 = "//"
            boolean r11 = r11.startsWith(r15)
            if (r11 != 0) goto L_0x028d
            java.lang.String r11 = ";"
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            java.lang.String r15 = "public static final "
            r14.append(r15)
            r14.append(r13)
            java.lang.String r14 = r14.toString()
            r45 = r5
            r38 = r6
            r44 = r10
            r40 = r12
            r27 = r14
            r4 = r24
            r15 = r37
            r14 = r11
            r11 = r7
            goto L_0x074d
        L_0x028d:
            r45 = r5
            r38 = r6
            r11 = r7
            r44 = r10
            r40 = r12
            r4 = r24
        L_0x0298:
            r15 = r37
            goto L_0x074d
        L_0x029c:
            r36 = r7
            r37 = r15
            org.bytedeco.javacpp.tools.TokenIndexer r7 = r1.tokens
            org.bytedeco.javacpp.tools.Token r7 = r7.get()
            java.lang.String r15 = r7.value
            if (r15 == 0) goto L_0x0730
            int r21 = r15.length()
            if (r21 != 0) goto L_0x02c0
            r47 = r4
            r45 = r5
            r38 = r6
            r44 = r10
            r40 = r12
            r4 = r24
            r6 = r34
            goto L_0x073e
        L_0x02c0:
            r21 = r15
            if (r12 == 0) goto L_0x02db
            java.lang.StringBuilder r9 = new java.lang.StringBuilder
            r9.<init>()
            r9.append(r6)
            r38 = r6
            java.lang.String r6 = "::"
            r9.append(r6)
            r9.append(r15)
            java.lang.String r15 = r9.toString()
            goto L_0x02dd
        L_0x02db:
            r38 = r6
        L_0x02dd:
            java.lang.String r6 = r2.namespace
            if (r6 == 0) goto L_0x02f7
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            java.lang.String r9 = r2.namespace
            r6.append(r9)
            java.lang.String r9 = "::"
            r6.append(r9)
            r6.append(r15)
            java.lang.String r15 = r6.toString()
        L_0x02f7:
            org.bytedeco.javacpp.tools.InfoMap r6 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r6 = r6.getFirst(r15)
            if (r6 == 0) goto L_0x0311
            java.lang.String[] r9 = r6.javaNames
            if (r9 == 0) goto L_0x0311
            java.lang.String[] r9 = r6.javaNames
            int r9 = r9.length
            if (r9 <= 0) goto L_0x0311
            java.lang.String[] r9 = r6.javaNames
            r22 = 0
            r21 = r9[r22]
            r40 = r12
            goto L_0x032f
        L_0x0311:
            r22 = 0
            if (r6 != 0) goto L_0x0332
            org.bytedeco.javacpp.tools.InfoMap r9 = r1.infoMap
            r39 = r6
            org.bytedeco.javacpp.tools.Info r6 = new org.bytedeco.javacpp.tools.Info
            r40 = r12
            r12 = 1
            java.lang.String[] r3 = new java.lang.String[r12]
            r3[r22] = r15
            r6.<init>((java.lang.String[]) r3)
            java.lang.String r3 = ""
            org.bytedeco.javacpp.tools.Info r3 = r6.cppText(r3)
            r6 = r3
            r9.put(r3)
        L_0x032f:
            r3 = r21
            goto L_0x0338
        L_0x0332:
            r39 = r6
            r40 = r12
            r3 = r21
        L_0x0338:
            java.lang.String r9 = ""
            org.bytedeco.javacpp.tools.TokenIndexer r12 = r1.tokens
            org.bytedeco.javacpp.tools.Token r12 = r12.next()
            r41 = r9
            r42 = r15
            r9 = 1
            java.lang.Object[] r15 = new java.lang.Object[r9]
            r9 = 61
            java.lang.Character r9 = java.lang.Character.valueOf(r9)
            r21 = 0
            r15[r21] = r9
            boolean r9 = r12.match(r15)
            if (r9 == 0) goto L_0x0501
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r9.get()
            java.lang.String r9 = r9.spacing
            int r15 = r9.length()
            if (r15 <= 0) goto L_0x0373
            r15 = 0
            char r12 = r9.charAt(r15)
            r15 = 32
            if (r12 != r15) goto L_0x0373
            r12 = 1
            java.lang.String r9 = r9.substring(r12)
        L_0x0373:
            java.lang.String r12 = ""
            r15 = 0
            org.bytedeco.javacpp.tools.Token r18 = new org.bytedeco.javacpp.tools.Token
            r18.<init>()
            r21 = 1
            r43 = r9
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.next()
            r9 = r0
            r44 = r10
            r10 = r18
        L_0x038a:
            r45 = r5
            r5 = 4
            java.lang.Object[] r0 = new java.lang.Object[r5]
            org.bytedeco.javacpp.tools.Token r18 = org.bytedeco.javacpp.tools.Token.EOF
            r22 = 0
            r0[r22] = r18
            r18 = 35
            java.lang.Character r18 = java.lang.Character.valueOf(r18)
            r19 = 1
            r0[r19] = r18
            r18 = 44
            java.lang.Character r22 = java.lang.Character.valueOf(r18)
            r17 = 2
            r0[r17] = r22
            r18 = 125(0x7d, float:1.75E-43)
            java.lang.Character r22 = java.lang.Character.valueOf(r18)
            r0[r31] = r22
            boolean r0 = r9.match(r0)
            if (r0 == 0) goto L_0x045a
            if (r15 <= 0) goto L_0x03bb
            goto L_0x045a
        L_0x03bb:
            java.lang.String r0 = r12.trim()     // Catch:{ NumberFormatException -> 0x03da }
            int r0 = java.lang.Integer.parseInt(r0)     // Catch:{ NumberFormatException -> 0x03da }
            r28 = r0
            java.lang.String r0 = ""
            r18 = r0
        L_0x03ca:
            r12 = r18
            r0 = r27
            r10 = r28
            r15 = r37
            r5 = 123(0x7b, float:1.72E-43)
            r18 = r9
            r9 = r43
            goto L_0x0513
        L_0x03da:
            r0 = move-exception
            r28 = 0
            if (r21 == 0) goto L_0x0408
            java.lang.String r12 = r1.translate(r12)
            int r18 = r12.length()
            if (r18 <= 0) goto L_0x03fa
            r46 = r0
            r5 = 0
            char r0 = r12.charAt(r5)
            r5 = 32
            if (r0 != r5) goto L_0x03fa
            r5 = 1
            java.lang.String r18 = r12.substring(r5)
            goto L_0x03ca
        L_0x03fa:
            r18 = r9
            r0 = r27
            r10 = r28
            r15 = r37
            r9 = r43
            r5 = 123(0x7b, float:1.72E-43)
            goto L_0x0513
        L_0x0408:
            r46 = r0
            java.lang.String r0 = ","
            boolean r0 = r14.equals(r0)
            if (r0 == 0) goto L_0x0415
            java.lang.String r0 = ";"
            r14 = r0
        L_0x0415:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r11)
            java.lang.String r5 = "\npublic static native @MemberGetter "
            r0.append(r5)
            r0.append(r13)
            java.lang.String r5 = " "
            r0.append(r5)
            r0.append(r3)
            java.lang.String r5 = "();\n"
            r0.append(r5)
            java.lang.String r11 = r0.toString()
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r5 = "public static final "
            r0.append(r5)
            r0.append(r13)
            java.lang.String r27 = r0.toString()
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r3)
            java.lang.String r5 = "()"
            r0.append(r5)
            java.lang.String r18 = r0.toString()
            goto L_0x03ca
        L_0x045a:
            r5 = 1
            java.lang.Object[] r0 = new java.lang.Object[r5]
            java.lang.Integer r18 = java.lang.Integer.valueOf(r5)
            r5 = 0
            r0[r5] = r18
            boolean r0 = r9.match(r0)
            if (r0 == 0) goto L_0x0477
            java.lang.String r0 = r9.value
            java.lang.String r5 = "L"
            boolean r0 = r0.endsWith(r5)
            if (r0 == 0) goto L_0x0477
            r0 = 1
            r37 = r0
        L_0x0477:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r12)
            java.lang.String r5 = r9.spacing
            r0.append(r5)
            r0.append(r9)
            java.lang.String r12 = r0.toString()
            r5 = 1
            java.lang.Object[] r0 = new java.lang.Object[r5]
            r18 = 40
            java.lang.Character r19 = java.lang.Character.valueOf(r18)
            r22 = 0
            r0[r22] = r19
            boolean r0 = r9.match(r0)
            if (r0 == 0) goto L_0x04a1
            int r15 = r15 + 1
            goto L_0x04b3
        L_0x04a1:
            java.lang.Object[] r0 = new java.lang.Object[r5]
            r5 = 41
            java.lang.Character r5 = java.lang.Character.valueOf(r5)
            r0[r22] = r5
            boolean r0 = r9.match(r0)
            if (r0 == 0) goto L_0x04b3
            int r15 = r15 + -1
        L_0x04b3:
            r5 = 1
            java.lang.Object[] r0 = new java.lang.Object[r5]
            r19 = 5
            java.lang.Integer r23 = java.lang.Integer.valueOf(r19)
            r0[r22] = r23
            boolean r0 = r10.match(r0)
            if (r0 == 0) goto L_0x04d8
            java.lang.Object[] r0 = new java.lang.Object[r5]
            java.lang.Character r5 = java.lang.Character.valueOf(r18)
            r0[r22] = r5
            boolean r0 = r9.match(r0)
            if (r0 != 0) goto L_0x04d3
            goto L_0x04d8
        L_0x04d3:
            r5 = 123(0x7b, float:1.72E-43)
            r20 = 125(0x7d, float:1.75E-43)
            goto L_0x04f3
        L_0x04d8:
            r5 = 2
            java.lang.Object[] r0 = new java.lang.Object[r5]
            r5 = 123(0x7b, float:1.72E-43)
            java.lang.Character r18 = java.lang.Character.valueOf(r5)
            r0[r22] = r18
            r20 = 125(0x7d, float:1.75E-43)
            java.lang.Character r18 = java.lang.Character.valueOf(r20)
            r19 = 1
            r0[r19] = r18
            boolean r0 = r9.match(r0)
            if (r0 == 0) goto L_0x04f6
        L_0x04f3:
            r0 = 0
            r21 = r0
        L_0x04f6:
            r10 = r9
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r9 = r0.next()
            r5 = r45
            goto L_0x038a
        L_0x0501:
            r45 = r5
            r44 = r10
            r5 = 123(0x7b, float:1.72E-43)
            r12 = r18
            r10 = r28
            r15 = r37
            r9 = r41
            r18 = r0
            r0 = r27
        L_0x0513:
            int r20 = r11.length()
            if (r20 <= 0) goto L_0x0540
            java.lang.String r5 = "\n"
            boolean r5 = r11.endsWith(r5)
            if (r5 != 0) goto L_0x0540
            int r5 = r0.length()
            if (r5 <= 0) goto L_0x0540
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r11)
            r5.append(r4)
            r47 = r4
            java.lang.String r4 = "\n"
            r5.append(r4)
            java.lang.String r11 = r5.toString()
            java.lang.String r4 = ""
            goto L_0x0544
        L_0x0540:
            r47 = r4
            r4 = r47
        L_0x0544:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r14)
            r5.append(r11)
            r5.append(r0)
            r5.append(r4)
            java.lang.String r5 = r5.toString()
            r48 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r14)
            r0.append(r4)
            java.lang.String r0 = r0.toString()
            java.lang.String r4 = r75.commentAfter()
            int r20 = r4.length()
            if (r20 != 0) goto L_0x059d
            r49 = r4
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r1.tokens
            org.bytedeco.javacpp.tools.Token r4 = r4.get()
            r50 = r11
            r51 = r14
            r11 = 1
            java.lang.Object[] r14 = new java.lang.Object[r11]
            r11 = 44
            java.lang.Character r11 = java.lang.Character.valueOf(r11)
            r20 = 0
            r14[r20] = r11
            boolean r4 = r4.match(r14)
            if (r4 == 0) goto L_0x05a3
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r1.tokens
            r4.next()
            java.lang.String r4 = r75.commentAfter()
            goto L_0x05a5
        L_0x059d:
            r49 = r4
            r50 = r11
            r51 = r14
        L_0x05a3:
            r4 = r49
        L_0x05a5:
            java.lang.String r11 = r7.spacing
            int r14 = r4.length()
            if (r14 <= 0) goto L_0x05df
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r5)
            r14.append(r11)
            r14.append(r4)
            java.lang.String r5 = r14.toString()
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r0)
            r14.append(r11)
            r14.append(r4)
            java.lang.String r0 = r14.toString()
            r14 = 10
            int r20 = r11.lastIndexOf(r14)
            if (r20 < 0) goto L_0x05df
            int r14 = r20 + 1
            java.lang.String r11 = r11.substring(r14)
        L_0x05df:
            int r14 = r11.length()
            if (r14 != 0) goto L_0x05ef
            java.lang.String r14 = ","
            boolean r14 = r5.endsWith(r14)
            if (r14 != 0) goto L_0x05ef
            java.lang.String r11 = " "
        L_0x05ef:
            java.lang.String r14 = "byte"
            boolean r14 = r13.equals(r14)
            if (r14 != 0) goto L_0x0605
            java.lang.String r14 = "short"
            boolean r14 = r13.equals(r14)
            if (r14 == 0) goto L_0x0600
            goto L_0x0605
        L_0x0600:
            java.lang.String r14 = ""
            r52 = r4
            goto L_0x061d
        L_0x0605:
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r52 = r4
            java.lang.String r4 = "("
            r14.append(r4)
            r14.append(r13)
            java.lang.String r4 = ")"
            r14.append(r4)
            java.lang.String r14 = r14.toString()
        L_0x061d:
            r4 = r14
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r5)
            r14.append(r11)
            r14.append(r3)
            r14.append(r9)
            r53 = r5
            java.lang.String r5 = " = "
            r14.append(r5)
            r14.append(r12)
            java.lang.String r5 = r14.toString()
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r0)
            r14.append(r11)
            r14.append(r3)
            r14.append(r9)
            r54 = r0
            java.lang.String r0 = "("
            r14.append(r0)
            r14.append(r4)
            r14.append(r12)
            java.lang.String r0 = r14.toString()
            java.lang.String r14 = r12.trim()
            int r14 = r14.length()
            if (r14 <= 0) goto L_0x06bb
            if (r10 <= 0) goto L_0x06b6
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r5)
            r55 = r4
            java.lang.String r4 = " + "
            r14.append(r4)
            r14.append(r10)
            java.lang.String r5 = r14.toString()
            r4 = r24
            boolean r14 = r4.containsKey(r12)
            if (r14 == 0) goto L_0x069d
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r0)
            r56 = r5
            java.lang.String r5 = ".value"
            r14.append(r5)
            java.lang.String r0 = r14.toString()
            goto L_0x069f
        L_0x069d:
            r56 = r5
        L_0x069f:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r0)
            java.lang.String r14 = " + "
            r5.append(r14)
            r5.append(r10)
            java.lang.String r0 = r5.toString()
            r5 = r56
            goto L_0x06dd
        L_0x06b6:
            r55 = r4
            r4 = r24
            goto L_0x06dd
        L_0x06bb:
            r55 = r4
            r4 = r24
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r5)
            r14.append(r10)
            java.lang.String r5 = r14.toString()
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r0)
            r14.append(r10)
            java.lang.String r0 = r14.toString()
        L_0x06dd:
            int r10 = r10 + 1
            if (r6 == 0) goto L_0x06f3
            boolean r14 = r6.skip
            if (r14 != 0) goto L_0x06e6
            goto L_0x06f3
        L_0x06e6:
            r28 = r10
            r0 = r18
            r27 = r48
            r11 = r50
            r14 = r51
        L_0x06f0:
            r18 = r12
            goto L_0x074d
        L_0x06f3:
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r14.append(r8)
            r14.append(r5)
            java.lang.String r8 = r14.toString()
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r57 = r6
            r6 = r34
            r14.append(r6)
            r14.append(r0)
            r58 = r0
            java.lang.String r0 = ")"
            r14.append(r0)
            java.lang.String r0 = r14.toString()
            r4.put(r3, r5)
            java.lang.String r6 = ","
            java.lang.String r14 = ""
            java.lang.String r3 = ""
            r34 = r0
            r11 = r3
            r28 = r10
            r27 = r14
            r0 = r18
            r14 = r6
            goto L_0x06f0
        L_0x0730:
            r47 = r4
            r45 = r5
            r38 = r6
            r44 = r10
            r40 = r12
            r4 = r24
            r6 = r34
        L_0x073e:
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            org.bytedeco.javacpp.tools.Token r3 = r3.next()
            java.lang.String r5 = r0.spacing
            r3.spacing = r5
            r34 = r6
            goto L_0x0298
        L_0x074d:
            org.bytedeco.javacpp.tools.TokenIndexer r3 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r3.get()
            r24 = r4
            r4 = r35
            r7 = r36
            r6 = r38
            r12 = r40
            r10 = r44
            r5 = r45
            r3 = r77
            r25 = 0
            goto L_0x020b
        L_0x0767:
            r45 = r5
            r38 = r6
            r36 = r7
            r44 = r10
            r40 = r12
            r37 = r15
            r4 = r24
            r6 = r34
        L_0x0777:
            if (r15 == 0) goto L_0x0794
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = " "
            r3.append(r5)
            r3.append(r13)
            java.lang.String r3 = r3.toString()
            java.lang.String r5 = " long"
            java.lang.String r8 = r8.replace(r3, r5)
            java.lang.String r16 = "long long"
            java.lang.String r13 = "long"
        L_0x0794:
            r3 = r16
            java.lang.String r5 = r75.commentBefore()
            org.bytedeco.javacpp.tools.Declaration r7 = new org.bytedeco.javacpp.tools.Declaration
            r7.<init>()
            org.bytedeco.javacpp.tools.TokenIndexer r9 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r9.get()
            r9 = r38
        L_0x07a7:
            r10 = 2
            java.lang.Object[] r12 = new java.lang.Object[r10]
            java.lang.Character r16 = java.lang.Character.valueOf(r30)
            r17 = 0
            r12[r17] = r16
            org.bytedeco.javacpp.tools.Token r16 = org.bytedeco.javacpp.tools.Token.EOF
            r10 = 1
            r12[r10] = r16
            boolean r12 = r0.match(r12)
            if (r12 != 0) goto L_0x086d
            r12 = r0
            r0 = 0
        L_0x07bf:
            r59 = r4
            java.lang.Object[] r4 = new java.lang.Object[r10]
            r10 = 42
            java.lang.Character r10 = java.lang.Character.valueOf(r10)
            r16 = 0
            r4[r16] = r10
            boolean r4 = r12.match(r4)
            if (r4 == 0) goto L_0x07df
            int r0 = r0 + 1
            org.bytedeco.javacpp.tools.TokenIndexer r4 = r1.tokens
            org.bytedeco.javacpp.tools.Token r12 = r4.next()
            r4 = r59
            r10 = 1
            goto L_0x07bf
        L_0x07df:
            r4 = 1
            java.lang.Object[] r10 = new java.lang.Object[r4]
            r4 = 5
            java.lang.Integer r16 = java.lang.Integer.valueOf(r4)
            r17 = 0
            r10[r17] = r16
            boolean r10 = r12.match(r10)
            if (r10 == 0) goto L_0x085a
            java.lang.String r10 = r12.value
            if (r9 == 0) goto L_0x07fb
            int r16 = r9.length()
            if (r16 != 0) goto L_0x07fc
        L_0x07fb:
            r9 = r10
        L_0x07fc:
            org.bytedeco.javacpp.tools.InfoMap r4 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r4 = r4.getFirst(r3)
            if (r0 <= 0) goto L_0x0836
            r60 = r0
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            r61 = r9
            org.bytedeco.javacpp.tools.Info r9 = new org.bytedeco.javacpp.tools.Info
            r9.<init>((org.bytedeco.javacpp.tools.Info) r4)
            org.bytedeco.javacpp.tools.Info r9 = r9.cast()
            r62 = r12
            r63 = r14
            r12 = 1
            java.lang.String[] r14 = new java.lang.String[r12]
            r16 = 0
            r14[r16] = r10
            org.bytedeco.javacpp.tools.Info r9 = r9.cppNames(r14)
            java.lang.String[] r14 = r4.pointerTypes
            org.bytedeco.javacpp.tools.Info r9 = r9.valueTypes(r14)
            java.lang.String[] r14 = new java.lang.String[r12]
            java.lang.String r17 = "PointerPointer"
            r14[r16] = r17
            org.bytedeco.javacpp.tools.Info r9 = r9.pointerTypes(r14)
            r0.put(r9)
            goto L_0x0857
        L_0x0836:
            r60 = r0
            r61 = r9
            r62 = r12
            r63 = r14
            r12 = 1
            r16 = 0
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r9 = new org.bytedeco.javacpp.tools.Info
            r9.<init>((org.bytedeco.javacpp.tools.Info) r4)
            org.bytedeco.javacpp.tools.Info r9 = r9.cast()
            java.lang.String[] r14 = new java.lang.String[r12]
            r14[r16] = r10
            org.bytedeco.javacpp.tools.Info r9 = r9.cppNames(r14)
            r0.put(r9)
        L_0x0857:
            r9 = r61
            goto L_0x0860
        L_0x085a:
            r60 = r0
            r62 = r12
            r63 = r14
        L_0x0860:
            org.bytedeco.javacpp.tools.TokenIndexer r0 = r1.tokens
            org.bytedeco.javacpp.tools.Token r0 = r0.next()
            r4 = r59
            r14 = r63
            goto L_0x07a7
        L_0x086d:
            r59 = r4
            r63 = r14
            java.lang.String r4 = r2.namespace
            if (r4 == 0) goto L_0x088c
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r10 = r2.namespace
            r4.append(r10)
            java.lang.String r10 = "::"
            r4.append(r10)
            r4.append(r9)
            java.lang.String r4 = r4.toString()
            goto L_0x088d
        L_0x088c:
            r4 = r9
        L_0x088d:
            org.bytedeco.javacpp.tools.InfoMap r10 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r10 = r10.getFirst(r4)
            if (r10 == 0) goto L_0x0898
            boolean r12 = r10.enumerate
            goto L_0x0899
        L_0x0898:
            r12 = 0
        L_0x0899:
            org.bytedeco.javacpp.tools.InfoMap r14 = r1.infoMap
            r64 = r9
            r9 = 0
            java.util.List r9 = r14.get(r9)
            java.util.Iterator r9 = r9.iterator()
        L_0x08a6:
            boolean r14 = r9.hasNext()
            if (r14 == 0) goto L_0x08bb
            java.lang.Object r14 = r9.next()
            org.bytedeco.javacpp.tools.Info r14 = (org.bytedeco.javacpp.tools.Info) r14
            r65 = r9
            boolean r9 = r14.enumerate
            r12 = r12 | r9
            r9 = r65
            goto L_0x08a6
        L_0x08bb:
            if (r10 == 0) goto L_0x08d3
            boolean r9 = r10.skip
            if (r9 == 0) goto L_0x08d3
            r9 = r45
            r7.text = r9
            r72 = r6
            r68 = r10
            r67 = r12
            r73 = r13
            r66 = r15
            r10 = r44
            goto L_0x0b6d
        L_0x08d3:
            r9 = r45
            r14 = 10
            int r14 = r9.lastIndexOf(r14)
            if (r14 >= 0) goto L_0x08e1
            r66 = r15
            r15 = r9
            goto L_0x08e9
        L_0x08e1:
            r66 = r15
            int r15 = r14 + 1
            java.lang.String r15 = r9.substring(r15)
        L_0x08e9:
            if (r12 == 0) goto L_0x0add
            if (r10 == 0) goto L_0x08ff
            r67 = r12
            java.lang.String[] r12 = r10.valueTypes
            if (r12 == 0) goto L_0x0901
            java.lang.String[] r12 = r10.valueTypes
            int r12 = r12.length
            if (r12 <= 0) goto L_0x0901
            java.lang.String[] r12 = r10.valueTypes
            r16 = 0
            r12 = r12[r16]
            goto L_0x0903
        L_0x08ff:
            r67 = r12
        L_0x0901:
            r12 = r64
        L_0x0903:
            r68 = r10
            java.lang.String r10 = r2.namespace
            if (r10 == 0) goto L_0x0922
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r69 = r14
            java.lang.String r14 = r2.namespace
            r10.append(r14)
            java.lang.String r14 = "::"
            r10.append(r14)
            r10.append(r12)
            java.lang.String r10 = r10.toString()
            goto L_0x0925
        L_0x0922:
            r69 = r14
            r10 = r12
        L_0x0925:
            java.lang.String r14 = ""
            boolean r16 = r10.equals(r4)
            if (r16 != 0) goto L_0x094b
            r70 = r10
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r14)
            r71 = r11
            java.lang.String r11 = "@Name(\""
            r10.append(r11)
            r10.append(r4)
            java.lang.String r11 = "\") "
            r10.append(r11)
            java.lang.String r14 = r10.toString()
            goto L_0x0972
        L_0x094b:
            r70 = r10
            r71 = r11
            java.lang.String r10 = r2.namespace
            if (r10 == 0) goto L_0x0972
            java.lang.String r10 = r2.javaName
            if (r10 != 0) goto L_0x0972
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            r10.append(r14)
            java.lang.String r11 = "@Namespace(\""
            r10.append(r11)
            java.lang.String r11 = r2.namespace
            r10.append(r11)
            java.lang.String r11 = "\") "
            r10.append(r11)
            java.lang.String r14 = r10.toString()
        L_0x0972:
            java.lang.StringBuilder r10 = new java.lang.StringBuilder
            r10.<init>()
            java.lang.String r11 = r7.text
            r10.append(r11)
            r10.append(r9)
            r10.append(r14)
            java.lang.String r11 = "public enum "
            r10.append(r11)
            r10.append(r12)
            java.lang.String r11 = " {"
            r10.append(r11)
            r10.append(r6)
            r11 = 1
            java.lang.Object[] r2 = new java.lang.Object[r11]
            java.lang.Character r11 = java.lang.Character.valueOf(r30)
            r72 = r6
            r6 = 0
            r2[r6] = r11
            org.bytedeco.javacpp.tools.Token r2 = r0.expect(r2)
            java.lang.String r2 = r2.spacing
            r10.append(r2)
            java.lang.String r2 = ";"
            r10.append(r2)
            int r2 = r5.length()
            if (r2 <= 0) goto L_0x09c0
            char r2 = r5.charAt(r6)
            r6 = 32
            if (r2 != r6) goto L_0x09c0
            r2 = 1
            java.lang.String r6 = r5.substring(r2)
            goto L_0x09c1
        L_0x09c0:
            r6 = r5
        L_0x09c1:
            r10.append(r6)
            java.lang.String r2 = "\n\n"
            r10.append(r2)
            r10.append(r15)
            java.lang.String r2 = "    public final "
            r10.append(r2)
            r10.append(r13)
            java.lang.String r2 = " value;\n"
            r10.append(r2)
            r10.append(r15)
            java.lang.String r2 = "    private "
            r10.append(r2)
            r10.append(r12)
            java.lang.String r2 = "("
            r10.append(r2)
            r10.append(r13)
            java.lang.String r2 = " v) { this.value = v; }\n"
            r10.append(r2)
            r10.append(r15)
            java.lang.String r2 = "    private "
            r10.append(r2)
            r10.append(r12)
            java.lang.String r2 = "("
            r10.append(r2)
            r10.append(r12)
            java.lang.String r2 = " e) { this.value = e.value; }\n"
            r10.append(r2)
            r10.append(r15)
            java.lang.String r2 = "    public "
            r10.append(r2)
            r10.append(r12)
            java.lang.String r2 = " intern() { for ("
            r10.append(r2)
            r10.append(r12)
            java.lang.String r2 = " e : values()) if (e.value == value) return e; return this; }\n"
            r10.append(r2)
            r10.append(r15)
            java.lang.String r2 = "    @Override public String toString() { return intern().name(); }\n"
            r10.append(r2)
            r10.append(r15)
            java.lang.String r2 = "}"
            r10.append(r2)
            java.lang.String r2 = r10.toString()
            r7.text = r2
            org.bytedeco.javacpp.tools.Info r2 = new org.bytedeco.javacpp.tools.Info
            org.bytedeco.javacpp.tools.InfoMap r6 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r6 = r6.getFirst(r3)
            r2.<init>((org.bytedeco.javacpp.tools.Info) r6)
            r6 = 1
            java.lang.String[] r10 = new java.lang.String[r6]
            r11 = 0
            r10[r11] = r4
            org.bytedeco.javacpp.tools.Info r2 = r2.cppNames(r10)
            java.lang.String[] r10 = r2.valueTypes
            java.lang.String[] r11 = r2.valueTypes
            int r11 = r11.length
            int r11 = r11 + r6
            java.lang.Object[] r6 = java.util.Arrays.copyOf(r10, r11)
            java.lang.String[] r6 = (java.lang.String[]) r6
            r2.valueTypes = r6
            r6 = 1
        L_0x0a5b:
            java.lang.String[] r10 = r2.valueTypes
            int r10 = r10.length
            if (r6 >= r10) goto L_0x0a8a
            java.lang.String[] r10 = r2.valueTypes
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r73 = r13
            java.lang.String r13 = "@Cast(\""
            r11.append(r13)
            r11.append(r4)
            java.lang.String r13 = "\") "
            r11.append(r13)
            java.lang.String[] r13 = r2.valueTypes
            int r16 = r6 + -1
            r13 = r13[r16]
            r11.append(r13)
            java.lang.String r11 = r11.toString()
            r10[r6] = r11
            int r6 = r6 + 1
            r13 = r73
            goto L_0x0a5b
        L_0x0a8a:
            r73 = r13
            java.lang.String[] r6 = r2.valueTypes
            r10 = 0
            r6[r10] = r12
            java.lang.String[] r6 = r2.pointerTypes
            java.lang.String[] r10 = r2.pointerTypes
            int r10 = r10.length
            java.lang.Object[] r6 = java.util.Arrays.copyOf(r6, r10)
            java.lang.String[] r6 = (java.lang.String[]) r6
            r2.pointerTypes = r6
            r6 = 0
        L_0x0a9f:
            java.lang.String[] r10 = r2.pointerTypes
            int r10 = r10.length
            if (r6 >= r10) goto L_0x0ac8
            java.lang.String[] r10 = r2.pointerTypes
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            java.lang.String r13 = "@Cast(\""
            r11.append(r13)
            r11.append(r4)
            java.lang.String r13 = "*\") "
            r11.append(r13)
            java.lang.String[] r13 = r2.pointerTypes
            r13 = r13[r6]
            r11.append(r13)
            java.lang.String r11 = r11.toString()
            r10[r6] = r11
            int r6 = r6 + 1
            goto L_0x0a9f
        L_0x0ac8:
            org.bytedeco.javacpp.tools.InfoMap r6 = r1.infoMap
            r10 = 0
            org.bytedeco.javacpp.tools.Info r10 = r2.cast(r10)
            org.bytedeco.javacpp.tools.Info r10 = r10.enumerate()
            r6.put(r10)
            r10 = r44
            r11 = r71
            goto L_0x0b6d
        L_0x0add:
            r72 = r6
            r68 = r10
            r71 = r11
            r67 = r12
            r73 = r13
            r69 = r14
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r6 = r7.text
            r2.append(r6)
            r2.append(r9)
            java.lang.String r6 = "/** "
            r2.append(r6)
            r10 = r44
            r2.append(r10)
            java.lang.String r6 = " "
            r2.append(r6)
            r2.append(r4)
            java.lang.String r6 = " */\n"
            r2.append(r6)
            r2.append(r15)
            r2.append(r8)
            r6 = 1
            java.lang.Object[] r11 = new java.lang.Object[r6]
            java.lang.Character r6 = java.lang.Character.valueOf(r30)
            r12 = 0
            r11[r12] = r6
            org.bytedeco.javacpp.tools.Token r6 = r0.expect(r11)
            java.lang.String r6 = r6.spacing
            r2.append(r6)
            java.lang.String r6 = ";"
            r2.append(r6)
            java.lang.String r2 = r2.toString()
            r7.text = r2
            int r2 = r4.length()
            if (r2 <= 0) goto L_0x0b55
            org.bytedeco.javacpp.tools.InfoMap r2 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r2 = r2.getFirst(r3)
            org.bytedeco.javacpp.tools.InfoMap r6 = r1.infoMap
            org.bytedeco.javacpp.tools.Info r11 = new org.bytedeco.javacpp.tools.Info
            r11.<init>((org.bytedeco.javacpp.tools.Info) r2)
            org.bytedeco.javacpp.tools.Info r11 = r11.cast()
            r12 = 1
            java.lang.String[] r13 = new java.lang.String[r12]
            r12 = 0
            r13[r12] = r4
            org.bytedeco.javacpp.tools.Info r11 = r11.cppNames(r13)
            r6.put(r11)
        L_0x0b55:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r6 = r7.text
            r2.append(r6)
            r11 = r71
            r2.append(r11)
            r2.append(r5)
            java.lang.String r2 = r2.toString()
            r7.text = r2
        L_0x0b6d:
            r2 = r77
            r2.add((org.bytedeco.javacpp.tools.Declaration) r7)
            org.bytedeco.javacpp.tools.TokenIndexer r6 = r1.tokens
            r6.next()
            r6 = 1
            return r6
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.enumeration(org.bytedeco.javacpp.tools.Context, org.bytedeco.javacpp.tools.DeclarationList):boolean");
    }

    /* access modifiers changed from: package-private */
    public boolean namespace(Context context, DeclarationList declList) throws ParserException {
        String str;
        if (!this.tokens.get().match(Token.NAMESPACE)) {
            return false;
        }
        Declaration decl = new Declaration();
        String spacing = this.tokens.get().spacing;
        String name = null;
        this.tokens.next();
        if (this.tokens.get().match(5)) {
            name = this.tokens.get().value;
            this.tokens.next();
        }
        if (this.tokens.get().match('=')) {
            this.tokens.next();
            context.namespaceMap.put(name, type(context).cppName);
            this.tokens.get().expect(';');
            this.tokens.next();
            return true;
        }
        this.tokens.get().expect('{');
        this.tokens.next();
        if (this.tokens.get().spacing.indexOf(10) < 0) {
            this.tokens.get().spacing = spacing;
        }
        Context context2 = new Context(context);
        if (name == null) {
            str = context2.namespace;
        } else if (context2.namespace != null) {
            str = context2.namespace + "::" + name;
        } else {
            str = name;
        }
        context2.namespace = str;
        declarations(context2, declList);
        decl.text += this.tokens.get().expect('}').spacing;
        this.tokens.next();
        declList.add(decl);
        return true;
    }

    /* access modifiers changed from: package-private */
    public boolean extern(Context context, DeclarationList declList) throws ParserException {
        if (this.tokens.get().match(Token.EXTERN)) {
            if (this.tokens.get(1).match(3)) {
                String spacing = this.tokens.get().spacing;
                Declaration decl = new Declaration();
                this.tokens.next().expect("\"C\"", "\"C++\"");
                if (!this.tokens.next().match('{')) {
                    this.tokens.get().spacing = spacing;
                    declList.add(decl);
                    return true;
                }
                this.tokens.next();
                declarations(context, declList);
                this.tokens.get().expect('}');
                this.tokens.next();
                declList.add(decl);
                return true;
            }
        }
        return false;
    }

    /* access modifiers changed from: package-private */
    public void declarations(Context context, DeclarationList declList) throws ParserException {
        String comment;
        String spacing;
        String str;
        String spacing2;
        String comment2;
        Context context2 = context;
        DeclarationList declarationList = declList;
        Token token = this.tokens.get();
        while (true) {
            char c = 0;
            if (!token.match(Token.EOF, '}')) {
                if (token.match(Token.PRIVATE, Token.PROTECTED, Token.PUBLIC)) {
                    if (this.tokens.get(1).match(':')) {
                        context2.inaccessible = !token.match(Token.PUBLIC);
                        this.tokens.next();
                        this.tokens.next();
                        token = this.tokens.get();
                        context2 = context;
                    }
                }
                Context ctx = context;
                String comment3 = commentBefore();
                Token token2 = this.tokens.get();
                String spacing3 = token2.spacing;
                TemplateMap map = template(ctx);
                if (map != null) {
                    token2 = this.tokens.get();
                    token2.spacing = spacing3;
                    ctx = new Context(ctx);
                    ctx.templateMap = map;
                }
                Declaration decl = new Declaration();
                if (comment3 != null && comment3.length() > 0) {
                    decl.inaccessible = ctx.inaccessible;
                    decl.text = comment3;
                    decl.comment = true;
                    declarationList.add(decl);
                }
                int startIndex = this.tokens.index;
                declarationList.infoMap = this.infoMap;
                declarationList.context = ctx;
                declarationList.templateMap = map;
                declarationList.infoIterator = null;
                declarationList.spacing = null;
                while (true) {
                    if (map != null && declarationList.infoIterator != null && declarationList.infoIterator.hasNext()) {
                        Info info = declarationList.infoIterator.next();
                        if (info != null) {
                            Type type = new Parser(this, info.cppNames[c]).type(context2);
                            if (type.arguments != null) {
                                int count = 0;
                                for (Map.Entry<String, Type> e : map.entrySet()) {
                                    if (count < type.arguments.length) {
                                        int count2 = count + 1;
                                        Type t = type.arguments[count];
                                        String s = t.cppName;
                                        if (!t.constValue || s.startsWith("const ")) {
                                            comment2 = comment3;
                                        } else {
                                            StringBuilder sb = new StringBuilder();
                                            comment2 = comment3;
                                            sb.append("const ");
                                            sb.append(s);
                                            s = sb.toString();
                                        }
                                        if (t.constPointer && !s.endsWith(" const")) {
                                            s = s + " const";
                                        }
                                        if (t.indirections > 0) {
                                            int i = 0;
                                            while (i < t.indirections) {
                                                s = s + "*";
                                                i++;
                                                spacing3 = spacing3;
                                            }
                                        }
                                        spacing2 = spacing3;
                                        if (t.reference) {
                                            s = s + "&";
                                        }
                                        t.cppName = s;
                                        e.setValue(t);
                                        count = count2;
                                    } else {
                                        comment2 = comment3;
                                        spacing2 = spacing3;
                                    }
                                    comment3 = comment2;
                                    spacing3 = spacing2;
                                    Context context3 = context;
                                }
                                comment = comment3;
                                spacing = spacing3;
                                this.tokens.index = startIndex;
                            }
                        }
                        comment = comment3;
                        if (declarationList.infoIterator == null || !declarationList.infoIterator.hasNext()) {
                            break;
                        }
                        comment3 = comment;
                        context2 = context;
                        c = 0;
                    } else {
                        comment = comment3;
                        spacing = spacing3;
                    }
                    if (this.tokens.get().match(';') || macro(ctx, declarationList) || extern(ctx, declarationList) || namespace(ctx, declarationList) || enumeration(ctx, declarationList) || group(ctx, declarationList) || typedef(ctx, declarationList) || using(ctx, declarationList) || function(ctx, declarationList) || variable(ctx, declarationList)) {
                        spacing3 = spacing;
                    } else {
                        spacing3 = this.tokens.get().spacing;
                        if (attribute() != null) {
                            this.tokens.get().spacing = spacing3;
                        } else {
                            StringBuilder sb2 = new StringBuilder();
                            sb2.append(token2.file);
                            sb2.append(":");
                            sb2.append(token2.lineNumber);
                            sb2.append(":");
                            if (token2.text != null) {
                                str = "\"" + token2.text + "\": ";
                            } else {
                                str = "";
                            }
                            sb2.append(str);
                            sb2.append("Could not parse declaration at '");
                            sb2.append(token2);
                            sb2.append("'");
                            throw new ParserException(sb2.toString());
                        }
                    }
                    while (true) {
                        if (!this.tokens.get().match(';')) {
                            break;
                        }
                        if (this.tokens.get().match(Token.EOF)) {
                            break;
                        }
                        this.tokens.next();
                    }
                    comment3 = comment;
                    context2 = context;
                    c = 0;
                }
                token = this.tokens.get();
                context2 = context;
            } else {
                StringBuilder sb3 = new StringBuilder();
                sb3.append(commentBefore());
                sb3.append(this.tokens.get().match(Token.EOF) ? this.tokens.get().spacing : "");
                String comment4 = sb3.toString();
                Declaration decl2 = new Declaration();
                if (comment4 != null && comment4.length() > 0) {
                    decl2.text = comment4;
                    decl2.comment = true;
                    declarationList.add(decl2);
                    return;
                }
                return;
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void parse(Context context, DeclarationList declList, String[] includePath, String include, boolean isCFile) throws IOException, ParserException {
        List<Token> tokenList = new ArrayList<>();
        File file = null;
        String filename = include;
        if (!filename.startsWith("<") || !filename.endsWith(">")) {
            File f = new File(filename);
            if (f.exists()) {
                file = f;
            }
        } else {
            filename = filename.substring(1, filename.length() - 1);
        }
        if (file == null && includePath != null) {
            int length = includePath.length;
            int i = 0;
            while (true) {
                if (i >= length) {
                    break;
                }
                File f2 = new File(includePath[i], filename).getCanonicalFile();
                if (f2.exists()) {
                    file = f2;
                    break;
                }
                i++;
            }
        }
        if (file == null) {
            file = new File(filename);
        }
        Info info = this.infoMap.getFirst(file.getName());
        if (info != null && info.skip && info.linePatterns == null) {
            return;
        }
        if (file.exists()) {
            Logger logger2 = this.logger;
            logger2.info("Parsing " + file);
            Token token = new Token();
            token.type = 4;
            token.value = "\n// Parsed from " + include + "\n\n";
            tokenList.add(token);
            Tokenizer tokenizer = new Tokenizer(file, this.encoding);
            if (!(info == null || info.linePatterns == null)) {
                tokenizer.filterLines(info.linePatterns, info.skip);
            }
            while (true) {
                Token nextToken = tokenizer.nextToken();
                Token token2 = nextToken;
                if (nextToken.isEmpty()) {
                    break;
                }
                if (token2.type == -1) {
                    token2.type = 4;
                }
                tokenList.add(token2);
            }
            if (this.lineSeparator == null) {
                this.lineSeparator = tokenizer.lineSeparator;
            }
            tokenizer.close();
            Token token3 = new Token(Token.EOF);
            token3.spacing = "\n";
            token3.file = file;
            token3.lineNumber = tokenList.get(tokenList.size() - 1).lineNumber;
            tokenList.add(token3);
            this.tokens = new TokenIndexer(this.infoMap, (Token[]) tokenList.toArray(new Token[tokenList.size()]), isCFile);
            declarations(context, declList);
            return;
        }
        throw new FileNotFoundException("Could not parse \"" + file + "\": File does not exist");
    }

    public File parse(String outputDirectory, String[] classPath, Class cls) throws IOException, ParserException {
        return parse(new File(outputDirectory), classPath, cls);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v26, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v103, resolved type: java.lang.String[]} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* JADX WARNING: Removed duplicated region for block: B:178:0x062f A[Catch:{ Throwable -> 0x06c4, all -> 0x06bf }] */
    /* JADX WARNING: Removed duplicated region for block: B:179:0x0636 A[Catch:{ Throwable -> 0x06c4, all -> 0x06bf }] */
    /* JADX WARNING: Removed duplicated region for block: B:222:0x06f0 A[SYNTHETIC, Splitter:B:222:0x06f0] */
    /* JADX WARNING: Removed duplicated region for block: B:226:0x06fa  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.io.File parse(java.io.File r48, java.lang.String[] r49, java.lang.Class r50) throws java.io.IOException, org.bytedeco.javacpp.tools.ParserException {
        /*
            r47 = this;
            r7 = r47
            r8 = r50
            java.util.Properties r0 = r7.properties
            r1 = 1
            org.bytedeco.javacpp.ClassProperties r9 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class) r8, (java.util.Properties) r0, (boolean) r1)
            java.util.Properties r0 = r7.properties
            r10 = 0
            org.bytedeco.javacpp.ClassProperties r11 = org.bytedeco.javacpp.Loader.loadProperties((java.lang.Class) r8, (java.util.Properties) r0, (boolean) r10)
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r12 = r0
            java.lang.String r0 = "platform.cinclude"
            java.util.List r0 = r11.get(r0)
            r12.addAll(r0)
            java.lang.String r0 = "platform.cinclude"
            java.util.List r0 = r9.get(r0)
            r12.addAll(r0)
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r13 = r0
            java.lang.String r0 = "platform.include"
            java.util.List r0 = r11.get(r0)
            r13.addAll(r0)
            java.lang.String r0 = "platform.cinclude"
            java.util.List r0 = r11.get(r0)
            r13.addAll(r0)
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r14 = r0
            java.lang.String r0 = "platform.include"
            java.util.List r0 = r9.get(r0)
            r14.addAll(r0)
            java.lang.String r0 = "platform.cinclude"
            java.util.List r0 = r9.get(r0)
            r14.addAll(r0)
            java.lang.String r0 = "target"
            java.util.List r15 = r9.get(r0)
            java.lang.String r0 = "global"
            java.util.List r6 = r9.get(r0)
            java.lang.String r0 = "target"
            java.util.List r5 = r11.get(r0)
            java.lang.String r0 = "global"
            java.util.List r4 = r11.get(r0)
            java.lang.String r0 = "helper"
            java.util.List r3 = r11.get(r0)
            int r0 = r5.size()
            int r0 = r0 - r1
            java.lang.Object r0 = r5.get(r0)
            r2 = r0
            java.lang.String r2 = (java.lang.String) r2
            int r0 = r4.size()
            int r0 = r0 - r1
            java.lang.Object r0 = r4.get(r0)
            r1 = r0
            java.lang.String r1 = (java.lang.String) r1
            java.util.List r17 = r9.getInheritedClasses()
            org.bytedeco.javacpp.tools.InfoMap r0 = new org.bytedeco.javacpp.tools.InfoMap
            r0.<init>()
            r7.infoMap = r0
            java.util.Iterator r18 = r17.iterator()
        L_0x009f:
            boolean r0 = r18.hasNext()
            if (r0 == 0) goto L_0x00e6
            java.lang.Object r0 = r18.next()
            java.lang.Class r0 = (java.lang.Class) r0
            r19 = r0
            java.lang.Object r0 = r19.newInstance()     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d8 }
            org.bytedeco.javacpp.tools.InfoMapper r0 = (org.bytedeco.javacpp.tools.InfoMapper) r0     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d8 }
            boolean r10 = r0 instanceof org.bytedeco.javacpp.tools.BuildEnabled     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d8 }
            if (r10 == 0) goto L_0x00cc
            r10 = r0
            org.bytedeco.javacpp.tools.BuildEnabled r10 = (org.bytedeco.javacpp.tools.BuildEnabled) r10     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d8 }
            r20 = r4
            org.bytedeco.javacpp.tools.Logger r4 = r7.logger     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00c8 }
            r21 = r5
            java.util.Properties r5 = r7.properties     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d6 }
            java.lang.String r8 = r7.encoding     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d6 }
            r10.init(r4, r5, r8)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d6 }
            goto L_0x00d0
        L_0x00c8:
            r0 = move-exception
            r21 = r5
            goto L_0x00dd
        L_0x00cc:
            r20 = r4
            r21 = r5
        L_0x00d0:
            org.bytedeco.javacpp.tools.InfoMap r4 = r7.infoMap     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d6 }
            r0.map(r4)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x00d6 }
            goto L_0x00dd
        L_0x00d6:
            r0 = move-exception
            goto L_0x00dd
        L_0x00d8:
            r0 = move-exception
            r20 = r4
            r21 = r5
        L_0x00dd:
            r4 = r20
            r5 = r21
            r8 = r50
            r10 = 0
            goto L_0x009f
        L_0x00e6:
            r20 = r4
            r21 = r5
            org.bytedeco.javacpp.tools.InfoMap r0 = new org.bytedeco.javacpp.tools.InfoMap
            r0.<init>()
            r7.leafInfoMap = r0
            java.lang.Object r0 = r50.newInstance()     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            org.bytedeco.javacpp.tools.InfoMapper r0 = (org.bytedeco.javacpp.tools.InfoMapper) r0     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            boolean r4 = r0 instanceof org.bytedeco.javacpp.tools.BuildEnabled     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            if (r4 == 0) goto L_0x0107
            r4 = r0
            org.bytedeco.javacpp.tools.BuildEnabled r4 = (org.bytedeco.javacpp.tools.BuildEnabled) r4     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            org.bytedeco.javacpp.tools.Logger r5 = r7.logger     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            java.util.Properties r8 = r7.properties     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            java.lang.String r10 = r7.encoding     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            r4.init(r5, r8, r10)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
        L_0x0107:
            org.bytedeco.javacpp.tools.InfoMap r4 = r7.leafInfoMap     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            r0.map(r4)     // Catch:{ ClassCastException | IllegalAccessException | InstantiationException -> 0x010d }
            goto L_0x010e
        L_0x010d:
            r0 = move-exception
        L_0x010e:
            org.bytedeco.javacpp.tools.InfoMap r0 = r7.infoMap
            org.bytedeco.javacpp.tools.InfoMap r4 = r7.leafInfoMap
            r0.putAll(r4)
            java.lang.Class<org.bytedeco.javacpp.tools.Parser> r0 = org.bytedeco.javacpp.tools.Parser.class
            java.lang.Package r0 = r0.getPackage()
            java.lang.String r0 = r0.getImplementationVersion()
            if (r0 != 0) goto L_0x0123
            java.lang.String r0 = "unknown"
        L_0x0123:
            r8 = r0
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r4 = "// Targeted by JavaCPP version "
            r0.append(r4)
            r0.append(r8)
            java.lang.String r4 = ": DO NOT EDIT THIS FILE\n\n"
            r0.append(r4)
            java.lang.String r0 = r0.toString()
            java.lang.String r4 = ""
            r5 = 46
            int r10 = r1.lastIndexOf(r5)
            if (r10 < 0) goto L_0x0167
            r5 = 0
            java.lang.String r4 = r1.substring(r5, r10)
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r0)
            r23 = r0
            java.lang.String r0 = "package "
            r5.append(r0)
            r5.append(r4)
            java.lang.String r0 = ";\n\n"
            r5.append(r0)
            java.lang.String r0 = r5.toString()
            r18 = r4
            goto L_0x016b
        L_0x0167:
            r23 = r0
            r18 = r4
        L_0x016b:
            org.bytedeco.javacpp.tools.InfoMap r4 = r7.leafInfoMap
            r5 = 0
            java.util.List r19 = r4.get(r5)
            java.util.Iterator r4 = r19.iterator()
        L_0x0176:
            boolean r23 = r4.hasNext()
            if (r23 == 0) goto L_0x01b5
            java.lang.Object r23 = r4.next()
            r5 = r23
            org.bytedeco.javacpp.tools.Info r5 = (org.bytedeco.javacpp.tools.Info) r5
            r25 = r4
            java.lang.String r4 = r5.javaText
            if (r4 == 0) goto L_0x01ad
            java.lang.String r4 = r5.javaText
            r26 = r8
            java.lang.String r8 = "import"
            boolean r4 = r4.startsWith(r8)
            if (r4 == 0) goto L_0x01af
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r0)
            java.lang.String r8 = r5.javaText
            r4.append(r8)
            java.lang.String r8 = "\n"
            r4.append(r8)
            java.lang.String r0 = r4.toString()
            goto L_0x01af
        L_0x01ad:
            r26 = r8
        L_0x01af:
            r4 = r25
            r8 = r26
            r5 = 0
            goto L_0x0176
        L_0x01b5:
            r26 = r8
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            r4.append(r0)
            java.lang.String r5 = "import java.nio.*;\nimport org.bytedeco.javacpp.*;\nimport org.bytedeco.javacpp.annotation.*;\n\n"
            r4.append(r5)
            java.lang.String r0 = r4.toString()
            r4 = r0
            r0 = 0
        L_0x01ca:
            int r5 = r15.size()
            if (r0 >= r5) goto L_0x023a
            java.lang.Object r5 = r15.get(r0)
            boolean r5 = r2.equals(r5)
            if (r5 != 0) goto L_0x0237
            java.lang.Object r5 = r15.get(r0)
            java.lang.String r5 = (java.lang.String) r5
            java.lang.Object r8 = r6.get(r0)
            boolean r5 = r5.equals(r8)
            if (r5 == 0) goto L_0x020a
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r4)
            java.lang.String r8 = "import static "
            r5.append(r8)
            java.lang.Object r8 = r15.get(r0)
            java.lang.String r8 = (java.lang.String) r8
            r5.append(r8)
            java.lang.String r8 = ".*;\n"
            r5.append(r8)
            java.lang.String r4 = r5.toString()
            goto L_0x0237
        L_0x020a:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r5.append(r4)
            java.lang.String r8 = "import "
            r5.append(r8)
            java.lang.Object r8 = r15.get(r0)
            java.lang.String r8 = (java.lang.String) r8
            r5.append(r8)
            java.lang.String r8 = ".*;\nimport static "
            r5.append(r8)
            java.lang.Object r8 = r6.get(r0)
            java.lang.String r8 = (java.lang.String) r8
            r5.append(r8)
            java.lang.String r8 = ".*;\n"
            r5.append(r8)
            java.lang.String r4 = r5.toString()
        L_0x0237:
            int r0 = r0 + 1
            goto L_0x01ca
        L_0x023a:
            int r0 = r15.size()
            r5 = 1
            if (r0 <= r5) goto L_0x0252
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r4)
            java.lang.String r5 = "\n"
            r0.append(r5)
            java.lang.String r4 = r0.toString()
        L_0x0252:
            r8 = r4
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r8)
            java.lang.String r4 = "public class "
            r0.append(r4)
            int r4 = r10 + 1
            java.lang.String r4 = r1.substring(r4)
            r0.append(r4)
            java.lang.String r4 = " extends "
            r0.append(r4)
            int r4 = r3.size()
            if (r4 <= 0) goto L_0x0282
            int r4 = r13.size()
            if (r4 <= 0) goto L_0x0282
            r4 = 0
            java.lang.Object r5 = r3.get(r4)
            java.lang.String r5 = (java.lang.String) r5
            goto L_0x0286
        L_0x0282:
            java.lang.String r5 = r50.getCanonicalName()
        L_0x0286:
            r0.append(r5)
            java.lang.String r4 = " {\n    static { Loader.load(); }\n"
            r0.append(r4)
            java.lang.String r5 = r0.toString()
            char r0 = java.io.File.separatorChar
            r4 = 46
            java.lang.String r4 = r1.replace(r4, r0)
            java.io.File r0 = new java.io.File
            r27 = r1
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            r1.append(r4)
            r28 = r2
            java.lang.String r2 = ".java"
            r1.append(r2)
            java.lang.String r1 = r1.toString()
            r2 = r48
            r0.<init>(r2, r1)
            r1 = r0
            org.bytedeco.javacpp.tools.Logger r0 = r7.logger
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r29 = r3
            java.lang.String r3 = "Targeting "
            r2.append(r3)
            r2.append(r1)
            java.lang.String r2 = r2.toString()
            r0.info(r2)
            org.bytedeco.javacpp.tools.Context r0 = new org.bytedeco.javacpp.tools.Context
            r0.<init>()
            r3 = r0
            org.bytedeco.javacpp.tools.InfoMap r0 = r7.infoMap
            r3.infoMap = r0
            r0 = r49
            char r2 = java.io.File.separatorChar
            int r10 = r4.lastIndexOf(r2)
            if (r10 < 0) goto L_0x0320
            java.lang.Object r2 = r49.clone()
            r0 = r2
            java.lang.String[] r0 = (java.lang.String[]) r0
            r2 = 0
        L_0x02eb:
            r30 = r5
            int r5 = r0.length
            if (r2 >= r5) goto L_0x031a
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            r31 = r6
            r6 = r0[r2]
            r5.append(r6)
            java.lang.String r6 = java.io.File.separator
            r5.append(r6)
            r32 = r11
            r6 = 0
            java.lang.String r11 = r4.substring(r6, r10)
            r5.append(r11)
            java.lang.String r5 = r5.toString()
            r0[r2] = r5
            int r2 = r2 + 1
            r5 = r30
            r6 = r31
            r11 = r32
            goto L_0x02eb
        L_0x031a:
            r31 = r6
            r32 = r11
            r11 = r0
            goto L_0x0327
        L_0x0320:
            r30 = r5
            r31 = r6
            r32 = r11
            r11 = r0
        L_0x0327:
            java.lang.String r0 = "platform.includepath"
            java.util.List r6 = r9.get(r0)
            java.lang.String r0 = "platform.includeresource"
            java.util.List r0 = r9.get(r0)
            java.util.Iterator r0 = r0.iterator()
        L_0x0337:
            boolean r2 = r0.hasNext()
            if (r2 == 0) goto L_0x0363
            java.lang.Object r2 = r0.next()
            java.lang.String r2 = (java.lang.String) r2
            java.io.File[] r5 = org.bytedeco.javacpp.Loader.cacheResources(r2)
            r33 = r0
            int r0 = r5.length
            r34 = r2
            r2 = 0
        L_0x034d:
            if (r2 >= r0) goto L_0x035f
            r16 = r5[r2]
            r35 = r0
            java.lang.String r0 = r16.getCanonicalPath()
            r6.add(r0)
            int r2 = r2 + 1
            r0 = r35
            goto L_0x034d
        L_0x035f:
            r0 = r33
            goto L_0x0337
        L_0x0363:
            int r0 = r13.size()
            if (r0 != 0) goto L_0x0381
            org.bytedeco.javacpp.tools.Logger r0 = r7.logger
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r5 = "Nothing targeted for "
            r2.append(r5)
            r2.append(r1)
            java.lang.String r2 = r2.toString()
            r0.info(r2)
            r5 = 0
            return r5
        L_0x0381:
            r5 = 0
            int r0 = r6.size()
            int r2 = r11.length
            int r0 = r0 + r2
            java.lang.String[] r0 = new java.lang.String[r0]
            java.lang.Object[] r0 = r6.toArray(r0)
            r2 = r0
            java.lang.String[] r2 = (java.lang.String[]) r2
            int r0 = r6.size()
            int r5 = r11.length
            r36 = r1
            r1 = 0
            java.lang.System.arraycopy(r11, r1, r2, r0, r5)
            org.bytedeco.javacpp.tools.DeclarationList r0 = new org.bytedeco.javacpp.tools.DeclarationList
            r0.<init>()
            java.util.Iterator r16 = r14.iterator()
        L_0x03a5:
            boolean r1 = r16.hasNext()
            if (r1 == 0) goto L_0x0427
            java.lang.Object r1 = r16.next()
            r5 = r1
            java.lang.String r5 = (java.lang.String) r5
            boolean r1 = r13.contains(r5)
            if (r1 != 0) goto L_0x03e9
            boolean r22 = r12.contains(r5)
            r37 = r9
            r38 = r10
            r9 = r27
            r10 = r36
            r1 = r47
            r23 = r2
            r39 = r11
            r11 = r28
            r2 = r3
            r40 = r15
            r24 = r29
            r15 = r3
            r3 = r0
            r25 = r4
            r4 = r23
            r27 = r5
            r41 = r8
            r8 = r30
            r28 = 0
            r30 = r6
            r29 = r31
            r6 = r22
            r1.parse(r2, r3, r4, r5, r6)
            goto L_0x0408
        L_0x03e9:
            r23 = r2
            r25 = r4
            r41 = r8
            r37 = r9
            r38 = r10
            r39 = r11
            r40 = r15
            r9 = r27
            r11 = r28
            r24 = r29
            r8 = r30
            r29 = r31
            r10 = r36
            r28 = 0
            r15 = r3
            r30 = r6
        L_0x0408:
            r27 = r9
            r36 = r10
            r28 = r11
            r3 = r15
            r2 = r23
            r4 = r25
            r31 = r29
            r6 = r30
            r9 = r37
            r10 = r38
            r11 = r39
            r15 = r40
            r30 = r8
            r29 = r24
            r8 = r41
            goto L_0x03a5
        L_0x0427:
            r23 = r2
            r25 = r4
            r41 = r8
            r37 = r9
            r38 = r10
            r39 = r11
            r40 = r15
            r9 = r27
            r11 = r28
            r24 = r29
            r8 = r30
            r29 = r31
            r10 = r36
            r28 = 0
            r15 = r3
            r30 = r6
            org.bytedeco.javacpp.tools.DeclarationList r1 = new org.bytedeco.javacpp.tools.DeclarationList
            r1.<init>(r0)
            r6 = r1
            int r0 = r13.size()
            if (r0 <= 0) goto L_0x0485
            r7.containers(r15, r6)
            java.util.Iterator r0 = r13.iterator()
        L_0x0459:
            boolean r1 = r0.hasNext()
            if (r1 == 0) goto L_0x0485
            java.lang.Object r1 = r0.next()
            r5 = r1
            java.lang.String r5 = (java.lang.String) r5
            boolean r1 = r14.contains(r5)
            if (r1 == 0) goto L_0x0480
            boolean r16 = r12.contains(r5)
            r1 = r47
            r2 = r15
            r3 = r6
            r4 = r23
            r22 = r5
            r27 = r6
            r6 = r16
            r1.parse(r2, r3, r4, r5, r6)
            goto L_0x0482
        L_0x0480:
            r27 = r6
        L_0x0482:
            r6 = r27
            goto L_0x0459
        L_0x0485:
            r27 = r6
            int r0 = r27.size()
            if (r0 != 0) goto L_0x04a4
            org.bytedeco.javacpp.tools.Logger r0 = r7.logger
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.String r2 = "Nothing targeted for "
            r1.append(r2)
            r1.append(r10)
            java.lang.String r1 = r1.toString()
            r0.info(r1)
            return r28
        L_0x04a4:
            java.io.File r1 = r10.getParentFile()
            if (r1 == 0) goto L_0x04ad
            r1.mkdirs()
        L_0x04ad:
            java.lang.String r0 = r7.encoding
            if (r0 == 0) goto L_0x04bb
            org.bytedeco.javacpp.tools.EncodingFileWriter r0 = new org.bytedeco.javacpp.tools.EncodingFileWriter
            java.lang.String r2 = r7.encoding
            java.lang.String r3 = r7.lineSeparator
            r0.<init>(r10, r2, r3)
            goto L_0x04c2
        L_0x04bb:
            org.bytedeco.javacpp.tools.EncodingFileWriter r0 = new org.bytedeco.javacpp.tools.EncodingFileWriter
            java.lang.String r2 = r7.lineSeparator
            r0.<init>(r10, r2)
        L_0x04c2:
            r2 = r0
            r2.append(r8)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.util.Iterator r0 = r19.iterator()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
        L_0x04ca:
            boolean r3 = r0.hasNext()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            if (r3 == 0) goto L_0x0514
            java.lang.Object r3 = r0.next()     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            org.bytedeco.javacpp.tools.Info r3 = (org.bytedeco.javacpp.tools.Info) r3     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            java.lang.String r4 = r3.javaText     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            if (r4 == 0) goto L_0x04fa
            java.lang.String r4 = r3.javaText     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            java.lang.String r5 = "import"
            boolean r4 = r4.startsWith(r5)     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            if (r4 != 0) goto L_0x04fa
            java.lang.StringBuilder r4 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            r4.<init>()     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            java.lang.String r5 = r3.javaText     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            r4.append(r5)     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            java.lang.String r5 = "\n"
            r4.append(r5)     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            java.lang.String r4 = r4.toString()     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            r2.append(r4)     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
        L_0x04fa:
            goto L_0x04ca
        L_0x04fb:
            r0 = move-exception
            r43 = r1
            r45 = r8
            r46 = r9
            r5 = r28
            r44 = r41
            goto L_0x06ed
        L_0x0508:
            r0 = move-exception
            r5 = r0
            r43 = r1
            r45 = r8
            r46 = r9
            r44 = r41
            goto L_0x06eb
        L_0x0514:
            r0 = 0
            java.util.Iterator r3 = r27.iterator()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
        L_0x0519:
            boolean r4 = r3.hasNext()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            if (r4 == 0) goto L_0x06af
            java.lang.Object r4 = r3.next()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            org.bytedeco.javacpp.tools.Declaration r4 = (org.bytedeco.javacpp.tools.Declaration) r4     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            boolean r5 = r11.equals(r9)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            if (r5 != 0) goto L_0x068f
            org.bytedeco.javacpp.tools.Type r5 = r4.type     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            if (r5 == 0) goto L_0x068f
            org.bytedeco.javacpp.tools.Type r5 = r4.type     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r5 = r5.javaName     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            if (r5 == 0) goto L_0x068f
            org.bytedeco.javacpp.tools.Type r5 = r4.type     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r5 = r5.javaName     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            int r5 = r5.length()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            if (r5 <= 0) goto L_0x068f
            java.io.File r5 = new java.io.File     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.StringBuilder r6 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r6.<init>()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r42 = r3
            org.bytedeco.javacpp.tools.Type r3 = r4.type     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r3 = r3.javaName     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r6.append(r3)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r3 = ".java"
            r6.append(r3)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r3 = r6.toString()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r5.<init>(r1, r3)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r3 = r5
            if (r0 == 0) goto L_0x0567
            boolean r5 = r0.comment     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            if (r5 != 0) goto L_0x0567
            java.lang.String r5 = r0.text     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
            r2.append(r5)     // Catch:{ Throwable -> 0x0508, all -> 0x04fb }
        L_0x0567:
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r5.<init>()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r6 = "\n// Targeting "
            r5.append(r6)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            org.bytedeco.javacpp.tools.Type r6 = r4.type     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r6 = r6.javaName     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r5.append(r6)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r6 = ".java\n\n"
            r5.append(r6)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.String r5 = r5.toString()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r2.append(r5)     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            org.bytedeco.javacpp.tools.Logger r5 = r7.logger     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            java.lang.StringBuilder r6 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r6.<init>()     // Catch:{ Throwable -> 0x06e1, all -> 0x06d4 }
            r43 = r1
            java.lang.String r1 = "Targeting "
            r6.append(r1)     // Catch:{ Throwable -> 0x0685, all -> 0x0679 }
            r6.append(r3)     // Catch:{ Throwable -> 0x0685, all -> 0x0679 }
            java.lang.String r1 = r6.toString()     // Catch:{ Throwable -> 0x0685, all -> 0x0679 }
            r5.info(r1)     // Catch:{ Throwable -> 0x0685, all -> 0x0679 }
            java.lang.StringBuilder r1 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x0685, all -> 0x0679 }
            r1.<init>()     // Catch:{ Throwable -> 0x0685, all -> 0x0679 }
            r5 = r41
            r1.append(r5)     // Catch:{ Throwable -> 0x066f, all -> 0x0663 }
            java.lang.String r6 = "import static "
            r1.append(r6)     // Catch:{ Throwable -> 0x066f, all -> 0x0663 }
            r1.append(r9)     // Catch:{ Throwable -> 0x066f, all -> 0x0663 }
            java.lang.String r6 = ".*;\n"
            r1.append(r6)     // Catch:{ Throwable -> 0x066f, all -> 0x0663 }
            if (r0 == 0) goto L_0x05cf
            boolean r6 = r0.comment     // Catch:{ Throwable -> 0x05c6, all -> 0x05bc }
            if (r6 == 0) goto L_0x05cf
            java.lang.String r6 = r0.text     // Catch:{ Throwable -> 0x05c6, all -> 0x05bc }
            goto L_0x05d1
        L_0x05bc:
            r0 = move-exception
            r1 = r0
            r44 = r5
            r45 = r8
            r46 = r9
            goto L_0x06c1
        L_0x05c6:
            r0 = move-exception
            r44 = r5
            r45 = r8
            r46 = r9
            goto L_0x06c5
        L_0x05cf:
            java.lang.String r6 = ""
        L_0x05d1:
            r1.append(r6)     // Catch:{ Throwable -> 0x066f, all -> 0x0663 }
            java.lang.String r6 = r4.text     // Catch:{ Throwable -> 0x066f, all -> 0x0663 }
            r44 = r5
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x065b, all -> 0x0651 }
            r5.<init>()     // Catch:{ Throwable -> 0x065b, all -> 0x0651 }
            r45 = r8
            java.lang.String r8 = "public static class "
            r5.append(r8)     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            org.bytedeco.javacpp.tools.Type r8 = r4.type     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            java.lang.String r8 = r8.javaName     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            r5.append(r8)     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            java.lang.String r8 = " "
            r5.append(r8)     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            java.lang.String r5 = r5.toString()     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            r8.<init>()     // Catch:{ Throwable -> 0x064b, all -> 0x0643 }
            r46 = r9
            java.lang.String r9 = "@Properties(inherit = "
            r8.append(r9)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r9 = r50.getCanonicalName()     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r8.append(r9)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r9 = ".class)\npublic class "
            r8.append(r9)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            org.bytedeco.javacpp.tools.Type r9 = r4.type     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r9 = r9.javaName     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r8.append(r9)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r9 = " "
            r8.append(r9)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r8 = r8.toString()     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r5 = r6.replace(r5, r8)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r1.append(r5)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r1 = r1.toString()     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.nio.file.Path r5 = r3.toPath()     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.lang.String r6 = r7.encoding     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            if (r6 == 0) goto L_0x0636
            java.lang.String r6 = r7.encoding     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            byte[] r6 = r1.getBytes(r6)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            goto L_0x063a
        L_0x0636:
            byte[] r6 = r1.getBytes()     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
        L_0x063a:
            r8 = 0
            java.nio.file.OpenOption[] r9 = new java.nio.file.OpenOption[r8]     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            java.nio.file.Files.write(r5, r6, r9)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r0 = 0
            goto L_0x06a2
        L_0x0643:
            r0 = move-exception
            r46 = r9
            r1 = r0
            r5 = r28
            goto L_0x06ee
        L_0x064b:
            r0 = move-exception
            r46 = r9
            r5 = r0
            goto L_0x06eb
        L_0x0651:
            r0 = move-exception
            r45 = r8
            r46 = r9
            r1 = r0
            r5 = r28
            goto L_0x06ee
        L_0x065b:
            r0 = move-exception
            r45 = r8
            r46 = r9
            r5 = r0
            goto L_0x06eb
        L_0x0663:
            r0 = move-exception
            r44 = r5
            r45 = r8
            r46 = r9
            r1 = r0
            r5 = r28
            goto L_0x06ee
        L_0x066f:
            r0 = move-exception
            r44 = r5
            r45 = r8
            r46 = r9
            r5 = r0
            goto L_0x06eb
        L_0x0679:
            r0 = move-exception
            r45 = r8
            r46 = r9
            r44 = r41
            r1 = r0
            r5 = r28
            goto L_0x06ee
        L_0x0685:
            r0 = move-exception
            r45 = r8
            r46 = r9
            r44 = r41
            r5 = r0
            goto L_0x06eb
        L_0x068f:
            r43 = r1
            r42 = r3
            r45 = r8
            r46 = r9
            r44 = r41
            r8 = 0
            if (r0 == 0) goto L_0x06a1
            java.lang.String r1 = r0.text     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r2.append(r1)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
        L_0x06a1:
            r0 = r4
        L_0x06a2:
            r3 = r42
            r1 = r43
            r41 = r44
            r8 = r45
            r9 = r46
            goto L_0x0519
        L_0x06af:
            r43 = r1
            r45 = r8
            r46 = r9
            r44 = r41
            if (r0 == 0) goto L_0x06c7
            java.lang.String r1 = r0.text     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r2.append(r1)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            goto L_0x06c7
        L_0x06bf:
            r0 = move-exception
            r1 = r0
        L_0x06c1:
            r5 = r28
            goto L_0x06ee
        L_0x06c4:
            r0 = move-exception
        L_0x06c5:
            r5 = r0
            goto L_0x06eb
        L_0x06c7:
            java.lang.String r1 = "\n}\n"
            java.io.Writer r1 = r2.append(r1)     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r1.close()     // Catch:{ Throwable -> 0x06c4, all -> 0x06bf }
            r2.close()
            return r10
        L_0x06d4:
            r0 = move-exception
            r43 = r1
            r45 = r8
            r46 = r9
            r44 = r41
            r1 = r0
            r5 = r28
            goto L_0x06ee
        L_0x06e1:
            r0 = move-exception
            r43 = r1
            r45 = r8
            r46 = r9
            r44 = r41
            r5 = r0
        L_0x06eb:
            throw r5     // Catch:{ all -> 0x06ec }
        L_0x06ec:
            r0 = move-exception
        L_0x06ed:
            r1 = r0
        L_0x06ee:
            if (r5 == 0) goto L_0x06fa
            r2.close()     // Catch:{ Throwable -> 0x06f4 }
            goto L_0x06fd
        L_0x06f4:
            r0 = move-exception
            r3 = r0
            r5.addSuppressed(r3)
            goto L_0x06fd
        L_0x06fa:
            r2.close()
        L_0x06fd:
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Parser.parse(java.io.File, java.lang.String[], java.lang.Class):java.io.File");
    }
}
