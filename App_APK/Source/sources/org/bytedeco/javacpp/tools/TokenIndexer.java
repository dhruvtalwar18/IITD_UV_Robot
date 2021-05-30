package org.bytedeco.javacpp.tools;

import java.util.ArrayList;
import java.util.List;

class TokenIndexer {
    Token[] array = null;
    int counter = 0;
    int index = 0;
    InfoMap infoMap = null;
    final boolean isCFile;
    boolean raw = false;

    TokenIndexer(InfoMap infoMap2, Token[] array2, boolean isCFile2) {
        this.infoMap = infoMap2;
        this.array = array2;
        this.isCFile = isCFile2;
    }

    /* access modifiers changed from: package-private */
    public Token[] filter(Token[] array2, int index2) {
        String value;
        String value2;
        String value3;
        String str;
        Token[] array3 = array2;
        int i = index2;
        if (i + 1 < array3.length) {
            int i2 = 1;
            char c = '#';
            int i3 = 0;
            if (array3[i].match('#')) {
                int i4 = 3;
                if (array3[i + 1].match(Token.IF, Token.IFDEF, Token.IFNDEF)) {
                    List<Token> tokens = new ArrayList<>();
                    for (int i5 = 0; i5 < i; i5++) {
                        tokens.add(array3[i5]);
                    }
                    boolean define = true;
                    Info info = null;
                    int count = 0;
                    int index3 = i;
                    boolean defined = false;
                    while (true) {
                        boolean defined2 = defined;
                        if (index3 >= array3.length) {
                            break;
                        }
                        String spacing = array3[index3].spacing;
                        int n = spacing.lastIndexOf(10) + i2;
                        Token keyword = null;
                        Token token = array3[index3];
                        Object[] objArr = new Object[i2];
                        objArr[i3] = Character.valueOf(c);
                        boolean match = token.match(objArr);
                        int i6 = 4;
                        if (match) {
                            Token token2 = array3[index3 + 1];
                            Object[] objArr2 = new Object[i4];
                            objArr2[i3] = Token.IF;
                            objArr2[i2] = Token.IFDEF;
                            objArr2[2] = Token.IFNDEF;
                            if (token2.match(objArr2)) {
                                count++;
                            }
                            if (count == i2) {
                                Token token3 = array3[index3 + 1];
                                Object[] objArr3 = new Object[6];
                                objArr3[i3] = Token.IF;
                                objArr3[i2] = Token.IFDEF;
                                objArr3[2] = Token.IFNDEF;
                                objArr3[i4] = Token.ELIF;
                                objArr3[4] = Token.ELSE;
                                objArr3[5] = Token.ENDIF;
                                if (token3.match(objArr3)) {
                                    keyword = array3[index3 + 1];
                                }
                            }
                            Token token4 = array3[index3 + 1];
                            Object[] objArr4 = new Object[i2];
                            objArr4[i3] = Token.ENDIF;
                            if (token4.match(objArr4)) {
                                count--;
                            }
                        }
                        if (keyword != null) {
                            index3 += 2;
                            Token comment = new Token();
                            comment.type = 4;
                            comment.spacing = spacing.substring(i3, n);
                            comment.value = "// " + spacing.substring(n) + "#" + keyword.spacing + keyword;
                            tokens.add(comment);
                            String value4 = "";
                            while (true) {
                                value = value4;
                                if (index3 < array3.length && array3[index3].spacing.indexOf(10) < 0) {
                                    Token token5 = array3[index3];
                                    Object[] objArr5 = new Object[i2];
                                    objArr5[0] = Integer.valueOf(i6);
                                    if (!token5.match(objArr5)) {
                                        value2 = value + array3[index3].spacing + array3[index3];
                                    } else {
                                        value2 = value;
                                    }
                                    StringBuilder sb = new StringBuilder();
                                    sb.append(comment.value);
                                    Token token6 = array3[index3];
                                    Object[] objArr6 = new Object[i2];
                                    objArr6[0] = "\n";
                                    if (token6.match(objArr6)) {
                                        str = "\n// ";
                                        value3 = value2;
                                    } else {
                                        StringBuilder sb2 = new StringBuilder();
                                        sb2.append(array3[index3].spacing);
                                        value3 = value2;
                                        sb2.append(array3[index3].toString().replaceAll("\n", "\n// "));
                                        str = sb2.toString();
                                    }
                                    sb.append(str);
                                    comment.value = sb.toString();
                                    index3++;
                                    value4 = value3;
                                    i2 = 1;
                                    i6 = 4;
                                }
                            }
                            if (!keyword.match(Token.IF, Token.IFDEF, Token.IFNDEF, Token.ELIF)) {
                                if (!keyword.match(Token.ELSE)) {
                                    if (keyword.match(Token.ENDIF) && count == 0) {
                                        break;
                                    }
                                } else {
                                    define = info == null || !define;
                                }
                            } else {
                                define = info == null || !defined2;
                                info = this.infoMap.getFirst(value);
                                if (info != null) {
                                    define = keyword.match(Token.IFNDEF) ? !info.define : info.define;
                                } else {
                                    try {
                                        define = Integer.decode(value.trim()).intValue() != 0;
                                    } catch (NumberFormatException e) {
                                    }
                                }
                            }
                        } else {
                            if (define) {
                                tokens.add(array3[index3]);
                                index3++;
                            } else {
                                index3++;
                            }
                        }
                        defined = define || defined2;
                        i2 = 1;
                        i4 = 3;
                        c = '#';
                        i3 = 0;
                    }
                    while (index3 < array3.length) {
                        tokens.add(array3[index3]);
                        index3++;
                    }
                    return (Token[]) tokens.toArray(new Token[tokens.size()]);
                }
            }
        }
        int i7 = i;
        return array3;
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:60:0x0130, code lost:
        if (r2[r3].match('(') == false) goto L_0x0132;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.bytedeco.javacpp.tools.Token[] expand(org.bytedeco.javacpp.tools.Token[] r29, int r30) {
        /*
            r28 = this;
            r1 = r28
            r2 = r29
            r3 = r30
            int r0 = r2.length
            if (r3 >= r0) goto L_0x039b
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            r4 = r2[r3]
            java.lang.String r4 = r4.value
            boolean r0 = r0.containsKey(r4)
            if (r0 == 0) goto L_0x039b
            r4 = r30
            org.bytedeco.javacpp.tools.InfoMap r0 = r1.infoMap
            r5 = r2[r3]
            java.lang.String r5 = r5.value
            java.util.List r5 = r0.get(r5)
            r0 = 0
            java.util.Iterator r6 = r5.iterator()
            r7 = r0
        L_0x0027:
            boolean r0 = r6.hasNext()
            if (r0 == 0) goto L_0x003c
            java.lang.Object r0 = r6.next()
            org.bytedeco.javacpp.tools.Info r0 = (org.bytedeco.javacpp.tools.Info) r0
            if (r0 == 0) goto L_0x003b
            java.lang.String r8 = r0.cppText
            if (r8 == 0) goto L_0x003b
            r7 = r0
        L_0x003b:
            goto L_0x0027
        L_0x003c:
            if (r7 == 0) goto L_0x039b
            java.lang.String r0 = r7.cppText
            if (r0 == 0) goto L_0x039b
            org.bytedeco.javacpp.tools.Tokenizer r0 = new org.bytedeco.javacpp.tools.Tokenizer     // Catch:{ IOException -> 0x0392 }
            java.lang.String r6 = r7.cppText     // Catch:{ IOException -> 0x0392 }
            r8 = r2[r3]     // Catch:{ IOException -> 0x0392 }
            java.io.File r8 = r8.file     // Catch:{ IOException -> 0x0392 }
            r9 = r2[r3]     // Catch:{ IOException -> 0x0392 }
            int r9 = r9.lineNumber     // Catch:{ IOException -> 0x0392 }
            r0.<init>((java.lang.String) r6, (java.io.File) r8, (int) r9)     // Catch:{ IOException -> 0x0392 }
            org.bytedeco.javacpp.tools.Token r6 = r0.nextToken()     // Catch:{ IOException -> 0x0392 }
            r8 = 1
            java.lang.Object[] r9 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x0392 }
            r10 = 35
            java.lang.Character r10 = java.lang.Character.valueOf(r10)     // Catch:{ IOException -> 0x0392 }
            r11 = 0
            r9[r11] = r10     // Catch:{ IOException -> 0x0392 }
            boolean r6 = r6.match(r9)     // Catch:{ IOException -> 0x0392 }
            if (r6 == 0) goto L_0x038f
            org.bytedeco.javacpp.tools.Token r6 = r0.nextToken()     // Catch:{ IOException -> 0x0392 }
            java.lang.Object[] r9 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x0392 }
            org.bytedeco.javacpp.tools.Token r10 = org.bytedeco.javacpp.tools.Token.DEFINE     // Catch:{ IOException -> 0x0392 }
            r9[r11] = r10     // Catch:{ IOException -> 0x0392 }
            boolean r6 = r6.match(r9)     // Catch:{ IOException -> 0x0392 }
            if (r6 == 0) goto L_0x038f
            org.bytedeco.javacpp.tools.Token r6 = r0.nextToken()     // Catch:{ IOException -> 0x0392 }
            java.lang.Object[] r9 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x0392 }
            java.lang.String[] r10 = r7.cppNames     // Catch:{ IOException -> 0x0392 }
            r10 = r10[r11]     // Catch:{ IOException -> 0x0392 }
            r9[r11] = r10     // Catch:{ IOException -> 0x0392 }
            boolean r6 = r6.match(r9)     // Catch:{ IOException -> 0x0392 }
            if (r6 != 0) goto L_0x008d
            r21 = r5
            goto L_0x0391
        L_0x008d:
            java.util.ArrayList r6 = new java.util.ArrayList     // Catch:{ IOException -> 0x0392 }
            r6.<init>()     // Catch:{ IOException -> 0x0392 }
            r9 = 0
        L_0x0093:
            if (r9 >= r3) goto L_0x00a2
            r10 = r2[r9]     // Catch:{ IOException -> 0x009d }
            r6.add(r10)     // Catch:{ IOException -> 0x009d }
            int r9 = r9 + 1
            goto L_0x0093
        L_0x009d:
            r0 = move-exception
            r21 = r5
            goto L_0x0395
        L_0x00a2:
            java.util.ArrayList r9 = new java.util.ArrayList     // Catch:{ IOException -> 0x0392 }
            r9.<init>()     // Catch:{ IOException -> 0x0392 }
            r10 = 0
            org.bytedeco.javacpp.tools.Token r12 = r0.nextToken()     // Catch:{ IOException -> 0x0392 }
            java.lang.String[] r13 = r7.cppNames     // Catch:{ IOException -> 0x0392 }
            r13 = r13[r11]     // Catch:{ IOException -> 0x0392 }
            java.lang.String r14 = "__COUNTER__"
            boolean r13 = r13.equals(r14)     // Catch:{ IOException -> 0x0392 }
            if (r13 == 0) goto L_0x00c4
            int r13 = r1.counter     // Catch:{ IOException -> 0x009d }
            int r14 = r13 + 1
            r1.counter = r14     // Catch:{ IOException -> 0x009d }
            java.lang.String r13 = java.lang.Integer.toString(r13)     // Catch:{ IOException -> 0x009d }
            r12.value = r13     // Catch:{ IOException -> 0x009d }
        L_0x00c4:
            r13 = r2[r3]     // Catch:{ IOException -> 0x0392 }
            java.lang.String r13 = r13.value     // Catch:{ IOException -> 0x0392 }
            java.lang.Object[] r14 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x0392 }
            r15 = 40
            java.lang.Character r16 = java.lang.Character.valueOf(r15)     // Catch:{ IOException -> 0x0392 }
            r14[r11] = r16     // Catch:{ IOException -> 0x0392 }
            boolean r14 = r12.match(r14)     // Catch:{ IOException -> 0x0392 }
            if (r14 == 0) goto L_0x025c
            org.bytedeco.javacpp.tools.Token r14 = r0.nextToken()     // Catch:{ IOException -> 0x0392 }
            r12 = r14
        L_0x00dd:
            boolean r14 = r12.isEmpty()     // Catch:{ IOException -> 0x0392 }
            r16 = 41
            if (r14 != 0) goto L_0x0115
            java.lang.Object[] r14 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x009d }
            r17 = 5
            java.lang.Integer r17 = java.lang.Integer.valueOf(r17)     // Catch:{ IOException -> 0x009d }
            r14[r11] = r17     // Catch:{ IOException -> 0x009d }
            boolean r14 = r12.match(r14)     // Catch:{ IOException -> 0x009d }
            if (r14 == 0) goto L_0x00fb
            java.lang.String r14 = r12.value     // Catch:{ IOException -> 0x009d }
            r9.add(r14)     // Catch:{ IOException -> 0x009d }
            goto L_0x010f
        L_0x00fb:
            java.lang.Object[] r14 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x009d }
            java.lang.Character r17 = java.lang.Character.valueOf(r16)     // Catch:{ IOException -> 0x009d }
            r14[r11] = r17     // Catch:{ IOException -> 0x009d }
            boolean r14 = r12.match(r14)     // Catch:{ IOException -> 0x009d }
            if (r14 == 0) goto L_0x010f
            org.bytedeco.javacpp.tools.Token r14 = r0.nextToken()     // Catch:{ IOException -> 0x009d }
            r12 = r14
            goto L_0x0115
        L_0x010f:
            org.bytedeco.javacpp.tools.Token r14 = r0.nextToken()     // Catch:{ IOException -> 0x009d }
            r12 = r14
            goto L_0x00dd
        L_0x0115:
            int r3 = r3 + 1
            int r14 = r9.size()     // Catch:{ IOException -> 0x0392 }
            if (r14 <= 0) goto L_0x0133
            int r14 = r2.length     // Catch:{ IOException -> 0x009d }
            if (r3 >= r14) goto L_0x0132
            r14 = r2[r3]     // Catch:{ IOException -> 0x009d }
            java.lang.Object[] r11 = new java.lang.Object[r8]     // Catch:{ IOException -> 0x009d }
            java.lang.Character r17 = java.lang.Character.valueOf(r15)     // Catch:{ IOException -> 0x009d }
            r18 = 0
            r11[r18] = r17     // Catch:{ IOException -> 0x009d }
            boolean r11 = r14.match(r11)     // Catch:{ IOException -> 0x009d }
            if (r11 != 0) goto L_0x0133
        L_0x0132:
            return r2
        L_0x0133:
            java.lang.StringBuilder r11 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x0392 }
            r11.<init>()     // Catch:{ IOException -> 0x0392 }
            r11.append(r13)     // Catch:{ IOException -> 0x0392 }
            r14 = r2[r3]     // Catch:{ IOException -> 0x0392 }
            java.lang.String r14 = r14.spacing     // Catch:{ IOException -> 0x0392 }
            r11.append(r14)     // Catch:{ IOException -> 0x0392 }
            r14 = r2[r3]     // Catch:{ IOException -> 0x0392 }
            r11.append(r14)     // Catch:{ IOException -> 0x0392 }
            java.lang.String r11 = r11.toString()     // Catch:{ IOException -> 0x0392 }
            int r13 = r9.size()     // Catch:{ IOException -> 0x0392 }
            java.util.List[] r13 = new java.util.List[r13]     // Catch:{ IOException -> 0x0392 }
            r10 = r13
            r13 = 0
            r14 = 0
            int r3 = r3 + 1
        L_0x0156:
            int r15 = r2.length     // Catch:{ IOException -> 0x0255 }
            if (r3 >= r15) goto L_0x020c
            r15 = r2[r3]     // Catch:{ IOException -> 0x0392 }
            java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x0392 }
            r8.<init>()     // Catch:{ IOException -> 0x0392 }
            r8.append(r11)     // Catch:{ IOException -> 0x0392 }
            r21 = r5
            java.lang.String r5 = r15.spacing     // Catch:{ IOException -> 0x038d }
            r8.append(r5)     // Catch:{ IOException -> 0x038d }
            r8.append(r15)     // Catch:{ IOException -> 0x038d }
            java.lang.String r5 = r8.toString()     // Catch:{ IOException -> 0x038d }
            r11 = r5
            if (r14 != 0) goto L_0x0187
            r5 = 1
            java.lang.Object[] r8 = new java.lang.Object[r5]     // Catch:{ IOException -> 0x038d }
            java.lang.Character r5 = java.lang.Character.valueOf(r16)     // Catch:{ IOException -> 0x038d }
            r17 = 0
            r8[r17] = r5     // Catch:{ IOException -> 0x038d }
            boolean r5 = r15.match(r8)     // Catch:{ IOException -> 0x038d }
            if (r5 == 0) goto L_0x0187
            goto L_0x020e
        L_0x0187:
            if (r14 != 0) goto L_0x01a2
            r5 = 1
            java.lang.Object[] r8 = new java.lang.Object[r5]     // Catch:{ IOException -> 0x038d }
            r5 = 44
            java.lang.Character r5 = java.lang.Character.valueOf(r5)     // Catch:{ IOException -> 0x038d }
            r17 = 0
            r8[r17] = r5     // Catch:{ IOException -> 0x038d }
            boolean r5 = r15.match(r8)     // Catch:{ IOException -> 0x038d }
            if (r5 == 0) goto L_0x01a2
            int r13 = r13 + 1
            r17 = 40
            goto L_0x0203
        L_0x01a2:
            r5 = 3
            java.lang.Object[] r8 = new java.lang.Object[r5]     // Catch:{ IOException -> 0x038d }
            r17 = 40
            java.lang.Character r19 = java.lang.Character.valueOf(r17)     // Catch:{ IOException -> 0x038d }
            r18 = 0
            r8[r18] = r19     // Catch:{ IOException -> 0x038d }
            r19 = 91
            java.lang.Character r19 = java.lang.Character.valueOf(r19)     // Catch:{ IOException -> 0x038d }
            r20 = 1
            r8[r20] = r19     // Catch:{ IOException -> 0x038d }
            r19 = 123(0x7b, float:1.72E-43)
            java.lang.Character r19 = java.lang.Character.valueOf(r19)     // Catch:{ IOException -> 0x038d }
            r22 = 2
            r8[r22] = r19     // Catch:{ IOException -> 0x038d }
            boolean r8 = r15.match(r8)     // Catch:{ IOException -> 0x038d }
            if (r8 == 0) goto L_0x01cc
            int r14 = r14 + 1
            goto L_0x01f0
        L_0x01cc:
            java.lang.Object[] r5 = new java.lang.Object[r5]     // Catch:{ IOException -> 0x038d }
            java.lang.Character r8 = java.lang.Character.valueOf(r16)     // Catch:{ IOException -> 0x038d }
            r18 = 0
            r5[r18] = r8     // Catch:{ IOException -> 0x038d }
            r8 = 93
            java.lang.Character r8 = java.lang.Character.valueOf(r8)     // Catch:{ IOException -> 0x038d }
            r19 = 1
            r5[r19] = r8     // Catch:{ IOException -> 0x038d }
            r8 = 125(0x7d, float:1.75E-43)
            java.lang.Character r8 = java.lang.Character.valueOf(r8)     // Catch:{ IOException -> 0x038d }
            r5[r22] = r8     // Catch:{ IOException -> 0x038d }
            boolean r5 = r15.match(r5)     // Catch:{ IOException -> 0x038d }
            if (r5 == 0) goto L_0x01f0
            int r14 = r14 + -1
        L_0x01f0:
            int r5 = r10.length     // Catch:{ IOException -> 0x038d }
            if (r13 >= r5) goto L_0x0203
            r5 = r10[r13]     // Catch:{ IOException -> 0x038d }
            if (r5 != 0) goto L_0x01fe
            java.util.ArrayList r5 = new java.util.ArrayList     // Catch:{ IOException -> 0x038d }
            r5.<init>()     // Catch:{ IOException -> 0x038d }
            r10[r13] = r5     // Catch:{ IOException -> 0x038d }
        L_0x01fe:
            r5 = r10[r13]     // Catch:{ IOException -> 0x038d }
            r5.add(r15)     // Catch:{ IOException -> 0x038d }
        L_0x0203:
            int r3 = r3 + 1
            r5 = r21
            r8 = 1
            r15 = 40
            goto L_0x0156
        L_0x020c:
            r21 = r5
        L_0x020e:
            r5 = 0
        L_0x020f:
            int r8 = r10.length     // Catch:{ IOException -> 0x0250 }
            if (r5 >= r8) goto L_0x024c
            org.bytedeco.javacpp.tools.InfoMap r8 = r1.infoMap     // Catch:{ IOException -> 0x0250 }
            r15 = r10[r5]     // Catch:{ IOException -> 0x0250 }
            r23 = r3
            r3 = 0
            java.lang.Object r15 = r15.get(r3)     // Catch:{ IOException -> 0x0247 }
            org.bytedeco.javacpp.tools.Token r15 = (org.bytedeco.javacpp.tools.Token) r15     // Catch:{ IOException -> 0x0247 }
            java.lang.String r3 = r15.value     // Catch:{ IOException -> 0x0247 }
            boolean r3 = r8.containsKey(r3)     // Catch:{ IOException -> 0x0247 }
            if (r3 == 0) goto L_0x0242
            r3 = r10[r5]     // Catch:{ IOException -> 0x0247 }
            r8 = r10[r5]     // Catch:{ IOException -> 0x0247 }
            int r8 = r8.size()     // Catch:{ IOException -> 0x0247 }
            org.bytedeco.javacpp.tools.Token[] r8 = new org.bytedeco.javacpp.tools.Token[r8]     // Catch:{ IOException -> 0x0247 }
            java.lang.Object[] r3 = r3.toArray(r8)     // Catch:{ IOException -> 0x0247 }
            org.bytedeco.javacpp.tools.Token[] r3 = (org.bytedeco.javacpp.tools.Token[]) r3     // Catch:{ IOException -> 0x0247 }
            r8 = 0
            org.bytedeco.javacpp.tools.Token[] r3 = r1.expand(r3, r8)     // Catch:{ IOException -> 0x0247 }
            java.util.List r3 = java.util.Arrays.asList(r3)     // Catch:{ IOException -> 0x0247 }
            r10[r5] = r3     // Catch:{ IOException -> 0x0247 }
        L_0x0242:
            int r5 = r5 + 1
            r3 = r23
            goto L_0x020f
        L_0x0247:
            r0 = move-exception
            r3 = r23
            goto L_0x0395
        L_0x024c:
            r23 = r3
            r13 = r11
            goto L_0x025e
        L_0x0250:
            r0 = move-exception
            r23 = r3
            goto L_0x0395
        L_0x0255:
            r0 = move-exception
            r23 = r3
            r21 = r5
            goto L_0x0395
        L_0x025c:
            r21 = r5
        L_0x025e:
            int r5 = r6.size()     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.InfoMap r8 = r1.infoMap     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Info r8 = r8.getFirst(r13)     // Catch:{ IOException -> 0x038d }
            r7 = r8
        L_0x0269:
            if (r7 == 0) goto L_0x0274
            boolean r8 = r7.skip     // Catch:{ IOException -> 0x038d }
            if (r8 != 0) goto L_0x0270
            goto L_0x0274
        L_0x0270:
            r27 = r9
            goto L_0x0307
        L_0x0274:
            boolean r8 = r12.isEmpty()     // Catch:{ IOException -> 0x038d }
            if (r8 != 0) goto L_0x0305
            r8 = 0
            r11 = 0
        L_0x027c:
            int r14 = r9.size()     // Catch:{ IOException -> 0x038d }
            if (r11 >= r14) goto L_0x02e8
            java.lang.Object r14 = r9.get(r11)     // Catch:{ IOException -> 0x038d }
            java.lang.String r14 = (java.lang.String) r14     // Catch:{ IOException -> 0x038d }
            java.lang.String r15 = r12.value     // Catch:{ IOException -> 0x038d }
            boolean r14 = r14.equals(r15)     // Catch:{ IOException -> 0x038d }
            if (r14 == 0) goto L_0x02df
            java.lang.String r14 = r12.spacing     // Catch:{ IOException -> 0x038d }
            r15 = r10[r11]     // Catch:{ IOException -> 0x038d }
            java.util.Iterator r15 = r15.iterator()     // Catch:{ IOException -> 0x038d }
        L_0x0298:
            boolean r16 = r15.hasNext()     // Catch:{ IOException -> 0x038d }
            if (r16 == 0) goto L_0x02d9
            java.lang.Object r16 = r15.next()     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token r16 = (org.bytedeco.javacpp.tools.Token) r16     // Catch:{ IOException -> 0x038d }
            r24 = r16
            org.bytedeco.javacpp.tools.Token r1 = new org.bytedeco.javacpp.tools.Token     // Catch:{ IOException -> 0x038d }
            r25 = r8
            r8 = r24
            r1.<init>(r8)     // Catch:{ IOException -> 0x038d }
            if (r14 == 0) goto L_0x02c9
            r26 = r8
            java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x038d }
            r8.<init>()     // Catch:{ IOException -> 0x038d }
            r27 = r9
            java.lang.String r9 = r1.spacing     // Catch:{ IOException -> 0x038d }
            r8.append(r9)     // Catch:{ IOException -> 0x038d }
            r8.append(r14)     // Catch:{ IOException -> 0x038d }
            java.lang.String r8 = r8.toString()     // Catch:{ IOException -> 0x038d }
            r1.spacing = r8     // Catch:{ IOException -> 0x038d }
            goto L_0x02cd
        L_0x02c9:
            r26 = r8
            r27 = r9
        L_0x02cd:
            r6.add(r1)     // Catch:{ IOException -> 0x038d }
            r14 = 0
            r8 = r25
            r9 = r27
            r1 = r28
            goto L_0x0298
        L_0x02d9:
            r25 = r8
            r27 = r9
            r8 = 1
            goto L_0x02ec
        L_0x02df:
            r25 = r8
            r27 = r9
            int r11 = r11 + 1
            r1 = r28
            goto L_0x027c
        L_0x02e8:
            r25 = r8
            r27 = r9
        L_0x02ec:
            if (r8 != 0) goto L_0x02f9
            int r1 = r12.type     // Catch:{ IOException -> 0x038d }
            r9 = -1
            if (r1 != r9) goto L_0x02f6
            r1 = 4
            r12.type = r1     // Catch:{ IOException -> 0x038d }
        L_0x02f6:
            r6.add(r12)     // Catch:{ IOException -> 0x038d }
        L_0x02f9:
            org.bytedeco.javacpp.tools.Token r1 = r0.nextToken()     // Catch:{ IOException -> 0x038d }
            r12 = r1
            r9 = r27
            r1 = r28
            goto L_0x0269
        L_0x0305:
            r27 = r9
        L_0x0307:
            r1 = r5
        L_0x0308:
            int r8 = r6.size()     // Catch:{ IOException -> 0x038d }
            if (r1 >= r8) goto L_0x035c
            java.lang.Object r8 = r6.get(r1)     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token r8 = (org.bytedeco.javacpp.tools.Token) r8     // Catch:{ IOException -> 0x038d }
            r9 = 1
            java.lang.Object[] r11 = new java.lang.Object[r9]     // Catch:{ IOException -> 0x038d }
            java.lang.String r9 = "##"
            r14 = 0
            r11[r14] = r9     // Catch:{ IOException -> 0x038d }
            boolean r8 = r8.match(r11)     // Catch:{ IOException -> 0x038d }
            if (r8 == 0) goto L_0x0359
            if (r1 <= 0) goto L_0x0359
            int r8 = r1 + 1
            int r9 = r6.size()     // Catch:{ IOException -> 0x038d }
            if (r8 >= r9) goto L_0x0359
            java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x038d }
            r8.<init>()     // Catch:{ IOException -> 0x038d }
            int r9 = r1 + -1
            java.lang.Object r9 = r6.get(r9)     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token r9 = (org.bytedeco.javacpp.tools.Token) r9     // Catch:{ IOException -> 0x038d }
            java.lang.String r11 = r9.value     // Catch:{ IOException -> 0x038d }
            r8.append(r11)     // Catch:{ IOException -> 0x038d }
            int r11 = r1 + 1
            java.lang.Object r11 = r6.get(r11)     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token r11 = (org.bytedeco.javacpp.tools.Token) r11     // Catch:{ IOException -> 0x038d }
            java.lang.String r11 = r11.value     // Catch:{ IOException -> 0x038d }
            r8.append(r11)     // Catch:{ IOException -> 0x038d }
            java.lang.String r8 = r8.toString()     // Catch:{ IOException -> 0x038d }
            r9.value = r8     // Catch:{ IOException -> 0x038d }
            r6.remove(r1)     // Catch:{ IOException -> 0x038d }
            r6.remove(r1)     // Catch:{ IOException -> 0x038d }
            int r1 = r1 + -1
        L_0x0359:
            r8 = 1
            int r1 = r1 + r8
            goto L_0x0308
        L_0x035c:
            int r3 = r3 + 1
            int r1 = r2.length     // Catch:{ IOException -> 0x038d }
            if (r3 >= r1) goto L_0x0367
            r1 = r2[r3]     // Catch:{ IOException -> 0x038d }
            r6.add(r1)     // Catch:{ IOException -> 0x038d }
            goto L_0x035c
        L_0x0367:
            if (r7 == 0) goto L_0x036d
            boolean r1 = r7.skip     // Catch:{ IOException -> 0x038d }
            if (r1 != 0) goto L_0x037f
        L_0x036d:
            int r1 = r6.size()     // Catch:{ IOException -> 0x038d }
            if (r5 >= r1) goto L_0x037f
            java.lang.Object r1 = r6.get(r5)     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token r1 = (org.bytedeco.javacpp.tools.Token) r1     // Catch:{ IOException -> 0x038d }
            r8 = r2[r4]     // Catch:{ IOException -> 0x038d }
            java.lang.String r8 = r8.spacing     // Catch:{ IOException -> 0x038d }
            r1.spacing = r8     // Catch:{ IOException -> 0x038d }
        L_0x037f:
            int r1 = r6.size()     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token[] r1 = new org.bytedeco.javacpp.tools.Token[r1]     // Catch:{ IOException -> 0x038d }
            java.lang.Object[] r1 = r6.toArray(r1)     // Catch:{ IOException -> 0x038d }
            org.bytedeco.javacpp.tools.Token[] r1 = (org.bytedeco.javacpp.tools.Token[]) r1     // Catch:{ IOException -> 0x038d }
            r0 = r1
            goto L_0x039c
        L_0x038d:
            r0 = move-exception
            goto L_0x0395
        L_0x038f:
            r21 = r5
        L_0x0391:
            return r2
        L_0x0392:
            r0 = move-exception
            r21 = r5
        L_0x0395:
            java.lang.RuntimeException r1 = new java.lang.RuntimeException
            r1.<init>(r0)
            throw r1
        L_0x039b:
            r0 = r2
        L_0x039c:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.TokenIndexer.expand(org.bytedeco.javacpp.tools.Token[], int):org.bytedeco.javacpp.tools.Token[]");
    }

    /* access modifiers changed from: package-private */
    public int preprocess(int index2, int count) {
        Token[] a;
        while (true) {
            a = null;
            if (index2 >= this.array.length) {
                break;
            }
            Token[] a2 = null;
            while (a2 != this.array) {
                a2 = this.array;
                this.array = filter(this.array, index2);
                this.array = expand(this.array, index2);
            }
            if (!this.array[index2].match(4) && count - 1 < 0) {
                break;
            }
            index2++;
        }
        while (a != this.array) {
            a = this.array;
            this.array = filter(this.array, index2);
            this.array = expand(this.array, index2);
        }
        return index2;
    }

    /* access modifiers changed from: package-private */
    public Token get() {
        return get(0);
    }

    /* access modifiers changed from: package-private */
    public Token get(int i) {
        int k = this.raw ? this.index + i : preprocess(this.index, i);
        if (k < this.array.length) {
            return this.array[k];
        }
        return this.array[this.array.length - 1].match(Token.EOF) ? this.array[this.array.length - 1] : Token.EOF;
    }

    /* access modifiers changed from: package-private */
    public Token next() {
        this.index = this.raw ? this.index + 1 : preprocess(this.index, 1);
        if (this.index < this.array.length) {
            return this.array[this.index];
        }
        return this.array[this.array.length - 1].match(Token.EOF) ? this.array[this.array.length - 1] : Token.EOF;
    }
}
