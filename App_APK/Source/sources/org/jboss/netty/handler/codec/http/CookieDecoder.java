package org.jboss.netty.handler.codec.http;

public class CookieDecoder {
    private static final String COMMA = ",";

    public CookieDecoder() {
    }

    @Deprecated
    public CookieDecoder(boolean lenient) {
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v12, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v7, resolved type: java.lang.String} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.util.Set<org.jboss.netty.handler.codec.http.Cookie> decode(java.lang.String r33) {
        /*
            r32 = this;
            java.util.ArrayList r0 = new java.util.ArrayList
            r1 = 8
            r0.<init>(r1)
            r2 = r0
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>(r1)
            r1 = r0
            r3 = r33
            extractKeyValuePairs(r3, r2, r1)
            boolean r0 = r2.isEmpty()
            if (r0 == 0) goto L_0x001e
            java.util.Set r0 = java.util.Collections.emptySet()
            return r0
        L_0x001e:
            r4 = 0
            r5 = 0
            java.lang.Object r0 = r2.get(r5)
            java.lang.String r0 = (java.lang.String) r0
            java.lang.String r6 = "Version"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x003e
            java.lang.Object r0 = r1.get(r5)     // Catch:{ NumberFormatException -> 0x003b }
            java.lang.String r0 = (java.lang.String) r0     // Catch:{ NumberFormatException -> 0x003b }
            int r0 = java.lang.Integer.parseInt(r0)     // Catch:{ NumberFormatException -> 0x003b }
            r4 = r0
            goto L_0x003c
        L_0x003b:
            r0 = move-exception
        L_0x003c:
            r0 = 1
            goto L_0x003f
        L_0x003e:
            r0 = 0
        L_0x003f:
            int r6 = r2.size()
            if (r6 > r0) goto L_0x004a
            java.util.Set r5 = java.util.Collections.emptySet()
            return r5
        L_0x004a:
            java.util.TreeSet r6 = new java.util.TreeSet
            r6.<init>()
        L_0x004f:
            int r7 = r2.size()
            if (r0 >= r7) goto L_0x01f6
            java.lang.Object r7 = r2.get(r0)
            java.lang.String r7 = (java.lang.String) r7
            java.lang.Object r8 = r1.get(r0)
            java.lang.String r8 = (java.lang.String) r8
            if (r8 != 0) goto L_0x0065
            java.lang.String r8 = ""
        L_0x0065:
            org.jboss.netty.handler.codec.http.DefaultCookie r9 = new org.jboss.netty.handler.codec.http.DefaultCookie
            r9.<init>(r7, r8)
            r10 = 0
            r11 = 0
            r12 = 0
            r13 = 0
            r14 = 0
            r15 = 0
            r16 = 0
            r17 = -1
            java.util.ArrayList r5 = new java.util.ArrayList
            r3 = 2
            r5.<init>(r3)
            r3 = r5
            int r5 = r0 + 1
            r18 = r7
            r19 = r8
            r7 = r17
            r8 = r4
            r4 = r16
            r16 = r0
        L_0x0088:
            int r0 = r2.size()
            r20 = r6
            if (r5 >= r0) goto L_0x01ad
            java.lang.Object r0 = r2.get(r5)
            r6 = r0
            java.lang.String r6 = (java.lang.String) r6
            java.lang.Object r0 = r1.get(r5)
            r22 = r1
            r1 = r0
            java.lang.String r1 = (java.lang.String) r1
            java.lang.String r0 = "Discard"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00b0
            r0 = 1
            r10 = r0
        L_0x00aa:
            r29 = r1
            r31 = r2
            goto L_0x0196
        L_0x00b0:
            java.lang.String r0 = "Secure"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00bb
            r0 = 1
            r11 = r0
            goto L_0x00aa
        L_0x00bb:
            java.lang.String r0 = "HTTPOnly"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00c6
            r0 = 1
            r12 = r0
            goto L_0x00aa
        L_0x00c6:
            java.lang.String r0 = "Comment"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00d1
            r0 = r1
            r13 = r0
            goto L_0x00aa
        L_0x00d1:
            java.lang.String r0 = "CommentURL"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00dc
            r0 = r1
            r14 = r0
            goto L_0x00aa
        L_0x00dc:
            java.lang.String r0 = "Domain"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00e7
            r0 = r1
            r15 = r0
            goto L_0x00aa
        L_0x00e7:
            java.lang.String r0 = "Path"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x00f2
            r0 = r1
            r4 = r0
            goto L_0x00aa
        L_0x00f2:
            java.lang.String r0 = "Expires"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x013e
            org.jboss.netty.handler.codec.http.CookieDateFormat r0 = new org.jboss.netty.handler.codec.http.CookieDateFormat     // Catch:{ ParseException -> 0x0133 }
            r0.<init>()     // Catch:{ ParseException -> 0x0133 }
            java.util.Date r0 = r0.parse(r1)     // Catch:{ ParseException -> 0x0133 }
            long r18 = r0.getTime()     // Catch:{ ParseException -> 0x0133 }
            long r23 = java.lang.System.currentTimeMillis()     // Catch:{ ParseException -> 0x0133 }
            r0 = 0
            long r18 = r18 - r23
            r23 = 0
            int r0 = (r18 > r23 ? 1 : (r18 == r23 ? 0 : -1))
            if (r0 > 0) goto L_0x011a
            r0 = 0
            r27 = r13
            r28 = r14
            goto L_0x0130
        L_0x011a:
            r25 = 1000(0x3e8, double:4.94E-321)
            r27 = r13
            r28 = r14
            long r13 = r18 / r25
            int r0 = (int) r13     // Catch:{ ParseException -> 0x0131 }
            long r13 = r18 % r25
            int r17 = (r13 > r23 ? 1 : (r13 == r23 ? 0 : -1))
            if (r17 == 0) goto L_0x012c
            r21 = 1
            goto L_0x012e
        L_0x012c:
            r21 = 0
        L_0x012e:
            int r0 = r0 + r21
        L_0x0130:
            goto L_0x014e
        L_0x0131:
            r0 = move-exception
            goto L_0x0138
        L_0x0133:
            r0 = move-exception
            r27 = r13
            r28 = r14
        L_0x0138:
            r29 = r1
            r31 = r2
            goto L_0x0192
        L_0x013e:
            r27 = r13
            r28 = r14
            java.lang.String r0 = "Max-Age"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x0158
            int r0 = java.lang.Integer.parseInt(r1)
        L_0x014e:
            r7 = r0
        L_0x014f:
            r29 = r1
            r31 = r2
            r13 = r27
            r14 = r28
            goto L_0x0196
        L_0x0158:
            java.lang.String r0 = "Version"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x0165
            int r8 = java.lang.Integer.parseInt(r1)
            goto L_0x014f
        L_0x0165:
            java.lang.String r0 = "Port"
            boolean r0 = r0.equalsIgnoreCase(r6)
            if (r0 == 0) goto L_0x01a6
            java.lang.String r0 = ","
            java.lang.String[] r13 = r1.split(r0)
            r14 = r13
            r29 = r1
            int r1 = r14.length
            r0 = 0
        L_0x0178:
            r30 = r0
            r31 = r2
            r2 = r30
            if (r2 >= r1) goto L_0x0192
            r0 = r14[r2]
            r17 = r0
            java.lang.Integer r0 = java.lang.Integer.valueOf(r17)     // Catch:{ NumberFormatException -> 0x018c }
            r3.add(r0)     // Catch:{ NumberFormatException -> 0x018c }
            goto L_0x018d
        L_0x018c:
            r0 = move-exception
        L_0x018d:
            int r0 = r2 + 1
            r2 = r31
            goto L_0x0178
        L_0x0192:
            r13 = r27
            r14 = r28
        L_0x0196:
            int r5 = r5 + 1
            int r16 = r16 + 1
            r18 = r6
            r6 = r20
            r1 = r22
            r19 = r29
            r2 = r31
            goto L_0x0088
        L_0x01a6:
            r29 = r1
            r31 = r2
            r18 = r6
            goto L_0x01b7
        L_0x01ad:
            r22 = r1
            r31 = r2
            r27 = r13
            r28 = r14
            r29 = r19
        L_0x01b7:
            r9.setVersion(r8)
            r9.setMaxAge(r7)
            r9.setPath(r4)
            r9.setDomain(r15)
            r9.setSecure(r11)
            r9.setHttpOnly(r12)
            if (r8 <= 0) goto L_0x01d1
            r13 = r27
            r9.setComment(r13)
            goto L_0x01d3
        L_0x01d1:
            r13 = r27
        L_0x01d3:
            r0 = 1
            if (r8 <= r0) goto L_0x01e2
            r14 = r28
            r9.setCommentUrl(r14)
            r9.setPorts((java.lang.Iterable<java.lang.Integer>) r3)
            r9.setDiscard(r10)
            goto L_0x01e4
        L_0x01e2:
            r14 = r28
        L_0x01e4:
            r1 = r20
            r1.add(r9)
            int r0 = r16 + 1
            r6 = r1
            r4 = r8
            r1 = r22
            r2 = r31
            r3 = r33
            r5 = 0
            goto L_0x004f
        L_0x01f6:
            r22 = r1
            r31 = r2
            r1 = r6
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.http.CookieDecoder.decode(java.lang.String):java.util.Set");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:53:0x00d9, code lost:
        r2 = r6;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static void extractKeyValuePairs(java.lang.String r13, java.util.List<java.lang.String> r14, java.util.List<java.lang.String> r15) {
        /*
            int r0 = r13.length()
            r1 = 0
            r2 = 0
            r3 = r1
            r5 = r3
            r1 = 0
            r4 = 0
        L_0x000a:
            if (r2 != r0) goto L_0x000d
            goto L_0x0024
        L_0x000d:
            char r6 = r13.charAt(r2)
            r7 = 32
            if (r6 == r7) goto L_0x00e2
            r7 = 44
            if (r6 == r7) goto L_0x00e2
            r7 = 59
            if (r6 == r7) goto L_0x00e2
            switch(r6) {
                case 9: goto L_0x00e2;
                case 10: goto L_0x00e2;
                case 11: goto L_0x00e2;
                case 12: goto L_0x00e2;
                case 13: goto L_0x00e2;
                default: goto L_0x0020;
            }
        L_0x0020:
        L_0x0021:
            if (r2 != r0) goto L_0x0025
        L_0x0024:
            return
        L_0x0025:
            char r6 = r13.charAt(r2)
            r8 = 36
            if (r6 != r8) goto L_0x0030
            int r2 = r2 + 1
            goto L_0x0021
        L_0x0030:
            if (r2 != r0) goto L_0x0036
            r3 = 0
            r5 = 0
            goto L_0x00da
        L_0x0036:
            r6 = r2
        L_0x0037:
            char r8 = r13.charAt(r6)
            if (r8 == r7) goto L_0x00d3
            r9 = 61
            if (r8 == r9) goto L_0x004c
            int r6 = r6 + 1
            if (r6 != r0) goto L_0x0037
            java.lang.String r3 = r13.substring(r2)
            r5 = 0
            goto L_0x00d9
        L_0x004c:
            java.lang.String r3 = r13.substring(r2, r6)
            int r6 = r6 + 1
            if (r6 != r0) goto L_0x0058
            java.lang.String r5 = ""
            goto L_0x00d9
        L_0x0058:
            r5 = r6
            char r1 = r13.charAt(r6)
            r8 = 39
            r9 = 34
            if (r1 == r9) goto L_0x007e
            if (r1 != r8) goto L_0x0066
            goto L_0x007e
        L_0x0066:
            int r4 = r13.indexOf(r7, r6)
            if (r4 <= 0) goto L_0x0072
            java.lang.String r7 = r13.substring(r5, r4)
            r6 = r4
            goto L_0x0077
        L_0x0072:
            java.lang.String r7 = r13.substring(r5)
            r6 = r0
        L_0x0077:
            r4 = r1
            r1 = r5
            r2 = r6
            r5 = r7
            goto L_0x00da
        L_0x007e:
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            int r7 = r13.length()
            int r7 = r7 - r6
            r4.<init>(r7)
            r7 = r4
            r10 = r1
            r4 = 0
            int r6 = r6 + 1
        L_0x008d:
            if (r6 != r0) goto L_0x0099
            java.lang.String r8 = r7.toString()
            r4 = r1
            r1 = r5
            r2 = r6
            r5 = r8
            goto L_0x00da
        L_0x0099:
            r11 = 92
            if (r4 == 0) goto L_0x00ba
            r4 = 0
            int r12 = r6 + 1
            char r1 = r13.charAt(r6)
            if (r1 == r9) goto L_0x00ae
            if (r1 == r8) goto L_0x00ae
            if (r1 == r11) goto L_0x00ae
            r7.append(r1)
            goto L_0x00b8
        L_0x00ae:
            int r6 = r7.length()
            int r6 = r6 + -1
            r7.setCharAt(r6, r1)
        L_0x00b8:
            r6 = r12
            goto L_0x008d
        L_0x00ba:
            int r12 = r6 + 1
            char r1 = r13.charAt(r6)
            if (r1 != r10) goto L_0x00cc
            java.lang.String r6 = r7.toString()
            r4 = r1
            r1 = r5
            r5 = r6
            r2 = r12
            goto L_0x00da
        L_0x00cc:
            r7.append(r1)
            if (r1 != r11) goto L_0x00b8
            r4 = 1
            goto L_0x00b8
        L_0x00d3:
            java.lang.String r3 = r13.substring(r2, r6)
            r5 = 0
        L_0x00d9:
            r2 = r6
        L_0x00da:
            r14.add(r3)
            r15.add(r5)
            goto L_0x000a
        L_0x00e2:
            int r2 = r2 + 1
            goto L_0x000a
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.http.CookieDecoder.extractKeyValuePairs(java.lang.String, java.util.List, java.util.List):void");
    }
}
