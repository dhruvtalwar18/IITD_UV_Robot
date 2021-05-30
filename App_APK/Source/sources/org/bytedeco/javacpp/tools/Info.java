package org.bytedeco.javacpp.tools;

public class Info {
    String[] annotations = null;
    String base = null;
    boolean cast = false;
    String[] cppNames = null;
    String cppText = null;
    String[] cppTypes = null;
    boolean define = false;
    boolean enumerate = false;
    boolean flatten = false;
    String[] javaNames = null;
    String javaText = null;
    String[] linePatterns = null;
    String[] pointerTypes = null;
    boolean purify = false;
    boolean skip = false;
    boolean skipDefaults = false;
    boolean translate = false;
    String[] valueTypes = null;
    boolean virtualize = false;

    public Info() {
    }

    public Info(String... cppNames2) {
        this.cppNames = cppNames2;
    }

    public Info(Info i) {
        String[] strArr = null;
        this.cppNames = i.cppNames != null ? (String[]) i.cppNames.clone() : null;
        this.javaNames = i.javaNames != null ? (String[]) i.javaNames.clone() : null;
        this.annotations = i.annotations != null ? (String[]) i.annotations.clone() : null;
        this.cppTypes = i.cppTypes != null ? (String[]) i.cppTypes.clone() : null;
        this.valueTypes = i.valueTypes != null ? (String[]) i.valueTypes.clone() : null;
        this.pointerTypes = i.pointerTypes != null ? (String[]) i.pointerTypes.clone() : strArr;
        this.cast = i.cast;
        this.define = i.define;
        this.flatten = i.flatten;
        this.translate = i.translate;
        this.skip = i.skip;
        this.purify = i.purify;
        this.virtualize = i.virtualize;
        this.base = i.base;
        this.cppText = i.cppText;
        this.javaText = i.javaText;
    }

    public Info cppNames(String... cppNames2) {
        this.cppNames = cppNames2;
        return this;
    }

    public Info javaNames(String... javaNames2) {
        this.javaNames = javaNames2;
        return this;
    }

    public Info annotations(String... annotations2) {
        this.annotations = annotations2;
        return this;
    }

    public Info cppTypes(String... cppTypes2) {
        this.cppTypes = cppTypes2;
        return this;
    }

    public Info valueTypes(String... valueTypes2) {
        this.valueTypes = valueTypes2;
        return this;
    }

    public Info pointerTypes(String... pointerTypes2) {
        this.pointerTypes = pointerTypes2;
        return this;
    }

    public Info linePatterns(String... linePatterns2) {
        this.linePatterns = linePatterns2;
        return this;
    }

    public Info cast() {
        this.cast = true;
        return this;
    }

    public Info cast(boolean cast2) {
        this.cast = cast2;
        return this;
    }

    public Info define() {
        this.define = true;
        return this;
    }

    public Info define(boolean define2) {
        this.define = define2;
        return this;
    }

    public Info enumerate() {
        this.enumerate = true;
        return this;
    }

    public Info enumerate(boolean enumerate2) {
        this.enumerate = enumerate2;
        return this;
    }

    public Info flatten() {
        this.flatten = true;
        return this;
    }

    public Info flatten(boolean flatten2) {
        this.flatten = flatten2;
        return this;
    }

    public Info translate() {
        this.translate = true;
        return this;
    }

    public Info translate(boolean translate2) {
        this.translate = translate2;
        return this;
    }

    public Info skip() {
        this.skip = true;
        return this;
    }

    public Info skip(boolean skip2) {
        this.skip = skip2;
        return this;
    }

    public Info skipDefaults() {
        this.skipDefaults = true;
        return this;
    }

    public Info skipDefaults(boolean skipDefaults2) {
        this.skipDefaults = this.skip;
        return this;
    }

    public Info purify() {
        this.purify = true;
        return this;
    }

    public Info purify(boolean purify2) {
        this.purify = purify2;
        return this;
    }

    public Info virtualize() {
        this.virtualize = true;
        return this;
    }

    public Info virtualize(boolean virtualize2) {
        this.virtualize = virtualize2;
        return this;
    }

    public Info base(String base2) {
        this.base = base2;
        return this;
    }

    public Info cppText(String cppText2) {
        this.cppText = cppText2;
        return this;
    }

    public Info javaText(String javaText2) {
        this.javaText = javaText2;
        return this;
    }
}
