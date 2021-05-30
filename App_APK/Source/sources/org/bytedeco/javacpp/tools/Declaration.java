package org.bytedeco.javacpp.tools;

class Declaration {
    boolean abstractMember = false;
    boolean comment = false;
    boolean constMember = false;
    Declarator declarator = null;
    boolean function = false;
    boolean inaccessible = false;
    boolean incomplete = false;
    String signature = "";
    String text = "";
    Type type = null;
    boolean variable = false;

    Declaration() {
    }
}
