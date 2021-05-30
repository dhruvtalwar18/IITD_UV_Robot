package uk.co.blogspot.fractiousg.texample;

class TextureRegion {
    public float u1;
    public float u2;
    public float v1;
    public float v2;

    public TextureRegion(float texWidth, float texHeight, float x, float y, float width, float height) {
        this.u1 = x / texWidth;
        this.v1 = y / texHeight;
        this.u2 = this.u1 + (width / texWidth);
        this.v2 = this.v1 + (height / texHeight);
    }
}
