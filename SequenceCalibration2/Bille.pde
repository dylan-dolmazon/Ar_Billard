class Bille {

  private float coordX;
  private float coordY;
  private float wi;
  private float he;
  private float vx = 0;
  private float vy = 0;
  private boolean selected=false;
  private boolean whiteBille=false;
  private color couleur=color(255, 0, 0);

  public Bille(float x, float y, float w, float h) {
    coordX=x;
    coordY=y;
    wi=w;
    he=h;
  }
  public Bille(float x, float y, float w, float h,color c) {
    coordX=x;
    coordY=y;
    wi=w;
    he=h;
    couleur=c;
  }
  public void setWhiteBille(boolean b) {
    whiteBille=true;
  }
  public boolean getWhiteBille() {
    return whiteBille;
  }
  public float getX() {
    return coordX;
  }
  public float getY() {
    return coordY;
  }
  public float getWidth() {
    return wi;
  }
  public float getHeight() {
    return he;
  }
  public float getVX() {
    return vx;
  }
  public float getVY() {
    return vy;
  }
  public boolean getSelected() {
    return selected;
  }

  public void setselected( boolean b) {

    selected=b;
  }
 
  public boolean getselected() {

    return selected;
  }
  public color GetCouleur(){return couleur;}

}
