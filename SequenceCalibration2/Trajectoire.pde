float[] tracerTrajectoire() {

  stroke(5);

  float a = selected[1][0] - selected[2][0];
  float b = selected[1][1] - selected[2][1];

  //Calcul du point de choque entre trou et bille colorée
  float xArriveBilleJaune = selected[1][0] + ((a)/ sqrt((a*a)+(b*b)))*25;
  float yArriveBilleJaune = selected[1][1] + ((b)/ sqrt((a*a)+(b*b)))*25;


  a = selected[1][0] - selected[0][0];
  b= selected[1][1] - selected[0][1];

  //calcul colision entre point de choque et bille blanche
  float xArriveBilleNoir = xArriveBilleJaune + ((a)/ sqrt((a*a)+(b*b)))*25;
  float yArriveBilleNoir = yArriveBilleJaune + ((b)/ sqrt((a*a)+(b*b)))*25;

  float[] tab=new float[8];
  tab[0]=selected[2][0];
  tab[1]=selected[2][1];
  tab[2]=xArriveBilleJaune;
  tab[3]=yArriveBilleJaune;
  tab[4]=selected[0][0];
  tab[5]=selected[0][1];
  tab[6]=xArriveBilleNoir;
  tab[7]=yArriveBilleNoir;
  return tab;
}
