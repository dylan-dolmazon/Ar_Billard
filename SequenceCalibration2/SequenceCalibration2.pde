import netP5.*;
import oscP5.*;
import controlP5.*;
import processing.video.*;
import boofcv.processing.*;
import java.util.*;
import georegression.struct.shapes.Polygon2D_F64;
import georegression.struct.point.Point2D_F64;
import boofcv.alg.fiducial.qrcode.QrCode;
import boofcv.struct.image.*;

// 0 = A , 1 = B, 2 = C, 3 = D avec A,B,C,D valeur des fiducials markers
ArrayList<Point2D_F32> qrCodes = new ArrayList();
FiducialFound[] theorique = new FiducialFound[4];

Capture cam;
SimpleFiducial detector;

//for app
int STATE=0;
// width and height display
int constWidth = 1920;int constHeight = 1080;

// for calcul homographie
PImage fiducial1,fiducial2,fiducial3,fiducial4, imgrecord;

//size of théorique fudicial
float imgWidth=500; float imgHeight=500;
//position in sketch
float wFH =constWidth/2;  float hFH = constHeight/2;
PointTransformHomography_F32 homography_modele2image2,homography_projecteur;
List<FiducialFound> found;

//for draw bille and trajectory
boolean appIsReady = false;
boolean billeReceived=false;
boolean afficheTraj = false;
boolean configReady = false;
boolean resetTabBille = false;
boolean traj = false;
ArrayList<Bille> billeListReceived= new ArrayList<Bille>();
ArrayList<Integer> posTrou = new ArrayList<Integer>();
ArrayList<Integer> posMapper = new ArrayList<Integer>();
float selected[][] = new float[3][3];
int trouSelected;

// for mobile App
int windowWidth, windowHeight;
float coordXReceived, coordYReceived;
OscP5 oscP5;
NetAddress myRemoteLocation;

void setup(){
  windowWidth=displayWidth;
  windowHeight=displayHeight;
  size(displayWidth,displayHeight);
  initializeCamera(displayWidth, displayHeight);
  background(255);
  
  oscP5 = new OscP5(this, 12000);
  //use mobile IPV4
  myRemoteLocation = new NetAddress("172.31.246.198", 12000);
  
  //fiducial detector from boofcv start
  detector = Boof.fiducialSquareBinaryRobust(0.1);
  detector.guessCrappyIntrinsic(cam.width,cam.height);
}

void draw(){
  switch(STATE) {
  case 0: 
    if (cam.available() == true) {
          cam.read(); 
          fiducial1 = loadImage("fiducial.PNG");
          image(fiducial1,wFH-600,hFH - 400);
          fiducial2 = loadImage("fiducial2.PNG");
          image(fiducial2,wFH,hFH-400);
          fiducial3 = loadImage("fiducial1.PNG");
          image(fiducial3,wFH,hFH);
          fiducial4 = loadImage("fiducial3.PNG");
          image(fiducial4,wFH-600,hFH);
          found = detector.detect(cam);
          
          //détection of four fiducial projected and 4 papers fiducial
          if (found.size() > 7){
            // Configure the line's appearance
            background(255);
            strokeWeight(1);
            stroke(255, 0, 0);
            fill(0);
            textSize(72);
            text("Vous pouvez commencer à jouer",wFH-500 ,hFH);
            FiducialFound[] tmp = new FiducialFound[4];
            
            //order fiducial with contain
            for( FiducialFound qr : found ) {
              if(qr.getId() == 1){
                tmp[0] = qr;
              }
              else if(qr.getId() == 2){
                tmp[1] = qr;
              }else if(qr.getId() == 3){
                tmp[2] = qr;
              }else if(qr.getId() == 4){
                tmp[3] = qr;
              }else if(qr.getId() == 5){
                theorique[0] = qr;
              }else if(qr.getId() == 6){
                theorique[1] = qr;
             }else if(qr.getId() == 7){
                theorique[2] = qr;
              }else if(qr.getId() == 8){
                theorique[3] = qr;
              }
              
            }
            for(FiducialFound detectedQR : tmp){
              Point2D_F32 ptn = new Point2D_F32();
              
              ptn.x =(float) (detectedQR.getImageLocation().x);
              ptn.y =(float) (detectedQR.getImageLocation().y);
              qrCodes.add(ptn);
            }
            STATE = 1;
          }
      }
    break;
    
  case 1:

    //homographie entre sketch et billard
    homography_modele2image2 = searchHomography((double)qrCodes.get(0).x, (double)qrCodes.get(0).y,(double)qrCodes.get(1).x, (double)qrCodes.get(1).y, (double)qrCodes.get(2).x, (double)qrCodes.get(2).y, (double)qrCodes.get(3).x, (double)qrCodes.get(3).y,0.0, 0.0,windowWidth, 0.0, windowWidth, windowHeight,0.0, windowHeight);   
     
    //homographie entre caméra et projecteur
    homography_projecteur = searchHomography((double)wFH-475, (double)hFH-275,(double)wFH+125, (double)hFH-275, (double)wFH+125, (double)hFH+125, (double)wFH-475, (double)hFH+125,(double)theorique[0].getImageLocation().x, (double)theorique[0].getImageLocation().y,(double)theorique[1].getImageLocation().x, (double)theorique[1].getImageLocation().y, (double)theorique[2].getImageLocation().x, (double)theorique[2].getImageLocation().y, (double)theorique[3].getImageLocation().x, (double)theorique[3].getImageLocation().y);
    
    STATE =4; //attente
    break;
      
   case 2: 
     
    if(configReady){
          background(255);

          //draw balls
          for(Bille bille : billeListReceived){
                       
            Point2D_F32 resultTransformation1=new Point2D_F32();
            Point2D_F32 resultTransformation2=new Point2D_F32();
            homography_modele2image2.compute(bille.getX(), bille.getY(), resultTransformation1); //first homographie
            homography_projecteur.compute(resultTransformation1.x, resultTransformation1.y, resultTransformation2); //second one
            fill(bille.GetCouleur());
            ellipse(resultTransformation2.x,resultTransformation2.y,bille.getWidth(),bille.getWidth());
          }
        }
        if(traj){
        
          float[] tab = tracerTrajectoire();
          Point2D_F32 resultTransformation1=new Point2D_F32();
          Point2D_F32 resultTransformation2=new Point2D_F32();
          Point2D_F32 resultTransformation3=new Point2D_F32();
          Point2D_F32 resultTransformation4=new Point2D_F32();
          homography_modele2image2.compute(tab[0], tab[1], resultTransformation1); //first homographie
          homography_projecteur.compute(resultTransformation1.x, resultTransformation1.y, resultTransformation2);//second one
          homography_modele2image2.compute(tab[2], tab[3], resultTransformation3);//first homographie
          homography_projecteur.compute(resultTransformation3.x, resultTransformation3.y, resultTransformation4);// second one
          line(resultTransformation2.x,resultTransformation2.y,resultTransformation4.x,resultTransformation4.y);
          
          resultTransformation1=new Point2D_F32();
          resultTransformation2=new Point2D_F32();
          resultTransformation3=new Point2D_F32();
          resultTransformation4=new Point2D_F32();
          homography_modele2image2.compute(tab[4], tab[5], resultTransformation1);//first homographie
          homography_projecteur.compute(resultTransformation1.x, resultTransformation1.y, resultTransformation2);// second one
          homography_modele2image2.compute(tab[6], tab[7], resultTransformation3);//first homographie
          homography_projecteur.compute(resultTransformation3.x, resultTransformation3.y, resultTransformation4);// second one
          line(resultTransformation2.x,resultTransformation2.y,resultTransformation4.x,resultTransformation4.y);
          
     }
    STATE = 4; // attente
      break;
  }
}
/*
void findBalls(PImage input){
  SimpleGray gray = Boof.gray(input,ImageDataType.F32);

  // Threshold the image using its mean value
  double threshold = gray.mean();

  PImage psp;
  // find blobs and contour of the particles
  SimpleBinary imageThresholded = gray.threshold(128,true).erode8(1);
  GrayU8 ps = imageThresholded.getImage();
  /*SimpleGray sgi = new SimpleGray(ps);
  fffpsp = sgi.convert();
  psp.save("C:\\temp\\grayT.png");*/
  ResultsBlob results = imageThresholded.contour();

  // Visualize the results
  imgContour = results.getContours().visualize();
  imgBlobs = results.getLabeledImage().visualize();
  imgContourB = true;
  //surface.setSize(input.width, input.height);
  
  
}
*/

void initializeCamera( int desiredWidth, int desiredHeight ) {
  String[] cameras = Capture.list();

  if (cameras.length == 0) {
    println("There are no cameras available for capture.");
    exit();
  } else {
    cam = new Capture(this, desiredWidth, desiredHeight,cameras[0]);
    cam.start();
  }
}

//for communication betwen app and mobile app
void oscEvent(OscMessage theOscMessage) {
  
  
  if (theOscMessage.checkAddrPattern("/coordPoche")) {
    if (theOscMessage.checkTypetag("ffi")) {
      println("recu coord poche");
      posTrou.add(theOscMessage.get(0).intValue());
      posTrou.add(theOscMessage.get(1).intValue());
    }
  }
  if (theOscMessage.checkAddrPattern("/posBille")) {
    if (resetTabBille) {
      println("reset tab bille");
      billeListReceived.clear();
      resetTabBille = false;
      configReady = false;
      traj = false;
    }
    if (theOscMessage.checkTypetag("fffff")) {
      println("recu d'une bille");
      println(theOscMessage.get(0).floatValue()+ " " + theOscMessage.get(1).floatValue());
      int r = int(theOscMessage.get(2).floatValue());
      int g = int(theOscMessage.get(3).floatValue());
      int b = int(theOscMessage.get(4).floatValue());
      color col = color(r, g, b);
      billeListReceived.add(new Bille(theOscMessage.get(0).floatValue(), theOscMessage.get(1).floatValue()*windowHeight, 25, 25, col));
      STATE = 3;  
  }
    if (theOscMessage.checkTypetag("s")) {
      println("fin recu de billes");
      configReady = true;
      resetTabBille = true;//Remplissage fichier pour sketch mapping
      STATE = 2;
    }
  }
  if (theOscMessage.checkAddrPattern("/billeSelect")) {
    if (theOscMessage.checkTypetag("iffs")) {
      if (theOscMessage.get(3).toString().equals("trou")) {
        println("recu select trou");
        trouSelected = theOscMessage.get(0).intValue();
        selected[2][0] = theOscMessage.get(1).floatValue();
        selected[2][1] = theOscMessage.get(2).floatValue()*windowHeight;
        traj = true;
        STATE = 2;
      }
    }
  }
  if (theOscMessage.checkTypetag("ffs")) {
    if (theOscMessage.get(2).toString().equals("blanc")) {
      println("recu select blanche");
      selected[0][0] = theOscMessage.get(0).floatValue();
      selected[0][1] = theOscMessage.get(1).floatValue()*windowHeight;
    } else if (theOscMessage.get(2).toString().equals("select")) {
      println("recu select noire");
      selected[1][0] = theOscMessage.get(0).floatValue();
      selected[1][1] = theOscMessage.get(1).floatValue()*windowHeight;
      STATE = 2;
    }
  }
  if (theOscMessage.checkAddrPattern("/waitSize")) {
    if (theOscMessage.checkTypetag("s")) {
      SendSizeOfWindow();
    }
  }
  if (theOscMessage.checkAddrPattern("/coupDirect")) {
    println("training");
    if(resetTabBille){
      println("bjkvdf");
     billeListReceived.clear();
     resetTabBille = false;
     configReady = false;
     traj=false;
    }
    //level1 1bille rouge 1 blanche
    if (theOscMessage.checkTypetag("ffffffff")) {
      println("reçu lvl 1");
      //bille blanche
      selected[0][0] = theOscMessage.get(3).floatValue();
      selected[0][1] = theOscMessage.get(4).floatValue()*windowHeight;
      billeListReceived.add(new Bille(theOscMessage.get(3).floatValue(), theOscMessage.get(4).floatValue()*windowHeight, 25, 25, color(255,255,255)));
      //bille select
      selected[1][0] = theOscMessage.get(0).floatValue();
      selected[1][1] = theOscMessage.get(1).floatValue()*windowHeight;
      billeListReceived.add(new Bille(theOscMessage.get(0).floatValue(), theOscMessage.get(1).floatValue()*windowHeight, 25, 25, color(255,0,0)));
      
      //poche
      selected[2][0] = theOscMessage.get(6).floatValue();
      selected[2][1] = theOscMessage.get(7).floatValue()*windowHeight;
      
       resetTabBille = true;
       configReady = true;
       traj=true;
      println("training");
      STATE = 2;
    }
    //level1 2 bille rouge 1 blanche
    if (theOscMessage.checkTypetag("fffffffff")) {
      println("reçu lvl 2");
    }
  }
}

//Permet le calcul du vrai positionnement des billes
void SendSizeOfWindow() {
  OscMessage mySize = new OscMessage("/sendSize");
  mySize.add(windowWidth);
  mySize.add(windowHeight);
  oscP5.send(mySize, myRemoteLocation); 
  println("Size, sending to "+myRemoteLocation+ " " + "envoie");
}
