#include <Wire.h>
#include <Pixy2.h>
#include <Adafruit_MotorShield.h> // inclusion de la librairie pour commander un motor shield
#include <MsTimer2.h> // inclusion de la librairie pour le timer de la duree du match

#include <Servo.h> // inclusion de la librairie pour commander ses servomoteurs
#include <Adafruit_NeoPixel.h> //librairie pour les led
#ifdef __AVR__
  #include <avr/power.h>
#endif



#define REGLAGE false //true (vrai) ou false (faux)

//parametre des led
#define PIN            7 //port pour piloter
#define NUMPIXELS      3 //nombre de led

//reglages pour tete chercheuse
#define PETITE_VITESSE 3000
#define GRANDE_VITESSE 4500
#define X_MILIEU 157
#define X_ECART 15
#define ANGLE_MAX 35 //angle maximum avant de considérer que la tete chercheuse a merdé

#define K 0.1099 //1 pas = K cm
#define DUREE 100000
#define TIRETTE 5
#define ORANGE 0
#define VIOLET 1
#define ATTRAPE_ATOME_VERTICAL 150
#define ATTRAPE_ATOME_HORIZONTAL 77
#define LIBERE_ATOME_RANGE 1
#define LIBERE_ATOME_SORTI 95
#define POUSSE_ATOME_RANGE 150
#define POUSSE_ATOME_DEPLIE 5
#define ALLUMEUR_SORTI 165
#define ALLUMEUR_RANGE 90
#define ALLUMEUR_TEST 45

#define ROUGE 1
#define BLEU 2
#define VERT 3



// This is the main Pixy object 
Pixy2 pixy;

bool bProgrammeDemarre; // variable qui indique si le programme est demarre (demarre==true)
int couleur_equipe;
Servo attrape_atome;
Servo libere_atome;
Servo pousse_atome;
Servo allumeur;

// Création d'une carte moteur avec l'adresse I2C par défaut
Adafruit_MotorShield Carte_Moteur = Adafruit_MotorShield();

// Ou création avec une adresse I2C différente (si empilement)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connection d'un moteur de 200 par tour (1.8 degrés)
// moteur_G #1 (M1 and M2)
// moteur_D #2 (M3 and M4)
Adafruit_StepperMotor *Moteur_G = Carte_Moteur.getStepper(200, 1);
Adafruit_StepperMotor *Moteur_D = Carte_Moteur.getStepper(200, 2);

//création des leds
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 500; // delay for half a second

void AVANCER(float x){ //en cm
  int pas = x/K;
  for(int i=0;i<pas;i++)
  {
    Moteur_G->step(1,BACKWARD,SINGLE); //en avant
    Moteur_D->step(1,FORWARD,SINGLE); //en avant
  }
}

void RECULER(float x){ //en cm
  int pas = x/K;
  for(int i=0;i<pas;i++)
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
  }
}

void RECALAGE(int time_out){
  int contacteur_AD =analogRead(A1);
  int contacteur_AG=analogRead(A2);
  unsigned long temps_init=millis();
  //unsigned long temps_init2=millis();
  unsigned long tempo=0;
  //unsigned long tempo2=0;
  do
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
    tempo=millis()-temps_init; 

    contacteur_AD =analogRead(A1);
   contacteur_AG=analogRead(A2);
   /* 
       tempo2=millis()-temps_init2;  
    if(tempo2>1000){
   Serial.print(contacteur_AG);Serial.print("\t"); Serial.println(contacteur_AD);
   temps_init2=millis();
   }*/
   
  }
  while(((contacteur_AD>800)||(contacteur_AG>800))&&(tempo<(time_out*1000)));
  temps_init=millis();
  do
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
    tempo=millis()-temps_init; 
  }
  while(tempo<500);
} 



/**
 * Si on arrive à se recaler ==> on renvoie true
 * Si on n'arrive pas ==> on renvoie false
 */
bool TETE_CHERCHEUSE(int temporisation, int couleur)
{
  unsigned long temps_debut=millis();
  unsigned long temps_compteur=0;
  int compteur=201;
  int x_milieu=X_MILIEU;
  int x_capture=X_MILIEU;
  int compteur_erreur=0;
  int compteur_angle=0;

  
  while(temps_compteur<(temporisation*1000))
  {

    if(compteur>2)
    {
      // grab blocks!
      pixy.ccc.getBlocks();
      x_capture=0;
      
      if (pixy.ccc.numBlocks)
      {
        for(int i=0; i<pixy.ccc.numBlocks; i++)
        {
          if(pixy.ccc.blocks[i].m_signature==couleur)
          {
            int x_detecte=pixy.ccc.blocks[i].m_x;
            int y_detecte=pixy.ccc.blocks[i].m_y;
            if((couleur==BLEU)&&(y_detecte>140))
            {
              x_capture=x_detecte;
            }
            if((couleur==ROUGE)&&(y_detecte>140))
            {
              x_capture=x_detecte;
            }
            //x_capture=pixy.ccc.blocks[0].m_x;
          //pixy.ccc.blocks[i].print();
          }
        }
      }

      if(x_capture==0)
      {
        //compteur_erreur=compteur_erreur+1;
        x_capture=x_milieu;
      }

      compteur=0;
    }  
    
    //l'atome est à droite de l'image
    if(x_capture>(x_milieu+X_ECART))
    {
      //tourner à droite
      Moteur_G->step(1,BACKWARD,SINGLE); //en avant
      Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
      compteur_angle=compteur_angle+1;
    }
    //l'atome est à gauche de l'image
    if(x_capture<(x_milieu-X_ECART))
    {
      //tourner à gauche
      Moteur_G->step(1,FORWARD,SINGLE); //en arriere
      Moteur_D->step(1,FORWARD,SINGLE); //en avant
      compteur_angle=compteur_angle-1;
    }
    
    temps_compteur=millis()-temps_debut;
    compteur=compteur+1;

    if((compteur_erreur>=3)||(compteur_angle>ANGLE_MAX)||(compteur_angle<(-ANGLE_MAX)))
    {
      TOURNER_DE(compteur_angle); //on a trop tourné on tourne du même angle dans l'autre sens
      return false; //on sort de la tête chercheuse
    }
  }

  return true;
}


void TOURNER_DROITE(void){
  for(int i=0;i<100;i++)
  {
    Moteur_G->step(1,BACKWARD,SINGLE); //en avant
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
  }
}

void TOURNER_GAUCHE(void){
  for(int i=0;i<100;i++)
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,FORWARD,SINGLE); //en avant
  }
}

void TOURNER_DE(int pas){
  if(pas>0)
  {
     for(int i=0;i<pas;i++)
    {
      Moteur_G->step(1,FORWARD,SINGLE); //en arriere
      Moteur_D->step(1,FORWARD,SINGLE); //en avant
    }
  }
  if(pas<0)
  {
    for(int i=0;i<(pas*(-1));i++)
    {
      Moteur_G->step(1,BACKWARD,SINGLE); //en avant
      Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
    }
  }
}

void InterruptTimer2() 
{ // debut de la fonction d'interruption Timer2

  digitalWrite(LED_BUILTIN, LOW);    // led eteinte

  //comme un chenillard à la fin du programme, il ne fait rien d'autre
  while(1)
  {
    for(int i=0;i<NUMPIXELS;i++)
    {
    pixels.setPixelColor(i, pixels.Color(0,150,0)); // vert
    pixels.show(); // ça applique la couleur programmée
    delay(delayval); // Delay for a period of time (in milliseconds).
    }
    for(int i=0;i<NUMPIXELS;i++)
    {
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // led éteinte
    }
    pixels.show(); // ça applique la couleur programmée
    delay(delayval); // Delay for a period of time (in milliseconds).
  }

}

void setup()
{
  pixy.init();

  pixels.begin(); //Pour utiliser les leds
  
  
  Carte_Moteur.begin();  //initialise le moteur avec une frequence par défaut 1.6KHz
  //Carte_Moteur.begin(1000);  //ou avec une fréquence différente 1KHz

    
  if (REGLAGE)
  {
    Moteur_G->setSpeed(2000);  // 7560 rpm   
    
    Moteur_D->setSpeed(2000);  // 7560 rpm      
  }
  else
  {
    Moteur_G->setSpeed(4000);  // 7560 rpm   
    
    Moteur_D->setSpeed(4000);  // 7560 rpm   
  }

  // initialisation interruption Timer 2
MsTimer2::set(DUREE, InterruptTimer2); // période = duree du match, on activera avec la tirette

pinMode(LED_BUILTIN, OUTPUT); // initialisation de la led interne de l'arduino

digitalWrite(LED_BUILTIN, LOW);    // led eteinte

pinMode(TIRETTE, INPUT);  //en entrée

pinMode(A0,INPUT_PULLUP); // choix de la couleur

couleur_equipe=VIOLET; //equipe violette par defaut

bProgrammeDemarre=false; // le programme n'est pas demarre quand l'arduino s'allume

pinMode(A1,INPUT_PULLUP); // contacteur arrière droit
pinMode(A2,INPUT_PULLUP); // contacteur arrière gauche

attrape_atome.attach(9); // pinoche 9 pour le servo attrape_atome
attrape_atome.write(100); // angle d'initialisation d'attrape_atome 
delay(1500);
attrape_atome.write(ATTRAPE_ATOME_VERTICAL); // angle d'initialisation d'attrape_atome 

libere_atome.attach(8);
libere_atome.write(LIBERE_ATOME_SORTI);
delay(1500);
libere_atome.write(LIBERE_ATOME_RANGE);

pousse_atome.attach(10);
pousse_atome.write(POUSSE_ATOME_RANGE);
delay(1500);
pousse_atome.write(POUSSE_ATOME_DEPLIE);


allumeur.attach(6);
allumeur.write(ALLUMEUR_TEST);
delay(1500);
allumeur.write(ALLUMEUR_RANGE);


// initialize serial communication at 9600 bits per second:
if(REGLAGE)
{
Serial.begin(9600);
Serial.setTimeout(1000);
}
}

//int couleur_équipe=1;

void loop() {

       int valeur_bouton=analogRead(A0);
      if(valeur_bouton<800) //bouton vers le haut
      {
        couleur_equipe=VIOLET;
        for(int i=0;i<NUMPIXELS;i++)
        {
        pixels.setPixelColor(i, pixels.Color(75,0,75)); // réglage en bleu assez faible
        pixels.show(); // ça applique la couleur programmée
        delay(delayval); // Delay for a period of time (in milliseconds).
        }
        /*pixels.setPixelColor(0, pixels.Color(150,0,0)); // Moderately bright green color.
        pixels.setPixelColor(1, pixels.Color(0,150,0)); // Moderately bright green color.
        pixels.setPixelColor(2, pixels.Color(0,0,150)); // Moderately bright green color.
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(delayval); // Delay for a period of time (in milliseconds).*/
      }
      else
      {
        couleur_equipe=ORANGE;
        for(int i=0;i<NUMPIXELS;i++)
        {
        pixels.setPixelColor(i, pixels.Color(255,50,0)); // Moderately bright green color.
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(delayval); // Delay for a period of time (in milliseconds).
        }
      }
  
  if (REGLAGE)
  {
    Serial.println("\n\nChoississez une action:");
    Serial.println("\t* 1 : piloter le servo qui attrape les atomes.");
    Serial.println("\t* 2 : piloter le servo qui pousse les atomes.");
    Serial.println("\t* 3 : piloter le servo qui libere les atomes.");
    Serial.println("\t* 4 : piloter le servo qui allume l'expérience.");
    Serial.println("\t* 5 : lire la valeur bouton de couleur.");
    Serial.println("\t* 6 : une tete chercheuse sur le bleu avec 40 secondes de délai.");
    Serial.flush();
    while (!Serial.available());
    int choix = Serial.parseInt();

    if((choix >=1)&&(choix<=4))
    {
      Serial.print("\nDonnez la valeur de la position du servo n°");
      Serial.print(choix);
      Serial.println(" entre 0 et 180:");
      delay(2000);
      Serial.flush();
      while (!Serial.available());
      int pos=Serial.parseInt();
      if((pos>0) && (pos < 180))
      {
        Serial.println(pos);
        if(choix==1)
          attrape_atome.write(pos);
        if(choix==2)
          pousse_atome.write(pos);
        if(choix==3)
          libere_atome.write(pos);
        if(choix==4)
          allumeur.write(pos);
      }
    }
    if(choix==5)
    {
      int valeur=analogRead(A0);
      if(valeur<800) //bouton vers le haut
        Serial.println("Equipe VIOLET");
      else
        Serial.println("Equipe ORANGE");
    }
    if(choix==6)
      TETE_CHERCHEUSE(40,BLEU);

  }
  else
  {
  
    if ((digitalRead(TIRETTE)==LOW)&&(bProgrammeDemarre==false))
    {
      bProgrammeDemarre=true; // le programme est demarre

      int valeur=analogRead(A0);
      if(valeur<800) //bouton vers le haut
        couleur_equipe=VIOLET;
      else
        couleur_equipe=ORANGE;
      
      MsTimer2::start(); // active Timer2
      
      digitalWrite(LED_BUILTIN, HIGH); // led allumee

      if (couleur_equipe==VIOLET)
      {
      
      //Séquence de recalage avant le bouton
      RECALAGE(15);
      AVANCER(20);
      TOURNER_DROITE();
      RECALAGE(15);
      AVANCER(27);
      TOURNER_DE(120);
      RECALAGE(15);
      AVANCER(20);          
      /*
      * on recule vers la bordure
      */
      allumeur.write(ALLUMEUR_SORTI);
      RECULER(50);
      delay(1000);
      allumeur.write(ALLUMEUR_RANGE);
      delay(1000);
      RECALAGE(15);//RECULER(15);
 
      /*
      * aller jusqu'à la bordure en se recalant
      */

      //AVANCER(200);
      AVANCER(50);

      //RECALAGE N°1
      TOURNER_DE(-110);
      RECALAGE(15);//RECULER(50);
      AVANCER(10);
      TOURNER_DE(110);
      
      AVANCER(50);

      //RECALAGE N°2
      TOURNER_DE(-110);
      RECALAGE(15);//RECULER(50);
      AVANCER(10);
      TOURNER_DE(110);
      
      AVANCER(50);

      //RECALAGE N°3
      TOURNER_DE(-110);
      RECALAGE(15);//RECULER(50);
      AVANCER(10);
      TOURNER_DE(110);
      
      attrape_atome.write(ATTRAPE_ATOME_HORIZONTAL);
      pousse_atome.write(POUSSE_ATOME_RANGE);

      //on se recale sur l'atome avec la caméra
      bool estOK=true;
      for(int k=0;k<NUMPIXELS;k++)
        pixels.setPixelColor(k, pixels.Color(0,0,0)); //on eteint toutes les leds
      pixels.show(); // ça applique la couleur programmée
      
      for(int j=0;j<3;j++)
      {
        bool tentative=TETE_CHERCHEUSE(2,BLEU);
        delay(500);
        AVANCER(20);

        if(tentative==true)
          pixels.setPixelColor(j, pixels.Color(0,150,0)); //tete chercheuse a reussi: led verte
        else
          pixels.setPixelColor(j, pixels.Color(150,0,0)); //tete chercheuse a foiré: led rouge
        pixels.show(); // ça applique la couleur programmée  
        
        estOK= estOK && tentative;
      }

      if(estOK==false) //la tete charcheuse a foiré au moins une fois, on se recale
      {
        AVANCER(10);
        RECULER(10);
        TOURNER_DE(-110);
        RECALAGE(15);//RECULER(40);
        attrape_atome.write(ATTRAPE_ATOME_HORIZONTAL);
        //delay(1000);
        pousse_atome.write(POUSSE_ATOME_RANGE);
        AVANCER(18);
        //TOURNER_GAUCHE();
        TOURNER_DE(130);
        delay(600);
        AVANCER(30);
        delay(600);
      }

      //on se remet face aux atomes
      RECULER(2);
      delay(600);
      AVANCER(2);
      
      /*
      * attraper les atomes
      */
      delay(2000);
      pousse_atome.write(POUSSE_ATOME_DEPLIE);
      delay(2000);
      pousse_atome.write(POUSSE_ATOME_RANGE);
      delay(500);
      RECULER(5);

    
      /*
      * se tourner vers la pente et y aller
      */
      TOURNER_DE(-110);
      //TOURNER_GAUCHE();
      RECALAGE(15);//RECULER(35);
      attrape_atome.write(ATTRAPE_ATOME_VERTICAL);
      pousse_atome.write(POUSSE_ATOME_DEPLIE);
      AVANCER(140);
      
      /*
      * libérer les atomes et pousser les 4 atomes vers la balance
      */
      RECULER(15);
      libere_atome.write(LIBERE_ATOME_SORTI);
      delay(2000);
      AVANCER(30);

      /**
       * on refait la même chose pour être sur
       */
      RECULER(15);
      libere_atome.write(LIBERE_ATOME_SORTI);
      delay(2000);
      AVANCER(20);

      //on retourne en base de la pente
      RECALAGE(15);


      }
      else
      {

      RECALAGE(15);//RECULER(50);
      AVANCER(20);
      //TOURNER_GAUCHE();
      TOURNER_DE(110);
      RECALAGE(15);//RECULER(45);
      AVANCER(33);
      //TOURNER_DROITE();
      TOURNER_DE(-110);
      RECALAGE(15);//RECULER(50);
      AVANCER(20);
      
      /*
      * on recule vers la bordure
      */
      allumeur.write(ALLUMEUR_SORTI-5);
      RECULER(50);
      delay(1000);
      allumeur.write(ALLUMEUR_RANGE);
      delay(1000);
      RECALAGE(15);//RECULER(15);
 
      /*
      * aller jusqu'à la bordure
      */
      AVANCER(50);

      //RECALAGE N°1
      TOURNER_DE(110);
      RECALAGE(15);//RECULER(50);
      AVANCER(10);
      TOURNER_DE(-110);
      
      AVANCER(50);

      //RECALAGE N°2
      TOURNER_DE(110);
      RECALAGE(15);//RECULER(50);
      AVANCER(10);
      TOURNER_DE(-110);
      
      AVANCER(50);

      //RECALAGE N°3
      TOURNER_DE(110);
      RECALAGE(15);//RECULER(50);
      AVANCER(10);
      TOURNER_DE(-110);

      attrape_atome.write(ATTRAPE_ATOME_HORIZONTAL);
      pousse_atome.write(POUSSE_ATOME_RANGE);

      //on se recale sur l'atome avec la caméra
      bool estOK=true;
      for(int k=0;k<NUMPIXELS;k++)
        pixels.setPixelColor(k, pixels.Color(0,0,0)); //on eteint toutes les leds
      pixels.show(); // ça applique la couleur programmée
      
      for(int j=0;j<3;j++)
      {
        bool tentative=TETE_CHERCHEUSE(2,ROUGE);
        delay(500);
        AVANCER(20);

        if(tentative==true)
          pixels.setPixelColor(j, pixels.Color(0,150,0)); //tete chercheuse a reussi: led verte
        else
          pixels.setPixelColor(j, pixels.Color(150,0,0)); //tete chercheuse a foiré: led rouge
        pixels.show(); // ça applique la couleur programmée  
        
        estOK= estOK && tentative;
      }

      if(estOK==false) //la tete charcheuse a foiré au moins une fois, on se recale
      {
        RECULER(10);
        //TOURNER_GAUCHE();
        TOURNER_DE(110);
        RECALAGE(15);//RECULER(40);
        attrape_atome.write(ATTRAPE_ATOME_HORIZONTAL);
        //delay(1000);
        pousse_atome.write(POUSSE_ATOME_RANGE);
        AVANCER(18);
        //TOURNER_DROITE();
        TOURNER_DE(-130);
        delay(600);
        AVANCER(30);
        delay(600);
        RECULER(2);
        delay(600);
        AVANCER(2);
      }
      
      /*
      * attraper les atomes
      */
      attrape_atome.write(ATTRAPE_ATOME_HORIZONTAL);
      delay(2000);
      pousse_atome.write(POUSSE_ATOME_DEPLIE);
      delay(2000);
      pousse_atome.write(POUSSE_ATOME_RANGE);
      delay(500);
      RECULER(5);

    
      /*
      * se tourner vers la pente et y aller
      */
      TOURNER_DE(110);
      //TOURNER_DROITE();
      RECALAGE(15);//RECULER(35);
      attrape_atome.write(ATTRAPE_ATOME_VERTICAL);
      pousse_atome.write(POUSSE_ATOME_DEPLIE);
      AVANCER(140);
      
      /*
      * libérer les atomes et pousser les 4 atomes vers la balance
      */
      RECULER(15);
      libere_atome.write(LIBERE_ATOME_SORTI);
      delay(2000);
      AVANCER(30);

      /**
       * on refait la même chose pour être sur
       */
      RECULER(15);
      libere_atome.write(LIBERE_ATOME_SORTI);
      delay(2000);
      AVANCER(20);

      //on retourne en base de la pente
      RECALAGE(15);

      } 
    }
  }
}
