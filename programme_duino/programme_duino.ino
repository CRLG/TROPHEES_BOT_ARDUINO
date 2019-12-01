#include <Wire.h>
//#include <Pixy2.h>
#include <Adafruit_MotorShield.h> // inclusion de la librairie pour commander un motor shield
#include <MsTimer2.h> // inclusion de la librairie pour le timer de la duree du match

#include <Servo.h> // inclusion de la librairie pour commander ses servomoteurs
//#include <Adafruit_NeoPixel.h> //librairie pour les led
#ifdef __AVR__
  #include <avr/power.h>
#endif



#define MANUEL true //true (vrai) ou false (faux)

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


//----------------------------------------------------------------------------------------------
// CREATION VARIABLE, MOTEUR, CAMERA, LEDS
//----------------------------------------------------------------------------------------------

// Creation caméra 
//Pixy2 pixy;

//Création variables globales
bool bProgrammeDemarre; // variable qui indique si le programme est demarre (demarre==true)
int couleur_equipe;
Servo attrape_atome;
Servo libere_atome;
Servo pousse_atome;
Servo allumeur;

//Création des 2 moteurs
// Création d'une carte moteur avec l'adresse I2C par défaut
Adafruit_MotorShield Carte_Moteur = Adafruit_MotorShield();
// Connection d'un moteur de 200 par tour (1.8 degrés)
// moteur_G #1 (M1 and M2)
// moteur_D #2 (M3 and M4)
Adafruit_StepperMotor *Moteur_G = Carte_Moteur.getStepper(200, 1);
Adafruit_StepperMotor *Moteur_D = Carte_Moteur.getStepper(200, 2);

//création des leds
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 500; // delais pour les leds

/*
 * FONCTION AVANCER
 * Fait avancer le robot d'une certaine distance.
 * Demande une valeur qui est la distance à faire en cm
 */
void AVANCER(float x){ //en cm
  int pas = x/K;
  for(int i=0;i<pas;i++)
  {
    Moteur_G->step(1,BACKWARD,SINGLE); //en avant
    Moteur_D->step(1,FORWARD,SINGLE); //en avant
  }
}

/*
 * FONCTION AVANCER_UN_PAS
 * Fait touner le moteur droit et gauche d'un pas 
 */
void AVANCER_UN_PAS(){ //en cm
    Moteur_G->step(1,BACKWARD,SINGLE); //en avant
    Moteur_D->step(1,FORWARD,SINGLE); //en avant
}

/*
 * FONCTION RECULER
 * Fait reculer le robot d'une certaine distance.
 * Demande une valeur qui est la distance à faire en cm
 */
void RECULER(float x){ //en cm
  int pas = x/K;
  for(int i=0;i<pas;i++)
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
  }
}



/*
 * FONCTION RECALAGE
 * Fait un recalage contre la bordure
 */
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


/*
 * FONCTION TOURNER_DROITE
 * Fait tourner le robot vers la droite
 */
void TOURNER_DROITE(void){
  for(int i=0;i<100;i++)
  {
    Moteur_G->step(1,BACKWARD,SINGLE); //en avant
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
  }
}

/*
 * FONCTION TOURNER_GAUCHE
 * Fait tourner le robot vers la gauche
 */
void TOURNER_GAUCHE(void){
  for(int i=0;i<100;i++)
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,FORWARD,SINGLE); //en avant
  }
}

/*
 * FONCTION TOURNER_DE
 * Fait tourner le robot de plusieurs pas moteur vers la gauche ou la droite
 * La fonction appelle une variable qui est le nombre de pas pour tourner
 * si le nombre est négatif c'est vers la gauche sinon vers la droite
 */
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

/*
 * FONCTION InterruptTimer2
 * fonction appelée au bout de 100 secondes, fin du
 * match en mode autonome
 */
void InterruptTimer2() 
{ // debut de la fonction d'interruption Timer2

  digitalWrite(LED_BUILTIN, LOW);    // led eteinte

  //comme un chenillard à la fin du programme, il ne fait rien d'autre
  while(1)
  {
    /*for(int i=0;i<NUMPIXELS;i++)
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
    delay(delayval); // Delay for a period of time (in milliseconds).*/
  }
}







//----------------------------------------------------------------------------------------------
// SETUP
//----------------------------------------------------------------------------------------------
void setup()
{
  //initialisation de la caméra
  //pixy.init();

  //initialisation du bandeau de leds
  //pixels.begin(); //Pour utiliser les leds

  pinMode(A4,INPUT); // joystick1
  pinMode(A5,INPUT); // joystick2
  
  //initialisation de la carte moteur
  Carte_Moteur.begin();  //initialise le moteur avec une frequence par défaut 1.6KHz

  //initialisation des vitesses des moteurs selon le mode (MANUEL ou AUTONOME)
  if (MANUEL)
  {
    Moteur_G->setSpeed(2000);    
    Moteur_D->setSpeed(2000);      
  //vitesse obligatoire pour eviter blocage pc
  }
  else
  {
    Moteur_G->setSpeed(4000);   
    Moteur_D->setSpeed(4000);   
  }

  // initialisation du chronométrage du match
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
/*
attrape_atome.write(100); // angle d'initialisation d'attrape_atome 
delay(1500);
attrape_atome.write(ATTRAPE_ATOME_VERTICAL); // angle d'initialisation d'attrape_atome 
*/

libere_atome.attach(8); // pinoche 8 pour le servo libere_atome
/*
libere_atome.write(LIBERE_ATOME_SORTI);
delay(1500);
libere_atome.write(LIBERE_ATOME_RANGE);
*/

pousse_atome.attach(10); // pinoche 10 pour le servo pousse_atome
/*
pousse_atome.write(POUSSE_ATOME_RANGE);
delay(1500);
pousse_atome.write(POUSSE_ATOME_DEPLIE);
*/

// pinoche 6 pour le servo allumeur
allumeur.attach(6);
/*
allumeur.write(ALLUMEUR_TEST);
delay(1500);
allumeur.write(ALLUMEUR_RANGE);
*/


// initialise la communication série à une vitesse de 9600 bits par seconde:
if(MANUEL)
{
Serial.begin(9600);
Serial.setTimeout(1000);
}
}
















//----------------------------------------------------------------------------------------------
// LOOP
//----------------------------------------------------------------------------------------------
void loop() {
/*
       //---------------------------------------------
       //CHOIX COULEUR
       //---------------------------------------------
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
 */ 

 
  if (MANUEL)
//----------------------------------------------------------------------------------------------
// MODE MANUEL
//----------------------------------------------------------------------------------------------
  {

      int joystick1=analogRead(A4);
      int joystick2=analogRead(A5);
      
      if(joystick2<500)
      {
        AVANCER(10);
      }
    
  }
  else
//----------------------------------------------------------------------------------------------
// MODE AUTONOME
//----------------------------------------------------------------------------------------------  
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
      

      } 
    }
  }
}
