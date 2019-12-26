#include <SoftwareSerial.h>
#include "messagerieTB.h"
#include <Wire.h>
#include <Pixy2.h>
#include <Adafruit_MotorShield.h> // inclusion de la librairie pour commander un motor shield
#include <MsTimer2.h> // inclusion de la librairie pour le timer de la duree du match
#include <Servo.h> // inclusion de la librairie pour commander ses servomoteurs
#include <Adafruit_NeoPixel.h> //librairie pour les led
#ifdef __AVR__
  #include <avr/power.h>
#endif

//CONFIGURATION DU ROBOT
//pour mettre en mode autonome (cad sans télécommande) true (vrai) ou false (faux)
#define AUTONOME true
//pour debuguer le programme, mettre à true
#define DEBUG false
//à décommenter pour utiliser le bandeau de LEDS
//#define UTILISE_LEDS true
//à décommenter pour utiliser la camera
//#define UTILISE_CAMERA true


//ports utilisés
/**
 * POUR INFO en utilisant le motor shield v2:
 * la masse (GND) ainsi que le 5v (par defaut) ou le 3.3v sont nécessaire pour faire fonctionner le shield. (le 5v ou le 3v peuvent être choisis par un jumper sur la carte)
 * Le shield utilise le signal SDA et SCL pour le protocole i2c utilisé pour contrôler les moteurs. Sur l'arduino UNO ce sont les pinoches analogiques A4 et A5.
 * Sur l'arduino Mega ces pinoches sont plutôt les pinoches numériques 20 et 21.
 * Il ne faut donc pas utiliser ces pinoches sur ces arduinos avec ce shield pour quoique ce soit d'autre que des capteurs ou actionneurs i2c.
 *
 * Etant donné que le shield utilise l'i2c pour communiquer, vous pouvez connecter d'autres capteurs ou actionneurs i2c sur les signaux SDA et SCL tant qu'ils n'utilisent pas l'adresse 0x60
 * (l'adresse par défaut du shield) ou l'adresse 0x070 (l'adresse utilisable par le shield pour contrôler un groupe de shield)
 *
 * Si vous voulez utiliser les broches ddu shield déddiées aux servos, elles sont reliées aux pinoches 9 et 10.
 * Si vous n'utilisez pas ces broches, les pinoches 9 et 10 sont libres d'utilisation.
 * Vous pouvez utiliser toutes les autres pinoches non mentionnées ci-dessus.
 */
#define PIN_COULEUR_EQUIPE  A0 //mode autonome
#define PIN_CONTACTEUR_AR_D A1
#define PIN_CONTACTEUR_AR_G A2
//pas encore utilisé        A3
//motor shield SDA          A4
//motor shield SCL          A5
//pas encore utilisé        0
//pas encore utilisé        1
//pas encore utilisé        2
#define PIN_RX              3 //mode téléguidé
#define PIN_TX              4 //mode téléguidé
#define PIN_TIRETTE         5 //mode autonome
#define PIN_LEDS            6 //mode autonome
#define PIN_SERVO_01        7
#define PIN_SERVO_02        8
#define PIN_SERVO_03        9
#define PIN_SERVO_04        10
#define PIN_SERVO_05        11
#define PIN_SERVO_06        12
//pas encore utilisé        13

//parametrage du match
#define DUREE_MATCH               100000 //en millisecondes
#define DISTANCE_PAR_PAS_MOTEUR   0.1099 //1 pas = K cm
#define EQUIPE_JAUNE              0
#define EQUIPE_BLEUE              1

//parametrage optionnel
#define LENTE                     2000
#define RAPIDE                    4000
#define NBRE_DE_LEDS              3 //nombre de led

//valeurs des servo_moteurs
#define SERVO_01_INIT 50
#define SERVO_01_TEST 90
#define SERVO_02_INIT 50
#define SERVO_02_TEST 90
#define SERVO_03_INIT 50
#define SERVO_03_TEST 90
#define SERVO_04_INIT 50
#define SERVO_04_TEST 90
#define SERVO_05_INIT 50
#define SERVO_05_TEST 90
#define SERVO_06_INIT 50
#define SERVO_06_TEST 90

//----------------------------------------------------------------------------------------------
// CREATION VARIABLE, MOTEUR, CAMERA, LEDS
//----------------------------------------------------------------------------------------------
//on informe le microcontrôleur que l'on utilise ses broches RX et TX pour une connexion série
SoftwareSerial liaisonSerie(PIN_RX, PIN_TX);

//Création variables globales
bool bProgrammeDemarre; // variable qui indique si le programme est demarre (demarre==true)
int couleur_equipe;

//Création des servos
Servo servo_01;
Servo servo_02;
Servo servo_03;
Servo servo_04;
Servo servo_05;
Servo servo_06;

//Création des 2 moteurs
// Création d'une carte moteur avec l'adresse I2C par défaut
Adafruit_MotorShield Carte_Moteur = Adafruit_MotorShield();
// Connection d'un moteur de 200 pas par tour (1.8 degrés)
// moteur_G #1 (M1 and M2)
// moteur_D #2 (M3 and M4)
Adafruit_StepperMotor *Moteur_G = Carte_Moteur.getStepper(200, 1);
Adafruit_StepperMotor *Moteur_D = Carte_Moteur.getStepper(200, 2);

//création des leds
#ifdef UTILISE_LEDS
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NBRE_DE_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);
  int delayval = 500; // delais pour les leds

  //couleurs possibles de leds
  #define LEDS_BLEU 1
  #define LEDS_ORANGE

  /*
   * FONCTION METTRE_LEDS_A
   * mets toutes les leds à une certaine couleur
   */
  void METTRE_LEDS_A(int couleur)
  {
      for(int i=0;i<NUMPIXELS;i++)
      {
        switch(couleur)
        {
          case LEDS_BLEU : pixels.setPixelColor(i, pixels.Color(75,0,75)); break;
          case LEDS_ORANGE : pixels.setPixelColor(i, pixels.Color(255,50,0)); break;
          default:    pixels.setPixelColor(i, pixels.Color(75,0,75)); break; // réglage en bleu par defaut
        }
        pixels.show(); //ça applique la couleur programmée
        delay(delayval); //on attend "delayval" milliseconds
      }
  }
#endif

//création caméra
#ifdef UTILISE_CAMERA
  Pixy2 pixy;
#endif

/*
 * FONCTION VITESSE
 * positionne la vitesse des moteurs
 * Demande une valeur qui est la vitesse de rotation en tours par minute
 */
void VITESSE(int valeur_vitesse)
{
    Moteur_G->setSpeed(valeur_vitesse);   
    Moteur_D->setSpeed(valeur_vitesse); 
}

/*
 * FONCTION AVANCER
 * Fait avancer le robot d'une certaine distance.
 * Demande une valeur qui est la distance à faire en cm
 */
void AVANCER(float x)
{
  int pas = x/DISTANCE_PAR_PAS_MOTEUR;
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
void AVANCER_UN_PAS()
{
    Moteur_G->step(1,BACKWARD,SINGLE); //en avant
    Moteur_D->step(1,FORWARD,SINGLE); //en avant
}

/*
 * FONCTION RECULER
 * Fait reculer le robot d'une certaine distance.
 * Demande une valeur qui est la distance à faire en cm
 */
void RECULER(float x)
{ 
  int pas = x/DISTANCE_PAR_PAS_MOTEUR;
  for(int i=0;i<pas;i++)
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
  }
}

/*
 * FONCTION RECULER_UN_PAS
 * Fait touner le moteur droit et gauche d'un pas 
 */
void RECULER_UN_PAS(){ 
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
}

/*
 * FONCTION RECALAGE
 * Fait un recalage contre la bordure
 */
void RECALAGE(int time_out){
  int contacteur_AD =analogRead(PIN_CONTACTEUR_AR_D);
  int contacteur_AG=analogRead(PIN_CONTACTEUR_AR_G);
  unsigned long temps_init=millis();
  unsigned long tempo=0;
  do
  {
    Moteur_G->step(1,FORWARD,SINGLE); //en arriere
    Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
    tempo=millis()-temps_init; 
    
    contacteur_AD =analogRead(PIN_CONTACTEUR_AR_D);
    contacteur_AG=analogRead(PIN_CONTACTEUR_AR_G);
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
      Moteur_G->step(1,BACKWARD,SINGLE); //en avant
      Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
    }
  }
  if(pas<0)
  {
    for(int i=0;i<(pas*(-1));i++)
    {
      Moteur_G->step(1,FORWARD,SINGLE); //en arriere
      Moteur_D->step(1,FORWARD,SINGLE); //en avant
    }
  }
}

/*
 * FONCTION InterruptTimer2
 * fonction appelée au bout de 100 secondes, fin du
 * match en mode autonome
 */
void FinDeMatch() 
{
  digitalWrite(LED_BUILTIN, LOW);    // led eteinte

  //comme un chenillard à la fin du programme, il ne fait rien d'autre
  while(1)
  {
#ifdef UTILISE_LEDS
    for(int i=0;i<NUMPIXELS;i++)
    {
      pixels.setPixelColor(i, pixels.Color(0,150,0)); //leds en vert
      pixels.show(); //ça applique la couleur programmée
      delay(delayval); //on attend pendant "delayval" millisecondes avant d'allumer la prochaine led
    }
    for(int i=0;i<NUMPIXELS;i++)
    {
      pixels.setPixelColor(i, pixels.Color(0,0,0)); //on éteint toutes les leds
    }
    pixels.show(); //ça applique la couleur programmée
    delay(delayval); //on attend pendant "delayval" millisecondes avant de recommencer le chenillard
#endif
  }
}






































//----------------------------------------------------------------------------------------------
// SETUP
//----------------------------------------------------------------------------------------------
void setup()
{
  if(DEBUG)
  {
    Serial.begin(9600);
  }

#ifdef UTILISE_CAMERA
  //initialisation de la caméra
  pixy.init();
#endif

#ifdef UTILISE_LEDS
  //initialisation du bandeau de leds
  //pixels.begin(); //Pour utiliser les leds
#endif
  
  //initialisation de la carte moteur
  Carte_Moteur.begin();  //initialise le moteur avec une frequence par défaut 1.6KHz

  //initialisation des vitesses des moteurs selon le mode (AUTONOME ou TELEGUIDE)
  if (AUTONOME)
  {
    //vitesse max
    Moteur_G->setSpeed(4000);   
    Moteur_D->setSpeed(4000); 
    // initialisation du chronométrage du match
    MsTimer2::set(DUREE_MATCH, FinDeMatch); // période = duree du match, on activera avec la tirette
    //initialisation de la tirette      
    pinMode(PIN_TIRETTE, INPUT);  //en entrée
    pinMode(PIN_COULEUR_EQUIPE,INPUT_PULLUP); // choix de la couleur
    couleur_equipe=EQUIPE_BLEUE; //equipe violette par defaut
    bProgrammeDemarre=false; // le programme n'est pas demarre quand l'arduino s'allume
  }
  else
  {
    //on ouvre la communication série de la télécommande
    liaisonSerie.begin(9600);
    //vitesse obligatoire pour eviter blocage pc
    Moteur_G->setSpeed(2000);    
    Moteur_D->setSpeed(2000);      
  }
  
  
  pinMode(LED_BUILTIN, OUTPUT); // initialisation de la led interne de l'arduino
  digitalWrite(LED_BUILTIN, LOW);    // led eteinte
  
  pinMode(PIN_CONTACTEUR_AR_D,INPUT_PULLUP); // contacteur arrière droit
  pinMode(PIN_CONTACTEUR_AR_G,INPUT_PULLUP); // contacteur arrière gauche
  
  servo_01.attach(PIN_SERVO_01);
  /*
  servo_01.write(SERVO_01_TEST); 
  delay(1500);
  servo_01.write(SERVO_01_INIT); 
  */
  
  servo_02.attach(PIN_SERVO_02);
  /*
  servo_02.write(SERVO_02_TEST); 
  delay(1500);
  servo_02.write(SERVO_02_INIT); 
  */

  servo_03.attach(PIN_SERVO_03);
  /*
  servo_03.write(SERVO_03_TEST); 
  delay(1500);
  servo_03.write(SERVO_03_INIT); 
  */

  servo_04.attach(PIN_SERVO_04);
  /*
  servo_04.write(SERVO_04_TEST); 
  delay(1500);
  servo_04.write(SERVO_04_INIT); 
  */
  
  servo_05.attach(PIN_SERVO_05);
  /*
  servo_05.write(SERVO_05_TEST); 
  delay(1500);
  servo_05.write(SERVO_05_INIT); 
  */

  servo_06.attach(PIN_SERVO_06);
  /*
  servo_06.write(SERVO_06_TEST); 
  delay(1500);
  servo_06.write(SERVO_06_INIT); 
  */
}

































//----------------------------------------------------------------------------------------------
// BOUCLE INFINIE
//----------------------------------------------------------------------------------------------
void loop() {
 
  if (!AUTONOME)
//----------------------------------------------------------------------------------------------
// MODE TELEGUIDE
//----------------------------------------------------------------------------------------------
  {
    while (liaisonSerie.available())
    {
      delay(10);
      char b = liaisonSerie.read();
     
      if(b==MESSAGE_JAUNE)
      {
        if(DEBUG) Serial.println("JAUNE");
      }
      if(b==MESSAGE_VERT)
      {
        if(DEBUG) Serial.println("VERT");
      }
      if(b==MESSAGE_BLEU)
      {
        if(DEBUG) Serial.println("BLEU");
      }
      if(b==MESSAGE_ROUGE)
      {
        if(DEBUG) Serial.println("ROUGE");
      }
      if(b==MESSAGE_CLICK)
      {
        if(DEBUG) Serial.println("CLICK");
      }
      //actions de joystick
      if(b==MESSAGE_GAUCHE_RAPIDE)
      {
        if(DEBUG) Serial.println("GAUCHE");
        VITESSE(RAPIDE);
        TOURNER_DE(-1);
      }
      if(b==MESSAGE_GAUCHE_LENT)
      {
        VITESSE(LENTE);
        TOURNER_DE(-1);
      }
      if(b==MESSAGE_DROITE_LENT)
      {
        VITESSE(LENTE);
        TOURNER_DE(1);  
      }
      if(b==MESSAGE_DROITE_RAPIDE)
      {
        if(DEBUG) Serial.println("DROITE");
        VITESSE(RAPIDE);
        TOURNER_DE(1);
      }

      if(b==MESSAGE_ARRIERE_RAPIDE)
      {
        if(DEBUG) Serial.println("ARRIERE");
        VITESSE(RAPIDE);
        RECULER_UN_PAS();
      }
      if(b==MESSAGE_ARRIERE_LENT)
      {
        VITESSE(LENTE);
        RECULER_UN_PAS();
      }
      if(b==MESSAGE_AVANT_LENT)
      {
        VITESSE(LENTE);
        AVANCER_UN_PAS();  
      }
      if(b==MESSAGE_AVANT_RAPIDE)
      {
        if(DEBUG) Serial.println("AVANT");
        VITESSE(RAPIDE);
        AVANCER_UN_PAS();
      }
    } 
  }
  else
//----------------------------------------------------------------------------------------------
// MODE AUTONOME
//----------------------------------------------------------------------------------------------  
  {
     //---------------------------------------------
     //CHOIX COULEUR
     //---------------------------------------------
    int valeur_bouton=analogRead(PIN_COULEUR_EQUIPE);     
    if(valeur_bouton<800) //bouton vers le haut
    {
      couleur_equipe=EQUIPE_BLEUE;
#ifdef UTILSE_LEDS
      METTRE_LEDS_A(LEDS_BLEU);
#endif
    }
    else
    {
      couleur_equipe=EQUIPE_JAUNE;
#ifdef UTILSE_LEDS
      METTRE_LEDS_A(LEDS_ORANGE);
#endif
    }

     //---------------------------------------------
     //LANCEMENT PROGRAMME
     //---------------------------------------------
    if ((digitalRead(PIN_TIRETTE)==LOW)&&(bProgrammeDemarre==false))
    {
      bProgrammeDemarre=true; // le programme est demarre

      //on vérifie la couleur de l'équipe avant de lancer le programme (pour être sûr)
      int valeur=analogRead(PIN_COULEUR_EQUIPE);
      if(valeur<800) //bouton vers le haut
        couleur_equipe=EQUIPE_BLEUE;
      else
        couleur_equipe=EQUIPE_JAUNE;

      //on lance le chrono du match
      MsTimer2::start(); // active Timer2

      //on allume la LED interne de l'arduino indiquant qu'on a enlevé le tirette
      digitalWrite(LED_BUILTIN, HIGH); // led allumee

      //selon la couleur de l'équipe on déroule un programme
      if (couleur_equipe==EQUIPE_BLEUE)
      {
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
        RECALAGE(15);
        AVANCER(20);
        TOURNER_DE(110);
        RECALAGE(15);
        AVANCER(33);
        TOURNER_DE(-110);
        RECALAGE(15);
        AVANCER(20);
      } 
    }
  }
}
