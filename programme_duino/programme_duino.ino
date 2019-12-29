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
#define AUTONOME false
//pour debuguer le programme, mettre à true
#define DEBUG false
//à décommenter pour utiliser le bandeau de LEDS
//#define UTILISE_LEDS true
//à décommenter pour utiliser la camera
//#define UTILISE_CAMERA true


//CONFIGURATION DES PORTS UTILISES SUR L'ARDUINO MEGA
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
#define PIN_COULEUR_EQUIPE  A0 //bouton de choix de couleur d'équipe pour le mode autonome
#define PIN_CONTACTEUR_AR_D A1 //contacteurs arrière droit pour le recalage
#define PIN_CONTACTEUR_AR_G A2 //contacteurs arrière gauche pour le recalage
#define PIN_A3              A3 //pas encore utilisé
#define PIN_A4              A4 //pas encore utilisé
#define PIN_A5              A5 //pas encore utilisé
#define PIN_A6              A6 //pas encore utilisé
#define PIN_A7              A7 //pas encore utilisé
#define PIN_A8              A8 //pas encore utilisé
#define PIN_A9              A9 //pas encore utilisé
#define PIN_A10             A10 //pas encore utilisé
#define PIN_A11             A11 //pas encore utilisé
#define PIN_A12             A12 //pas encore utilisé
#define PIN_A13             A13 //pas encore utilisé
#define PIN_A14             A14 //pas encore utilisé
#define PIN_A15             A15 //pas encore utilisé
#define PIN_00              0 //pas encore utilisé
#define PIN_02              1 //pas encore utilisé
#define PIN_03              2 //PWM - pas encore utilisé
#define PIN_TIRETTE         3 //capteur optique de la tirette pour le mode autonome
#define PIN_LEDS            4 //fil de communication avec le bandeau de leds
#define PIN_SERVO_01        5
#define PIN_SERVO_02        6
#define PIN_SERVO_03        7
#define PIN_SERVO_04        8
#define PIN_SERVO_05        9
#define PIN_RX              10 //voie de réception de la télécommande pour le mode téléguidé
#define PIN_TX              11 //voie d'envoi de la télécommande pour le mode téléguidé
#define PIN_PWM_GAUCHE      12 //PWM - commande puissance moteur gauche (orange)
#define PIN_PWM_DROIT       13 //PWM - commande puissance moteur droit (gris)
#define PIN_14              14 //pas encore utilisé
#define PIN_15              15 //pas encore utilisé
#define PIN_16              16 //pas encore utilisé
#define PIN_17              17 //pas encore utilisé
#define PIN_18              18 //pas encore utilisé
#define PIN_19              19 //pas encore utilisé
#define PIN_SDA             20 //ATTENTION utilisé par le motor shield
#define PIN_SCL             21 //ATTENTION utilisé par le motor shield
#define PIN_22              22 //pas encore utilisé
#define PIN_23              23 //pas encore utilisé
#define PIN_24              24 //pas encore utilisé
#define PIN_25              25 //pas encore utilisé
#define PIN_26              26 //pas encore utilisé
#define PIN_27              27 //pas encore utilisé
#define PIN_28              28 //pas encore utilisé
#define PIN_29              29 //pas encore utilisé
#define PIN_30              30 //pas encore utilisé
#define PIN_31              31 //pas encore utilisé
#define PIN_32              32 //pas encore utilisé
#define PIN_33              33 //pas encore utilisé
#define PIN_34              34 //pas encore utilisé
#define PIN_35              35 //pas encore utilisé
#define PIN_36              36 //pas encore utilisé
#define PIN_37              37 //pas encore utilisé
#define PIN_38              38 //pas encore utilisé
#define PIN_39              39 //pas encore utilisé
#define PIN_40              40 //pas encore utilisé
#define PIN_41              41 //pas encore utilisé
#define PIN_42              42 //pas encore utilisé
#define PIN_43              43 //pas encore utilisé
#define PIN_44              44 //PWM - pas encore utilisé
#define PIN_45              45 //PWM - pas encore utilisé
#define PIN_46              46 //PWM - pas encore utilisé
#define PIN_47              47 //pas encore utilisé
#define PIN_48              48 //pas encore utilisé
#define PIN_49              49 //pas encore utilisé
#define PIN_CMD_1_GAUCHE    50 //commande n°1 moteur gauche (jaune)
#define PIN_CMD_1_DROIT     51 //commande n°1 moteur droit (violet)
#define PIN_CMD_2_GAUCHE    52 //commande n°2 moteur gauche (vert)
#define PIN_CMD_2_DROIT     53 //commande n°2 moteur droit (bleu)

//parametrage du match
#define DUREE_MATCH               100000 //en millisecondes
#define DISTANCE_PAR_PAS_MOTEUR   0.1099 //1 pas = K cm
#define EQUIPE_JAUNE              0
#define EQUIPE_BLEUE              1

//parametrage divers
#define MOTEUR_GAUCHE             2 //identifiant moteur gauche
#define MOTEUR_DROIT              1 //identifiant moteur droit
#define AVANT                     1 //sens des moteurs des roues
#define ARRIERE                   -1 //sens des moteurs des roues
#define STOP                      0 //sens des moteurs des roues
#define VITESSE_PAP_LENTE         120 //vitesse des moteurs pas à pas
#define VITESSE_PAP_RAPIDE        120 //vitesse des moteurs pas à pas
#define NBRE_DE_LEDS              3 //nombre de leds
#define LEDS_BLEU                 1 //couleur bleue pour les leds
#define LEDS_ORANGE               2 //couleur orange pour les leds

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

//----------------------------------------------------------------------------------------------
// DECLARATION VARIABLES: MOTEUR, CAMERA, LEDS,...
//----------------------------------------------------------------------------------------------
//on informe l'arduino que l'on utilise ses broches PIN_RX et PIN_TX pour une connexion série pour la télécommande
SoftwareSerial liaisonSerie(PIN_RX, PIN_TX);

//Création variables globales
bool bProgrammeDemarre; // variable qui indique si le programme est demarre (demarre==true), utilisé par la tirette
int couleur_equipe;
char etatTelecommande; //dernier état reçu de la télécommande (pour éviter de traiter toujours le même ordre)

//Création des servos
Servo servo_01;
Servo servo_02;
Servo servo_03;
Servo servo_04;
Servo servo_05;

//Création des 2 moteurs pas à pas
// Création d'une carte moteur avec l'adresse I2C par défaut
Adafruit_MotorShield Carte_Moteur = Adafruit_MotorShield();
// Connection de 2 moteurs pas à pas de 200 pas par tour (1.8 degrés)
Adafruit_StepperMotor *Moteur_G = Carte_Moteur.getStepper(200, 1);// moteur_G (M1 et M2 connectés sur le shield)
Adafruit_StepperMotor *Moteur_D = Carte_Moteur.getStepper(200, 2);// moteur_D (M3 et M4 connectés sur le shield)

//création des leds
#ifdef UTILISE_LEDS
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NBRE_DE_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);
#endif

//création caméra
#ifdef UTILISE_CAMERA
  Pixy2 pixy;
#endif


//----------------------------------------------------------------------------------------------
// DEFINITION FONCTIONS: TOURNER_N_TOURS, RECULER,...
//----------------------------------------------------------------------------------------------

/**
 * FONCTION ACTION_MOTEUR
 * contrôle les moteurs des roues
 * Nécessite 3 valeurs: le moteur choisi "moteur", le sens de rotation "sens" et la puissance choisie entre 0% et 100% "pourcentage"
 * Rappel des commandes pour un pont H
 * _______________________________________________________________
 * pin1Moteur | pin2Moteur  | Comportement du moteur              |
 * _______________________________________________________________
 * HIGH       | LOW         | Le moteur tourne dans un sens       |
 * _______________________________________________________________
 * LOW        | HIGH        | Le moteur tourne dans l'autre sens  |
 * _______________________________________________________________
 * LOW        | LOW         | Frein électromagnétique             |
 * _______________________________________________________________
 * HIGH       | HIGH        | Frein électromagnétique             |
 * _______________________________________________________________
 */
void ACTION_MOTEUR(int moteur,int sens,int pourcentage){
  int pin1,etat1,pin2,etat2,pinPWM,puissance; //variable de la fonction
  //test numéro du moteur
  if (moteur==MOTEUR_DROIT)
  {
    pin1=PIN_CMD_1_DROIT;
    pin2=PIN_CMD_2_DROIT;
    pinPWM=PIN_PWM_DROIT;
  }
  if (moteur==MOTEUR_GAUCHE)
  {
    pin1=PIN_CMD_1_GAUCHE;
    pin2=PIN_CMD_2_GAUCHE;
    pinPWM=PIN_PWM_GAUCHE;
  }
  //test sens du moteur AVANT/ARRIERE/STOP
  if (sens==AVANT){
    etat1=1;
    etat2=0;
  }
  if (sens==ARRIERE){
    etat1=0;
    etat2=1;
  }
  if (sens==STOP) {
    etat1=0;
    etat2=0;
  }
  //on transforme la valeur de puissance en % en valeur comprise entre 0 et 255 pour que ce soit compréhensible par l'arduino
  puissance=map(pourcentage,0,100,0,255);
  //on passe les commandes à l'arduino
  analogWrite(pinPWM,puissance);
  digitalWrite(pin1,etat1);
  digitalWrite(pin2,etat2);
  //affichage sur le moniteur série si le débugage est activé
  if(DEBUG)
  {
    if (moteur==MOTEUR_DROIT) Serial.print("Moteur : droit");
    if (moteur==MOTEUR_GAUCHE) Serial.print("Moteur : gauche");
    if (sens==AVANT) Serial.print(" \tsens : avant");
    if (sens==ARRIERE) Serial.print(" \tsens : arrière");
    if (sens==STOP) Serial.print("\t! stop ! ");
    Serial.print("\tpourcentage : ");
    Serial.print(pourcentage);
    Serial.print("\tpuissance : ");
    Serial.println(puissance);
  }
}

/*
 * FONCTION AVANCER
 * fait avancer le robot
 * demande une valeur : la puissance moteur entre 0 et 100%
 */
void AVANCER(int puissance)
{
    ACTION_MOTEUR(MOTEUR_GAUCHE,AVANT,puissance);  
    ACTION_MOTEUR(MOTEUR_DROIT,AVANT,puissance); 
}

/*
 * FONCTION RECULER
 * fait reculer le robot
 * demande une valeur : la puissance moteur entre 0 et 100%
 */
void RECULER(int puissance)
{
    ACTION_MOTEUR(MOTEUR_GAUCHE,ARRIERE,puissance);  
    ACTION_MOTEUR(MOTEUR_DROIT,ARRIERE,puissance); 
}

/*
 * FONCTION STOPPER
 * fait s'arrêter le robot
 * demande aucune valeur
 */
void STOPPER()
{
    ACTION_MOTEUR(MOTEUR_GAUCHE,STOP,0);  
    ACTION_MOTEUR(MOTEUR_DROIT,STOP,0); 
}

/*
 * FONCTION TOURNER_GAUCHE
 * fait tourner vers la gauche le robot
 * demande une valeur : la puissance moteur entre 0 et 100%
 */
void TOURNER_GAUCHE(int puissance)
{
    ACTION_MOTEUR(MOTEUR_GAUCHE,ARRIERE,puissance);  
    ACTION_MOTEUR(MOTEUR_DROIT,AVANT,puissance); 
}

/*
 * FONCTION TOURNER_DROITE
 * fait tourner vers la droite le robot
 * demande une valeur : la puissance moteur entre 0 et 100%
 */
void TOURNER_DROITE(int puissance)
{
    ACTION_MOTEUR(MOTEUR_GAUCHE,AVANT,puissance);  
    ACTION_MOTEUR(MOTEUR_DROIT,ARRIERE,puissance); 
}

#ifdef UTILISE_LEDS
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
        delay(500); //on attend 500 millisecondes
      }
  }
#endif
/*
 * FONCTION VITESSE_MOTEURS_PAS_A_PAS
 * positionne la vitesse des moteurs pas à pas gauche et droite
 * Demande une valeur qui est la vitesse de rotation en tours par minute
 */
void VITESSE_MOTEURS_PAS_A_PAS(int valeur_vitesse)
{
    Moteur_G->setSpeed(valeur_vitesse);   
    Moteur_D->setSpeed(valeur_vitesse); 
}

/*
 * FONCTION TOURNER_N_TOURS
 * Fait tourner de plusieurs tours les moteurs pas à pas.
 * Demande deux valeurs qui sont le nombre de tours à faire et le sens de rotation
 */
void TOURNER_N_TOURS(float n, int sens)
{
  int pas = n*200;
  if(sens==AVANT)
  {
    for(int i=0;i<pas;i++)
    {
      Moteur_G->step(1,BACKWARD,SINGLE); //en avant
      Moteur_D->step(1,FORWARD,SINGLE); //en avant
    }
  }
  if(sens==ARRIERE)
  {
    for(int i=0;i<pas;i++)
    {
      Moteur_G->step(1,FORWARD,SINGLE); //en arriere
      Moteur_D->step(1,BACKWARD,SINGLE); //en arriere
    }
  }
}

/*
 * FONCTION TOURNER_UN_PAS
 * Fait touner le moteur pas à pas droit et gauche d'un pas
 * Demande le sens de rotation du moteur
 */
void TOURNER_UN_PAS(int sens)
{
    if(sens==AVANT)
    {
      Moteur_G->step(1,BACKWARD,SINGLE); //en avant
      Moteur_D->step(1,FORWARD,SINGLE); //en avant
    }
    if(sens==ARRIERE)
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
      delay(500); //on attend 500 millisecondes avant d'allumer la prochaine led
    }
    for(int i=0;i<NUMPIXELS;i++)
    {
      pixels.setPixelColor(i, pixels.Color(0,0,0)); //on éteint toutes les leds
    }
    pixels.show(); //ça applique la couleur programmée
    delay(500); //on attend 500 millisecondes avant de recommencer le chenillard
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
    Serial.println("Debug activé\n\n");
  }

#ifdef UTILISE_CAMERA
  //initialisation de la caméra
  pixy.init();
#endif

#ifdef UTILISE_LEDS
  //initialisation du bandeau de leds
  pixels.begin(); //Pour utiliser les leds
#endif

  //on utilise les pinoches réservées aux moteurs des roues
  pinMode(PIN_PWM_GAUCHE,OUTPUT); //commande puissance moteur gauche
  pinMode(PIN_PWM_DROIT,OUTPUT); //commande puissance moteur droit
  pinMode(PIN_CMD_1_GAUCHE,OUTPUT); //commande n°1 moteur gauche
  pinMode(PIN_CMD_1_DROIT,OUTPUT); //commande n°1 moteur droit
  pinMode(PIN_CMD_2_GAUCHE,OUTPUT); //commande n°2 moteur gauche
  pinMode(PIN_CMD_2_DROIT,OUTPUT); //commande n°2 moteur droit
  
  //initialisation de la carte moteur pour utiliser les moteurs pas à pas
  Carte_Moteur.begin();  //initialise le moteur avec une frequence par défaut 1.6KHz

  //initialisation des vitesses des moteurs selon le mode (AUTONOME ou TELEGUIDE)
  if (AUTONOME)
  {
    //initialisation du chronométrage du match
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
    etatTelecommande=MESSAGE_STOP;
  }

  //quel que soit le cas de figure les roues sont arretées au début
  STOPPER();
  
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
      char donneeRecue = liaisonSerie.read();
     
      if(donneeRecue==MESSAGE_JAUNE)
      {
        if(DEBUG) Serial.println("JAUNE");
      }
      if(donneeRecue==MESSAGE_VERT)
      {
        if(DEBUG) Serial.println("VERT");
      }
      if(donneeRecue==MESSAGE_BLEU)
      {
        if(DEBUG) Serial.println("BLEU");
      }
      if(donneeRecue==MESSAGE_ROUGE)
      {
        if(DEBUG) Serial.println("ROUGE");
      }
      if(donneeRecue==MESSAGE_CLICK)
      {
        if(DEBUG) Serial.println("CLICK");
      }
      //actions de joystick
      if((donneeRecue==MESSAGE_STOP)&&(etatTelecommande!=MESSAGE_STOP))
      {
        if(DEBUG) Serial.println("STOP");
        etatTelecommande=MESSAGE_STOP;
        STOPPER();
      }
      if((donneeRecue==MESSAGE_GAUCHE_RAPIDE)&&(etatTelecommande!=MESSAGE_GAUCHE_RAPIDE))
      {
        if(DEBUG) Serial.println("GAUCHE");
        etatTelecommande=MESSAGE_GAUCHE_RAPIDE;
        TOURNER_GAUCHE(50);
        
      }
      if((donneeRecue==MESSAGE_GAUCHE_LENT)&&(etatTelecommande!=MESSAGE_GAUCHE_LENT))
      {
        etatTelecommande=MESSAGE_GAUCHE_LENT;
        TOURNER_GAUCHE(20);
      }
      if((donneeRecue==MESSAGE_DROITE_LENT)&&(etatTelecommande!=MESSAGE_DROITE_LENT))
      {
        etatTelecommande=MESSAGE_DROITE_LENT;
        TOURNER_DROITE(20);
      }
      if((donneeRecue==MESSAGE_DROITE_RAPIDE)&&(etatTelecommande!=MESSAGE_DROITE_RAPIDE))
      {
        if(DEBUG) Serial.println("DROITE");
        etatTelecommande=MESSAGE_DROITE_RAPIDE;
        TOURNER_DROITE(50);
      }

      if((donneeRecue==MESSAGE_ARRIERE_RAPIDE)&&(etatTelecommande!=MESSAGE_ARRIERE_RAPIDE))
      {
        if(DEBUG) Serial.println("ARRIERE");
        etatTelecommande=MESSAGE_ARRIERE_RAPIDE;
        RECULER(50);
      }
      if((donneeRecue==MESSAGE_ARRIERE_LENT)&&(etatTelecommande!=MESSAGE_ARRIERE_LENT))
      {
        etatTelecommande=MESSAGE_ARRIERE_LENT;
        RECULER(20);
      }
      if((donneeRecue==MESSAGE_AVANT_LENT)&&(etatTelecommande!=MESSAGE_AVANT_LENT))
      {
        etatTelecommande=MESSAGE_AVANT_LENT;
        AVANCER(20);
      }
      if((donneeRecue==MESSAGE_AVANT_RAPIDE)&&(etatTelecommande!=MESSAGE_AVANT_RAPIDE))
      {
        if(DEBUG) Serial.println("AVANT");
        etatTelecommande=MESSAGE_AVANT_RAPIDE;
        AVANCER(50);
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
                  
      }
      else
      {
        
      } 
    }
  }
}
