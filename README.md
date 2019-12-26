# TROPHEES_BOT_ARDUINO
SW du robot des trophées

Le robot utilise le shield arduino "[adafruit motor shield v2](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino)". Tous les ponts H sont utilisés depuis le passage aux moteurs pas à pas pour le robot.
Pour l'instant l'arduino uno se révèle suffisant (sinon le shield est compatible avec un arduino Mega).

Il y a quelques limitations d'utilisations des entrées/sorties (certaines sont utilisées par le shield). Ces limitations sont expliquées dans le code.

Le code est également prêt pôur une utilisation de:
NeoPixel un bandeau de leds
PixyCam la dernière version de la cmucam (site [PixyCam](https://pixycam.com/pixy-cmucam5/))

Les librairies suivantes sont nécessaires pour compiler le programme:  
 - **SoftwareSerial.h** : pour la communication avec la télécommande  
 - **messagerieTB.h** : messagerie de la télécommande  
 - **Wire.h** : pour l'i2c utilisé par le shield  
 - **Pixy2.h** : pour piloter la caméra  
 - **Adafruit_MotorShield.h** : pour commander le motor shield  
 - **MsTimer2.h** : pour le timer de la durée du match  
 - **Servo.h** : pour commander les servomoteurs  
 - **Adafruit_NeoPixel.h** : pour piloter le bandeau de leds  

