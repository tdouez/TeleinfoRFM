//--------------------------------------------------------------------
//                 +/
//                 `hh-
//        ::        /mm:
//         hy`      -mmd
//         omo      +mmm.  -+`
//         hmy     .dmmm`   od-
//        smmo    .hmmmy    /mh
//      `smmd`   .dmmmd.    ymm
//     `ymmd-   -dmmmm/    omms
//     ymmd.   :mmmmm/    ommd.
//    +mmd.   -mmmmm/    ymmd-
//    hmm:   `dmmmm/    smmd-
//    dmh    +mmmm+    :mmd-
//    omh    hmmms     smm+
//     sm.   dmmm.     smm`
//      /+   ymmd      :mm
//           -mmm       +m:
//            +mm:       -o
//             :dy
//              `+:
//--------------------------------------------------------------------
//   __|              _/           _ )  |
//   _| |  |   ` \    -_)   -_)    _ \  |   -_)  |  |   -_)
//  _| \_,_| _|_|_| \___| \___|   ___/ _| \___| \_,_| \___|
//--------------------------------------------------------------------
// Code mise à disposition selon les termes de la Licence Creative Commons Attribution
// Pas d’Utilisation Commerciale.
// Partage dans les Mêmes Conditions 4.0 International.
//--------------------------------------------------------------------
// Trame : $origine;libelle;donnees   (libellé & donnnées)
//--------------------------------------------------------------------
// 2021/01/15 - FB V1.0.0
// 2021/06/11 - FB V1.0.1
// 2021/07/01 - FB V1.0.2 - Add ETIQU_URMS1..3
// 2021/11/21 - FB V1.0.3 - Bug fix  modes triphasé et producteur
// 2021/12/08 - FB V2.0.0 - Utilisation de la librairie libTeleinfo (https://github.com/hallard/LibTeleinfo) et chg protocole vers GW
// 2021/12/27 - FB V2.0.1 - Fix bug aléatoire, blocage envoi data
// 2022/01/07 - FB V2.0.2 - Optimisation utilisation mémoire et emissions datas. Ajout détection vitesse TIC.
// 2022/01/11 - FB V2.0.3 - Suppression warning et modif gestion timeout vitesse TIC
// 2022/01/23 - FB V2.0.4 - Correction sur détection TIC historique
// 2022/06/25 - FB V2.0.5 - Optimisation mémoire
// 2022/09/25 - FB V2.0.6 - Correction sur détection TIC standard
// 2023/03/23 - FB V2.0.7 - Ajout indication réception TIC et optimisation mémoire
// 2023/03/30 - FB V2.0.8 - Correction blocage aléatoire module, frequence maj 15 à 20s, clignotement durant charge condo
//--------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <LibTeleinfo.h>

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

#define VERSION   "v2.0.8"

#define ENTETE  "$"

#define POWER_PIN         4
#define TELEINFO_LED_PIN  3   // Teleinfo LED
#define CHARGE_LED_PIN    5   // Charge LED

//-------------------------------------------------------------------------------------
// Seuil : 4,5v à l'entrée du régulateur 3,3v > 4,5v*100k/(100k+68k)=2,68v sur A0
//                                                 3,3v->1023 => 2,68->830
//-------------------------------------------------------------------------------------
#define SEUIL_CHARGE_FULL   820
#define SEUIL_CHARGE_98     810

#define RFM_TX_POWER   10   // 5..23 dB

boolean full_tic = true;
boolean flag_tic = true;
boolean flag_send_tic = true;
boolean flag_adresse_tic;
boolean flag_boot;

const char char_ADCO[] PROGMEM = "ADCO";
const char char_ADSC[] PROGMEM = "ADSC";
const unsigned long SEND_FREQUENCY_FULL = 180000; // 3mn, Minimum time between send (in milliseconds). 
const unsigned long SEND_FREQUENCY_TIC  =  20000; // 15s,  Minimum time between send (in milliseconds). 
unsigned long lastTime_full = 0;
unsigned long lastTime_tic = 0;
char adresse_compteur[13];
_Mode_e mode_tic;

TInfo tinfo;

/*
// ---------------------------------------------------------------- 
// freeMemory
// ---------------------------------------------------------------- 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

// ---------------------------------------------------------------- 
// freeMemory
// ---------------------------------------------------------------- 
void affiche_freeMemory() {
	Serial.print(F("SRAM:"));
	Serial.println(freeMemory());
}
*/

// ---------------------------------------------------------------- 
// change_etat_led
// ---------------------------------------------------------------- 
void change_etat_led(uint8_t led)
{
  static uint8_t led_state;

  led_state = !led_state;
  digitalWrite(led, led_state);

}

// ---------------------------------------------------------------- 
// clignote_led
// ---------------------------------------------------------------- 
void clignote_led(uint8_t led, uint8_t nbr, int16_t delais)
{
uint8_t led_state;

  for (uint8_t i=0; i<nbr*2; i++) {
    led_state = !led_state;
    digitalWrite(led, led_state);
    delay(delais);
  }
  digitalWrite(led, LOW);
}

// ---------------------------------------------------------------- 
// init_speed_TIC
// ---------------------------------------------------------------- 
_Mode_e init_speed_TIC()
{
boolean flag_timeout = false;
boolean flag_found_speed = false;
uint32_t currentTime = millis();
uint8_t step = 0;
_Mode_e mode;

  digitalWrite(TELEINFO_LED_PIN, HIGH);
  digitalWrite(CHARGE_LED_PIN, HIGH);
  
  // Test en mode historique
  // Recherche des éléments de début, milieu et fin de trame 
  Serial.begin(1200); // mode historique
  while (!flag_timeout && !flag_found_speed) {
    if (Serial.available()>0) {
      char in = (char)Serial.read() & 127;  // seulement sur 7 bits
      // début trame
      if (in == 0x0A) {
        step = 1;
      }
      // premier milieu de trame
        if (step == 1 && in == 0x20) {
        step = 2;
      }
      // deuxième milieu de trame
        if (step == 2 && in == 0x20) {
        step = 3;
      }
      // fin trame
        if (step == 3 && in == 0x0D) {
        flag_found_speed = true;
        step = 0;
      }
    }
    if (currentTime + 10000 <  millis()) flag_timeout = true; // 10s de timeout
  }

  if (flag_timeout) { // trame avec vistesse histo non trouvée donc passage en mode standard
     mode = TINFO_MODE_STANDARD;
     Serial.end();
     Serial.begin(9600); // mode standard
     Serial.println(F("TIC mode standard"));
     clignote_led(CHARGE_LED_PIN, 3, 500);
  }
  else {
    mode = TINFO_MODE_HISTORIQUE;
    Serial.println(F("TIC mode historique"));
    clignote_led(CHARGE_LED_PIN, 5, 400);
  }
  
  digitalWrite(TELEINFO_LED_PIN, LOW);

  return mode;
}

// ---------------------------------------------------------------- 
// send_boot
//    Envoi info au démarrge avec version
// ---------------------------------------------------------------- 
void send_boot() 
{

  verif_charge_condo(SEUIL_CHARGE_FULL);
  start_rf95();
  LoRa.beginPacket();    // start packet
  LoRa.print(ENTETE);  
  LoRa.print(F("Linky_")); 
  LoRa.print(adresse_compteur); 
  LoRa.print(F(";VERSION;")); 
  LoRa.print(VERSION); 
  LoRa.print(F(";"));
  LoRa.endPacket();      // finish packet and send it
  delay(20);
  stop_rf95();
}

// ---------------------------------------------------------------- 
// search_adress_teleinfo
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
void search_adress_teleinfo(ValueList *vl_tic)
{

  if (vl_tic) {
    // parcours liste chainée vl_tic
    while (vl_tic->next) {
      vl_tic = vl_tic->next;
      if (vl_tic->name && strlen(vl_tic->name) && vl_tic->value && strlen(vl_tic->value)) {
        if (strstr_P(vl_tic->name, char_ADCO) == 0 || strstr_P(vl_tic->name, char_ADSC) == 0) {
          strncpy(adresse_compteur, vl_tic->value, 12);
          Serial.print(F("Adr cpt: "));
          Serial.println(adresse_compteur);
          flag_adresse_tic = true;
          break;
         }
      }
    }
  }
  digitalWrite(TELEINFO_LED_PIN, LOW);
}

// ---------------------------------------------------------------- 
// charge_condo
//    charge le super condo avec gestion led 
// ---------------------------------------------------------------- 
void charge_condo(int ref_seuil)
{
boolean flag_cap = false;

  Serial.print(F(">> Charge Condo.."));
  digitalWrite(CHARGE_LED_PIN, HIGH);
  digitalWrite(TELEINFO_LED_PIN, LOW);

  while (flag_cap == false) {
	if (analogRead(A0) >= ref_seuil) flag_cap = true;
	else {
		change_etat_led(CHARGE_LED_PIN);
		delay(300);
	}
  }
  digitalWrite(CHARGE_LED_PIN, LOW);
  Serial.println(F(". done"));
}

// ---------------------------------------------------------------- 
// test_charge_condo
//    test niveau tension sur le super condo. 
//    retourne vrai si niveau suffisant
// ---------------------------------------------------------------- 
boolean test_charge_condo(int ref_seuil)
{
  if (analogRead(A0) < ref_seuil) return false;

  return true;
}

// ---------------------------------------------------------------- 
// test_charge_condo
//    verif niveau tension sur le super condo
//    lance la charge si niveau insuffisant
// ---------------------------------------------------------------- 
void verif_charge_condo(int ref_seuil)
{
  if (test_charge_condo(ref_seuil) == false) charge_condo(ref_seuil);  
}

// ---------------------------------------------------------------- 
// send_teleinfo
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
void send_teleinfo(ValueList *vl_tic, boolean all_tic)
{
  
  if (vl_tic && flag_tic) {

    // parcours liste chainée vl_tic
    while (vl_tic->next) {
      vl_tic = vl_tic->next;
      // uniquement sur les nouvelles valeurs ou celles modifiées ou toutes
      if ( all_tic || ( vl_tic->flags & (TINFO_FLAGS_UPDATED | TINFO_FLAGS_ADDED) )) {
        if (vl_tic->name && strlen(vl_tic->name) > 2 && vl_tic->value && strlen(vl_tic->value) > 1) {
          Serial.print(F("Send "));
          Serial.print(vl_tic->name);
          Serial.print(F(":"));
          Serial.println(vl_tic->value);

    		  verif_charge_condo(SEUIL_CHARGE_98);
			    start_rf95();
    		  LoRa.beginPacket();    // start packet
    		  LoRa.print(ENTETE);  
    		  LoRa.print(F("Linky_")); 
    		  LoRa.print(adresse_compteur); 
    		  LoRa.print(F(";"));
    		  LoRa.print(vl_tic->name); 
    		  LoRa.print(F(";"));
    		  LoRa.print(vl_tic->value); 
    		  LoRa.print(F(";"));
    		  LoRa.endPacket();      // finish packet and send it
			    delay(20);
			    stop_rf95();
        }
      }
    }
    flag_tic = false;
  }
}

/* ======================================================================
Function: NewFrame 
Purpose : callback when we received a complete teleinfo frame
Input   : linked list pointer on the concerned data
Output  : - 
Comments: -
====================================================================== */
void NewFrame(ValueList *vl_tic)
{
  //affiche_freeMemory();
  
  if (!flag_adresse_tic) search_adress_teleinfo(vl_tic);
    else send_teleinfo(vl_tic, full_tic);
    
  if (full_tic) full_tic = false;
  
}

/* ======================================================================
Function: NewFrame 
Purpose : callback when we received a complete teleinfo frame
Input   : linked list pointer on the concerned data
Output  : - 
Comments: it's called only if one data in the frame is different than
          the previous frame
====================================================================== */
void UpdatedFrame(ValueList *vl_tic)
{
  //affiche_freeMemory();
  
  if (!flag_adresse_tic) search_adress_teleinfo(vl_tic);
    else send_teleinfo(vl_tic, full_tic);
    
  if (full_tic) full_tic = false;
}

/* ======================================================================
Function: DataCallback 
Purpose : callback when we detected new or modified data received
Input   : linked list pointer on the concerned data
          current flags value
Output  : - 
Comments: -
====================================================================== */
void DataCallback(ValueList * me, uint8_t  flags)
{

  change_etat_led(TELEINFO_LED_PIN);
    
}

// ---------------------------------------------------------------- 
// start_rf95
// ---------------------------------------------------------------- 
boolean start_rf95()
{
boolean rc = false;

  // activation alim et init RFM
  digitalWrite(POWER_PIN, HIGH);
  if (LoRa.begin(868E6)) {
	LoRa.enableCrc();
	LoRa.setTxPower(RFM_TX_POWER);
	rc = true;
  }
  else {
	rc = false;
	Serial.println(F("Starting LoRa failed!"));
  }
  
  return rc;
}

// ---------------------------------------------------------------- 
// stop_rf95
// ---------------------------------------------------------------- 
void stop_rf95()
{
	digitalWrite(POWER_PIN, LOW);
}

// ---------------------------------------------------------------- 
// setup
// ---------------------------------------------------------------- 
void setup()
{
  pinMode(TELEINFO_LED_PIN, OUTPUT);
  pinMode(CHARGE_LED_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  
  digitalWrite(POWER_PIN, LOW);
  digitalWrite(TELEINFO_LED_PIN, LOW);
  digitalWrite(CHARGE_LED_PIN, LOW);

  // init interface série suivant mode TIC
  mode_tic = init_speed_TIC();
  //Serial.begin(MODE_TIC == TINFO_MODE_HISTORIQUE ? 1200 : 9600);

  Serial.println(F("   __|              _/           _ )  |"));
  Serial.println(F("   _| |  |   ` \\    -_)   -_)    _ \\  |   -_)  |  |   -_)"));
  Serial.println(F("  _| \\_,_| _|_|_| \\___| \\___|   ___/ _| \\___| \\_,_| \\___|"));
  Serial.print(F("                                             "));
  Serial.println(VERSION);

  // première charge super condo
  verif_charge_condo(SEUIL_CHARGE_FULL);

  flag_boot = true;
  flag_adresse_tic = false;
  
  // init interface TIC
  tinfo.init(mode_tic);
  tinfo.attachData(DataCallback);
  tinfo.attachNewFrame(NewFrame);
  tinfo.attachUpdatedFrame(UpdatedFrame);

}


// ---------------------------------------------------------------- 
// loop
// ---------------------------------------------------------------- 
void loop()
{
uint32_t currentTime = millis();

  if (Serial.available()) tinfo.process(Serial.read());
  
  if (flag_adresse_tic) {
	  
    if (flag_boot) {
      // envoi version au démarrage
      send_boot();
      flag_boot = false;
    }
    
    // envoyer l'ensemble des données tous les SEND_FREQUENCY_FULL ms
    if (currentTime - lastTime_full > SEND_FREQUENCY_FULL) {
      //Serial.println(F("SEND_FREQUENCY_FULL"));
      full_tic = true;
      flag_tic = true;
      lastTime_full = currentTime;
    }
  
    // envoyer les données tous les SEND_FREQUENCY_TIC ms
    if (currentTime - lastTime_tic > SEND_FREQUENCY_TIC) {
      //Serial.println(F("SEND_FREQUENCY_TIC"));
      flag_tic = true;
      lastTime_tic = currentTime;
    }
  }

}