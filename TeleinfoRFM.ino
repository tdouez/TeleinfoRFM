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
// Trame : $origine;libelle;donnees   (libellé & donnnées cryptés)
//--------------------------------------------------------------------
// 2021/01/15 - FB V1.0.0
// 2021/06/11 - FB V1.0.1
// 2021/07/01 - FB V1.0.2 - Add ETIQU_URMS1..3
// 2021/11/21 - FB V1.0.3 - Bug fix  modes triphasé et producteur
// 2021/12/08 - FB V2.0.0 - Utilisation de la librairie libTeleinfo (https://github.com/hallard/LibTeleinfo) et chg protocole vers GW
// 2021/12/27 - FB V2.0.1 - Fix bug aléatoire, blocage envoi data
//--------------------------------------------------------------------

#define TI_DEBUG

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <jled.h>
#include <LibTeleinfo.h>

#define VERSION   "v2.0.1"

#define MODE_TIC  TINFO_MODE_STANDARD    // TINFO_MODE_HISTORIQUE ou TINFO_MODE_STANDARD

#define ENTETE  "$"

#define CRYPT_PASS "FumeeBleue"

#define POWER_PIN         4
#define TELEINFO_LED_PIN  3   // Teleinfo LED
#define CHARGE_LED_PIN    5   // Charge LED

//-------------------------------------------------------------------------------------
// Seuil : 4,5v à l'entrée du régulateur 3,3v > 4,5v*100k/(100k+68k)=2,68v sur A0
//                                                 3,3v->1023 => 2,68->830
//-------------------------------------------------------------------------------------
#define SEUIL_CHARGE_FULL   825
#define SEUIL_CHARGE_98     810

#define RFM_TX_POWER   10   // 5..23 dB

#define LG_TRAME_MAX  125
#define LG_TRAME_MIN  5
#define DEB_TRAME 0x0A
#define FIN_TRAME 0x0D


boolean full_tic = true;
boolean flag_send_tic = true;
boolean flag_adresse_tic;
boolean flag_boot;
boolean flag_tic_histo;

unsigned long SEND_FREQUENCY_FULL = 600000; // 10mn, Minimum time between send (in milliseconds). 
unsigned long SEND_FREQUENCY_TIC  =  15000; // 15s,  Minimum time between send (in milliseconds). 
unsigned long lastTime_full = 0;
unsigned long lastTime_tic = 0;
unsigned long lastTime_timeout_tic = 0;
char buffin[LG_TRAME_MAX];
int  index_buff = 0;
char adresse_compteur[16];

auto led_chg = JLed(CHARGE_LED_PIN);
TInfo tinfo;


// ---------------------------------------------------------------- 
// change_etat_led_teleinfo
// ---------------------------------------------------------------- 
void change_etat_led_teleinfo()
{
  static int led_state;

  led_state = !led_state;
  digitalWrite(TELEINFO_LED_PIN, led_state);

}

// ---------------------------------------------------------------- 
// LoRa_sendMessage
//    Envoi message LORA 
// ---------------------------------------------------------------- 
void LoRa_sendMessage(char *message) {
  
  LoRa.idle();
  LoRa.beginPacket();    // start packet
  LoRa.print(message);   // add payload
  LoRa.endPacket();      // finish packet and send it
  LoRa.sleep();

}

// ---------------------------------------------------------------- 
// send_boot
//    Envoi info au démarrge avec version
// ---------------------------------------------------------------- 
void send_boot() 
{
char trame[150];
char mode[5];
 
  Serial.println(F(">> Send BOOT"));

  trame[0] = 0;
  strcat(trame, ENTETE);
  strcat(trame, "Linky_");
  strcat(trame, adresse_compteur);
  strcat(trame, ";VERSION;");
  strcat(trame, VERSION); 
  strcat(trame, ";");
  Serial.println(trame);
  verif_charge_condo(SEUIL_CHARGE_98);
  LoRa_sendMessage(trame);

  Serial.println(F(">> fin Send BOOT"));
}

// ---------------------------------------------------------------- 
// search_adress_teleinfo
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
boolean search_adress_teleinfo(ValueList *vl_tic)
{

  if (vl_tic) {
    // parcours liste chainée vl_tic
    while (vl_tic->next) {
      vl_tic = vl_tic->next;

      if (vl_tic->name && strlen(vl_tic->name) && vl_tic->value && strlen(vl_tic->value)) {
        change_etat_led_teleinfo();
        if (strstr(vl_tic->name, "ADCO") || strstr(vl_tic->name, "ADSC")) {
          strncpy(adresse_compteur, vl_tic->value, 12);
          Serial.print(F("Adresse compteur: "));
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
// send_teleinfo
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
boolean send_teleinfo(ValueList *vl_tic, boolean all_tic)
{
char trame[150];
boolean flag_SINSTS = false;

  
  if (vl_tic) {
    // parcours liste chainée vl_tic
    while (vl_tic->next) {
      vl_tic = vl_tic->next;
      
      // uniquement sur les nouvelles valeurs ou celles modifiées ou toutes
      if ( all_tic || ( vl_tic->flags & (TINFO_FLAGS_UPDATED | TINFO_FLAGS_ADDED) )) {
     
        if (vl_tic->name && strlen(vl_tic->name) > 2 && vl_tic->value && strlen(vl_tic->value) > 1) {
          change_etat_led_teleinfo();
          // entete trame --------
          trame[0] = 0;
          strcat(trame, ENTETE);
          strcat(trame, "Linky_");
          strcat(trame, adresse_compteur);
          strcat(trame, ";");

          // données ---------------
          Serial.print(vl_tic->name);
          Serial.print(F(":"));
          Serial.println(vl_tic->value);
          
          strcat(trame, vl_tic->name);
          strcat(trame, ";");
          strncat(trame, vl_tic->value, 32);  // limite à 32 caractères par valeur
          strcat(trame, ";");
          
          Serial.println(trame);

          if (flag_send_tic) {
            verif_charge_condo(SEUIL_CHARGE_98);
            LoRa_sendMessage(trame);   
          } 
        } 
      }
    }
    flag_send_tic = false;
  }
  digitalWrite(TELEINFO_LED_PIN, LOW);
}

// ---------------------------------------------------------------- 
// charge_condo
//    charge le super condo avec gestion led 
// ---------------------------------------------------------------- 
bool charge_condo(int ref_seuil)
{
boolean flag_cap = false;

  Serial.println(F(">> Charge Condo.."));
  led_chg.Breathe(1000).DelayAfter(500).Forever();

  while (flag_cap == false) {
    if (analogRead(A0) >= ref_seuil) flag_cap = true;
    led_chg.Update();
  }
  led_chg.Off();
  
}

// ---------------------------------------------------------------- 
// test_charge_condo
//    test niveau tension sur le super condo. 
//    retourne vrai si niveau suffisant
// ---------------------------------------------------------------- 
boolean test_charge_condo(int ref_seuil)
{
boolean rc = true;
int seuil;

  seuil = analogRead(A0);
  if (seuil < ref_seuil) {
    rc=false;
  }
  return rc;
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

/* ======================================================================
Function: NewFrame 
Purpose : callback when we received a complete teleinfo frame
Input   : linked list pointer on the concerned data
Output  : - 
Comments: -
====================================================================== */
void NewFrame(ValueList *vl_tic)
{

  if (!flag_adresse_tic) search_adress_teleinfo(vl_tic);
    else send_teleinfo(vl_tic, true);
    
  full_tic = false;
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

  if (!flag_adresse_tic) search_adress_teleinfo(vl_tic);
    else send_teleinfo(vl_tic, full_tic);
    
  full_tic = false;
}

// ---------------------------------------------------------------- 
// Calcul checksum  -----------------------------------------------
// ---------------------------------------------------------------- 
char ckecksum(char *buff, int len)
{
int i;
char sum = 0;

    for (i=0; i<len; i++) sum = sum + buff[i];
    sum = (sum & 0x3F) + 0x20;

    //Serial.print("CalCS=");
    //Serial.println(sum, HEX);
    return(sum);
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
  led_chg.Off();

  // init interface série suivant mode TIC
  Serial.flush();
  Serial.end();
 
  Serial.begin(MODE_TIC == TINFO_MODE_HISTORIQUE ? 1200 : 9600);

  Serial.println(F("   __|              _/           _ )  |"));
  Serial.println(F("   _| |  |   ` \\    -_)   -_)    _ \\  |   -_)  |  |   -_)"));
  Serial.println(F("  _| \\_,_| _|_|_| \\___| \\___|   ___/ _| \\___| \\_,_| \\___|"));
  Serial.print(F("                                             "));
  Serial.println(VERSION);

  // première charge super condo
  verif_charge_condo(SEUIL_CHARGE_FULL);

  // activation alim et init RFM
  digitalWrite(POWER_PIN, HIGH);
  Serial.print(F("Init RF95: "));
  while (!LoRa.begin(868E6)) {
    Serial.println(F("Starting LoRa failed!"));
    charge_condo(SEUIL_CHARGE_FULL);
  }
  LoRa.enableCrc();
  LoRa.setTxPower(RFM_TX_POWER);
  LoRa.sleep();
  Serial.println(F("OK."));

  flag_boot = true;
  flag_adresse_tic = false;
  
  // init interface TIC
  tinfo.init(TINFO_MODE_STANDARD);
  tinfo.attachNewFrame(NewFrame);
  tinfo.attachUpdatedFrame(UpdatedFrame);

  Serial.println(F("Fin setup"));
}


// ---------------------------------------------------------------- 
// loop
// ---------------------------------------------------------------- 
void loop()
{
  uint32_t currentTime = millis();

  if (Serial.available()) tinfo.process(Serial.read() & 127);

  if (flag_adresse_tic) {
  
    if (flag_boot) {
      // envoi version au démarrage
      send_boot();
      flag_boot = false;
    }
    
    // envoyer l'ensemble des données tous les SEND_FREQUENCY_FULL ms
    if (currentTime - lastTime_full > SEND_FREQUENCY_FULL) {
      Serial.println(F("SEND_FREQUENCY_FULL"));
      full_tic = true;
      flag_send_tic = true;
      lastTime_full = currentTime;
    }
  
    // envoyer les données tous les SEND_FREQUENCY_TIC ms
    if (currentTime - lastTime_tic > SEND_FREQUENCY_TIC) {
      Serial.println(F("SEND_FREQUENCY_TIC"));
      flag_send_tic = true;
      lastTime_tic = currentTime;
    }
  }
  led_chg.Update();
}
