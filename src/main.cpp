// Avtor: Uroš Slejko
// program: Mehatronika
// leto projekta: 2021
//

//Test Git

// Preko vmesnika na osebnem računalniku pošiljamo po serijski povezavi naše vnose na
// tipkovnico, v lastno izdelanem formatu. Mikrokrmilnik te podatke pretvori v
// številke od 0 do 11. Te številke nam služijo kot kazalec v tabeli. Tam so vpisani
// ukazi za 4 koračne motorje v bajtu. In sicer po dva bita za en motor. Ko nam
// kazalec pokaže na mesto tabele za ukaz naprej morajo vsi motorji potiskati naprej.
// Med mikrokrmilnikom in motorji je še "Arduino CNC ščit", na katerem so A4988
// koračni gonilniki. Mikrokrmilnik pošilja tem gonilnikom podate o smeri vrtenja
// ter s pulzi določa hitrost vrtenja motorja.
//
// Prikaz kako so motorji postavljeni z njihovimi imeni v programu:
//
//  naprej
//   |^|
//  M0  M1         S3D6    S12D13
//
//  M2  M3         S2D5    S4D7

// Z uporabo #include, v program vnesemo knjižnico. Tako bomo lahko kasneje v programu
// uporabljali funkcije iz knjižnic, ki smo jih vnesli
#include <Arduino.h> // Vsebuje funkcije ki so potrebne za programiranje v Arduino okolju
#include <AccelStepper.h> //Vebuje funkcije za lažje programiranje koračnih motorjev

// Z uporabo #define povemo prevajalniku naj prvo vrednost zamenja s drugo programu.
// Tako pridobimo uporabo "spremenljivk", brez da bi mikrokrmilniku porabljali spomin.
#define M0_stp 3
#define M0_dir 6
#define M1_stp 12
#define M1_dir 13
#define M2_stp 2
#define M2_dir 5
#define M3_stp 4
#define M3_dir 7
#define EN 8

#define motorSpeed 22000           // Privzeta hitrost motorja v delovanju v pulz/s
#define M_default_speed 50000       // Privzeta maksimalna hitrost motorja v pulz/s
#define M_default_acceleration 1000 // Privzeto pospeševanje motorja v pulz/s/s

// Definiranje spremenljiv, ki so vporabljenje kasneje v programu 
int8_t commands[3] = {0, 0, 0}, currDirectionn;
float speedFactor = 0.02f, speedIncrease = 0.01f;


// Tabela z ukazi za motorje v vseh željenih smereh
// 10 = motor se vrti naprejw
// 01 = motor se ne vrti
// 00 = motor se vrti nazaj
int16_t motorInstructions[11] = {0B01010101,  //stoji
                                 0B10101010,  //naprej
                                 0B00000000,  //nazaj
                                 0B10000010,  //desno
                                 0B00101000,  //levo
                                 0B10010110,  //desno-naprej
                                 0B01000001,  //desno-nazaj
                                 0B01101001,  //levo-naprej
                                 0B01000100,  //levo-nazaj
                                 0B10001000,  //desno-obratS
                                 0B00100010}; //levo-obrat

// Knjižnici za koračne motorje (Accelstepper) je pred uporabo funkciji treba povedati
// na katere izhode so gonilniki povezani v tem primeru za motor 0 povemo, da je
// povezan na gonilnik, ter na katerih digitalnih izhodih je povezan
AccelStepper M0(AccelStepper::DRIVER, M0_stp, M0_dir); 
AccelStepper M1(AccelStepper::DRIVER, M1_stp, M1_dir);
AccelStepper M2(AccelStepper::DRIVER, M2_stp, M2_dir);
AccelStepper M3(AccelStepper::DRIVER, M3_stp, M3_dir);

// Pred uporabo motorjev nastavimo vrednosti na prej definirane vrednosti
void Inicializacija_motorjev()
{
  M0.setAcceleration(M_default_acceleration); 
  M0.setMaxSpeed(M_default_speed);            
  M0.setMinPulseWidth(3);                     

  M1.setAcceleration(M_default_acceleration);
  M1.setMaxSpeed(M_default_speed);
  M1.setMinPulseWidth(3);

  M2.setAcceleration(M_default_acceleration);
  M2.setMaxSpeed(M_default_speed);
  M2.setMinPulseWidth(3);

  M3.setAcceleration(M_default_acceleration);
  M3.setMaxSpeed(M_default_speed);
  M3.setMinPulseWidth(3);

  digitalWrite(EN, HIGH);
}

// dataIn format kakor pomenijo gibanje. podatek 111 pomeni da platforma stoji: 
//  <-{112}{110}->
// {201}{211}{221}
// {101}{111}{121}
// {001}{011}{021}

// Format_data funkcija spremeni črke ki jih dobimo po serijski povezavi prvo v številke
// ter nato spremeni te številke v kazalce, ki so uporabljeni za kazanje na polje v tabeli
void Format_data()
{
  byte dataIn[4]; // lokalno definirana spremenlivka, v katero se shranijo prispeli znaki
  Serial.readBytesUntil('X', dataIn, 4); // beremo serijski sprejem do znaka X in spravimo v dataIn
  for (int n = 0; n < 3; n++) // for zanka, katera sprejete znake spremeni v številke
    commands[n] = dataIn[n] - 48;

  if (commands[0] == 1 && commands[1] == 1 && commands[2] == 1)
  { // primerjanje
    currDirectionn = 0;                                               //stoji
    digitalWrite(EN, HIGH);
    speedFactor = 0.02f;
  }
  else if (commands[0] == 2 && commands[1] == 1 && commands[2] == 1)
  {
    currDirectionn = 1;                                               //naprej 
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 0 && commands[1] == 1 && commands[2] == 1)
  {
    currDirectionn = 2;                                               //nazaj
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 1 && commands[1] == 2 && commands[2] == 1)
  {
    currDirectionn = 3;                                               //desno
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 1 && commands[1] == 0 && commands[2] == 1)
  {
    currDirectionn = 4;                                               //levo
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 2 && commands[1] == 2 && commands[2] == 1)
  {
    currDirectionn = 5;                                               //naprej-desno
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 2 && commands[1] == 0 && commands[2] == 1)
  {
    currDirectionn = 6;                                               //naprej-levo
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 0 && commands[1] == 2 && commands[2] == 1)
  {
    currDirectionn = 7;                                               //nazaj-desno
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 0 && commands[1] == 0 && commands[2] == 1)
  {
    currDirectionn = 8;                                               //nazaj-levo
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 1 && commands[1] == 1 && commands[2] == 2)
  {
    currDirectionn = 9;                                               //vrtenje-desno
    digitalWrite(EN, LOW);
  }
  else if (commands[0] == 1 && commands[1] == 1 && commands[2] == 0)
  {
    currDirectionn = 10;                                              //vrtenje-levo
    digitalWrite(EN, LOW);
  }
  else
  {
    currDirectionn = 0;                 //v primeru drugačne številke platforma stoji
    digitalWrite(EN, HIGH);
    speedFactor = 0.02f;
  }
}

// Funkcija Drive_motors nam združi podatek v katero smer hočemo dase platforma premika 
// ter tabelo v kateri so zapisani smeri motorjev, in s temi podatki krmilimo motorje
void Drive_motors()
{
  M0.setSpeed((int)(((motorInstructions[currDirectionn] & 0b11000000) >> 6) - 1) * (speedFactor * motorSpeed * -1));
  M1.setSpeed((int)(((motorInstructions[currDirectionn] & 0b00110000) >> 4) - 1) * (speedFactor * motorSpeed));
  M2.setSpeed((int)(((motorInstructions[currDirectionn] & 0b00001100) >> 2) - 1) * (speedFactor * motorSpeed * -1));
  M3.setSpeed((int)(((motorInstructions[currDirectionn] & 0b00000011) >> 0) - 1) * (speedFactor * motorSpeed));
}

void SpeedUp ()
{
  int lastTime;
  if(speedFactor <= 1)
    if (millis() >= lastTime + 100)
    {
      speedFactor = speedFactor + speedIncrease;
      lastTime = millis();
    }
}

// Koda v setup funkciji se izvede samo enkrat pred začetkom programa
void setup()
{
  Serial.begin(115200); // Pričnemo serijsko komunikacijo

  pinMode(EN, OUTPUT); // Pin "enable" definiramo kot izhod mikrokrmilnika

  Inicializacija_motorjev(); // Nekatere vrednosti motorjev postavimo na privzete vrednosti
}

// Funkcija loop() se nenehno izvazja znova dokler je mikrormilnik prižgan
void loop()
{
  if (Serial.available()) // Če so podatki v serijskem zalogovniku se izvede koda
  {
    Format_data(); // Funkcija za pretvarjanje vhodnih podatkov
    Drive_motors(); // Funkcija, ki motorjem poda smer ter hitrost
    Serial.print('R');
  }
  SpeedUp ();
  M0.runSpeed(); // Funkcija, ki če je preteklo dovolj časa od zadnjega pulza pošlje naslednjega
  M1.runSpeed();
  M2.runSpeed();
  M3.runSpeed();
}