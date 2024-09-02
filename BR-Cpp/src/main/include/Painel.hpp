#ifndef PAINEL_HPP
#define PAINEL_HPP

#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"

class PAINEL
{
 public:

   PAINEL();

   // ----- Botões ----- //
   bool Start();
   bool Stop();
   bool Reset();
   bool Emergencia();

   // ----- Leds ----- //
   void LigarVerde();
   void DesligarVerde();
   void LigarVermelho();
   void DesligarVermelho();

 private:

   frc::DigitalInput B_Start {11}; // Modified for actual robot {old: 9} - JT
   frc::DigitalInput B_Reset {8}; // Not wired! - JT
   frc::DigitalInput B_Emergencia {10};
   frc::DigitalInput B_Stop {9}; // Modified to prevent error {old: 11} also not wired - JT

   frc::DigitalOutput Led_Verde {13}; // Modified for actual robot {old: 20} - JT
   frc::DigitalOutput Led_Vermelho {14}; // Modified for actual robot {old: 21} - JT
};




#endif //PAINEL_HPP