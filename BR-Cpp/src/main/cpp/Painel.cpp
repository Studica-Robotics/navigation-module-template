#include "Painel.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

PAINEL::PAINEL() {}

// ----- Botões ----- //

bool PAINEL::Start()
{
    return B_Start.Get();
}

bool PAINEL::Stop()
{
    return B_Stop.Get();
}

bool PAINEL::Reset()
{
    return B_Reset.Get();
}

bool PAINEL::Emergencia()
{
    return B_Emergencia.Get();
}

// ----- Leds ----- //

void PAINEL::LigarVerde()
{
    Led_Verde.Set(1);
}

void PAINEL::DesligarVerde()
{
    Led_Verde.Set(0);
}

void PAINEL::LigarVermelho()
{
    Led_Vermelho.Set(1);
}

void PAINEL::DesligarVermelho()
{
    Led_Vermelho.Set(0);
}