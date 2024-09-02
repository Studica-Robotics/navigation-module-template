#include "MovementControl.hpp"

void MovementControl::initMove()
{
    drivetrain.startLidar();
    drivetrain.resetNavX();
}

void MovementControl::guardarAngulo(bool& deveGuardarAngulo, double& anguloArmazenado)
{
    if(deveGuardarAngulo)
    {
        double angulo;
        drivetrain.getNavX(angulo);

        anguloArmazenado = angulo;
        deveGuardarAngulo = false;
    }
}

void MovementControl::lerDistanciaFiltrada(int angle, int& distance) 
{
    drivetrain.getLidarDistancia(angle, distance);
}

void MovementControl::evitarParedes()
{
    int distance0, distance1, distance2;

    lerDistanciaFiltrada(216, distance0);
    lerDistanciaFiltrada(250, distance1);
    lerDistanciaFiltrada(337, distance2);

    if(distance0 < 150 || distance1 < 150 || distance2 < 150)
    {
        drivetrain.re();
    }
}

void MovementControl::detectarCanto()
{
    int distance0, distance1, distance2;

    lerDistanciaFiltrada(270, distance0);
    lerDistanciaFiltrada(250, distance1);
    lerDistanciaFiltrada(300, distance2);

    double distanciaCentro, distanciaCentro1, distanciaCentro2;
    drivetrain.getDistanceCenter(270, distance0, distanciaCentro);
    drivetrain.getDistanceCenter(250, distance1, distanciaCentro1);
    drivetrain.getDistanceCenter(300, distance2, distanciaCentro2);

    double esquerdoMenor = distanciaCentro - distanciaCentro1;
    double direitoMenor = distanciaCentro - distanciaCentro2;

    canto = (esquerdoMenor >= 50) && (direitoMenor >= 50);
}

void MovementControl::moveByLidar()
{
    int distance0, distance1, distance2;

    lerDistanciaFiltrada(270, distance0);
    lerDistanciaFiltrada(250, distance1);
    lerDistanciaFiltrada(300, distance2);

    double distanciaCentro, distanciaCentro1, distanciaCentro2;
    drivetrain.getDistanceCenter(270, distance0, distanciaCentro);
    drivetrain.getDistanceCenter(250, distance1, distanciaCentro1);
    drivetrain.getDistanceCenter(300, distance2, distanciaCentro2);

    double rangeMIN = distanciaCentro - 40;
    double rangeMAX = distanciaCentro + 40;

    bool paredeFRONTAL = (distanciaCentro1 >= rangeMIN && distanciaCentro1 <= rangeMAX) && 
                         (distanciaCentro2 >= rangeMIN && distanciaCentro2 <= rangeMAX);

    bool frenteLivre = (distanciaCentro > 200 && distanciaCentro1 > 250 && distanciaCentro2 > 250);

    double difEsquerdoDireito = distanciaCentro1 - distanciaCentro2;
    double difDireitoEsquerdo = distanciaCentro2 - distanciaCentro1;

    evitarParedes();
    detectarCanto();

    if(frenteLivre)
    {
        bool esquerda = false, direita = false, frente = false;

        for (int i = 218; i < 328; i++)
        {
            lerDistanciaFiltrada(i, distance0);

            if(i > 270 && distance0 < 300) // Esquerda
            {
                esquerda = true;
                break;
            }
            else if(i < 260 && distance0 < 300) // Direita
            {
                direita = true;
                break; 
            }
            else
            {
                frente = true;
            } 
        }

        if(esquerda)
        {
            drivetrain.girarAntiHorario();
        }
        else if(direita)
        {
            drivetrain.girarHorario();
        }
        else
        {
            drivetrain.frente();
            guardar = true;
        }
    }
    else if(difDireitoEsquerdo > 40 && !frenteLivre)
    {
        drivetrain.girarHorario();
        guardar = true;
    }
    else if(difEsquerdoDireito > 40 && !frenteLivre)
    {
        drivetrain.girarAntiHorario();
        guardar = true;
    }
    else if(canto)
    {
        int d90Canto = 0; 
        bool entrouCanto = false;
        guardarAngulo(guardar, anguloArmazenado);

        while(true)
        {
            evitarParedes();
            drivetrain.volta90(anguloArmazenado, posicaoAtingida);

            if(posicaoAtingida)
            {
                lerDistanciaFiltrada(250, distance1);
                d90Canto = distance1;
                detectarCanto(); // atualiza a variável canto
            }
            if(canto)
            {
                if(!entrouCanto)
                {
                    guardarAngulo(guardar, anguloArmazenado);
                    entrouCanto = true;
                }

                drivetrain.volta90(anguloArmazenado, posicaoAtingida);

                if(posicaoAtingida)
                {
                    break;
                }
            }
            else if(d90Canto > 0)
            {
                break;
            }
        }
    }
    else if(paredeFRONTAL && !frenteLivre && difDireitoEsquerdo < 40 && difEsquerdoDireito < 40)
    {   
        if(guardar)
        {
            guardarAngulo(guardar, anguloArmazenado);
        }

        bool varerEsquerda = true;
        bool varerDireita = false;

        guardarAngulo(guardar, anguloArmazenado);
        while(true)
        {
            evitarParedes();
            if(varerEsquerda)
            {
                drivetrain.volta45(anguloArmazenado, posicaoAtingida);
                if(posicaoAtingida)
                {
                    lerDistanciaFiltrada(250, distance1);
                    d45 = distance1;
                    varerEsquerda = false;
                    varerDireita = true;
                }
            }
            else if(varerDireita)
            {
                drivetrain.volta90n(anguloArmazenado, posicaoAtingida);
                if(posicaoAtingida)
                {
                    lerDistanciaFiltrada(250, distance1);
                    d45n = distance1;
                    varerEsquerda = false;
                    varerDireita = false;
                }
            }
            else
            {
                if(d45 < 500 && d45n < 500) 
                {
                    guardarAngulo(guardar, anguloArmazenado);
                    while(true)
                    {
                        evitarParedes();
                        drivetrain.volta135n(anguloArmazenado, posicaoAtingida);
                        if(posicaoAtingida)
                        {
                            break;
                        }
                    } 
                    break;
                }
                else 
                {
                    if(d45 > d45n)
                    {
                        while(true)
                        {
                            evitarParedes();
                            drivetrain.volta90(anguloArmazenado, posicaoAtingida);
                            if(posicaoAtingida)
                            {
                                break;
                            }
                        }
                        break;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }
    else
    {
        varer45_2 = true;

        if(guardar)
        {
            guardarAngulo(guardar, anguloArmazenado);
        }
    }
}