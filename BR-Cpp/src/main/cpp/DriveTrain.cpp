#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include "DriveTrain.hpp"

DriveTrain::DriveTrain() {}

void DriveTrain::setM0(double speed)
{
    M0.Set(speed);
}

void DriveTrain::setM1(double speed)
{
    M1.Set(speed);
}

void DriveTrain::setM2(double speed)
{
    M2.Set(speed);
}

void DriveTrain::setM3(double speed)
{
    M3.Set(speed);
}

void DriveTrain::stopMotores()
{
    double speed = 0.0;
    setM0(speed);
    setM1(speed);
    setM2(speed);
    setM3(speed);
}

double anguloArmazenado;
double angulo;
void DriveTrain::frente()
{
    bool guardarAngulo = true;
    frc::SmartDashboard::PutNumber("Guardar Angulo", 0);
    if(guardarAngulo)
    {
        angulo;
        getNavX(angulo);

        anguloArmazenado = angulo;
        guardarAngulo = false;
        frc::SmartDashboard::PutBoolean("Guardar dentro", 1);
    }

    frc::SmartDashboard::PutNumber("anguloArmazenado", guardarAngulo);

    double anguloAtual;
    getNavX(anguloAtual);

    double dif = fmod(((anguloArmazenado - anguloAtual) * -1) + 360, 360.0);

    if(dif < 2 || dif > 358)
    {
        double speed = 0.5;
        setM0(-speed);
        setM1(-speed);
        setM2(speed);
        setM3(speed);
    }
    else if(dif > 2 || dif < 358)
    {
        if(dif < 180)
        {
            girarHorario();
        }
        else if(dif > 180)
        {
            girarAntiHorario();
        }
    }
}

void DriveTrain::re()
{
    double speed = 0.3;
    setM0(speed);
    setM1(speed);
    setM2(-speed);
    setM3(-speed);
}

void DriveTrain::girarHorario()
{
    double speed = 0.5;
    setM0(speed);
    setM1(speed);
    setM2(speed);
    setM3(speed);
}

void DriveTrain::girarAntiHorario()
{
    double speed = 0.5;
    setM0(-speed);
    setM1(-speed);
    setM2(-speed);
    setM3(-speed);
}

void DriveTrain::rotateTo(double posicao, bool& posicaoAtingida)
{
    double angulo;
    getNavX(angulo);
    double dif = fmod(((angulo - posicao) * -1) + 360, 360.0);

    if(dif < 4 || dif == 0 || dif > 356)
    {
        stopMotores();
        posicaoAtingida = true;
    }
    else if(dif < 180 || dif == 180)
    {
        girarAntiHorario();
        posicaoAtingida = false;
    }
    else if(dif > 180)
    {
        girarHorario();
        posicaoAtingida = false;
    }
}

void DriveTrain::volta45(double anguloAntigo, bool& posicaoAtingida)
{
    rotateTo(anguloAntigo + 45, posicaoAtingida);
    // frc::SmartDashboard::PutBoolean("Posição Atingida", posicaoAtingida);
}

void DriveTrain::volta90(double anguloAntigo, bool& posicaoAtingida)
{
    rotateTo(anguloAntigo + 90, posicaoAtingida);
    // frc::SmartDashboard::PutBoolean("Posição Atingida", posicaoAtingida);
}

void DriveTrain::volta90n(double anguloAntigo, bool& posicaoAtingida)
{
    rotateTo(anguloAntigo + (360 - 90), posicaoAtingida);
    // frc::SmartDashboard::PutBoolean("Posição Atingida", posicaoAtingida);
}

void DriveTrain::volta135(double anguloAntigo, bool& posicaoAtingida)
{
    rotateTo(anguloAntigo + 135, posicaoAtingida);
    // frc::SmartDashboard::PutBoolean("Posição Atingida", posicaoAtingida);
}

void DriveTrain::volta135n(double anguloAntigo, bool& posicaoAtingida)
{
    rotateTo(anguloAntigo + (360 - 135), posicaoAtingida);
    // frc::SmartDashboard::PutBoolean("Posição Atingida", posicaoAtingida);
}



void DriveTrain::volta45n(double anguloAntigo, bool& posicaoAtingida)
{
    rotateTo(anguloAntigo + (360 - 45), posicaoAtingida);
    // frc::SmartDashboard::PutBoolean("Posição Atingida", posicaoAtingida);
}


void DriveTrain::goTo0()
{
    bool posicaoAtingida;
    rotateTo(0.0, posicaoAtingida);
}

void DriveTrain::goTo90()
{
    bool posicaoAtingida;
    rotateTo(90.0, posicaoAtingida);
}

void DriveTrain::goTo180()
{
    bool posicaoAtingida;
    rotateTo(180.0, posicaoAtingida);
}

void DriveTrain::goTo270()
{
    bool posicaoAtingida;
    rotateTo(270.0, posicaoAtingida);
}

void DriveTrain::startLidar()
{
    lidar.Start();
}

void DriveTrain::stopLidar()
{
    lidar.Stop();
}

void DriveTrain::getLidarAngulo(int degrees, double& angle)
{
    scanData = lidar.GetData();
    angle = scanData.angle[degrees];

    // frc::SmartDashboard::PutNumber("LidarAngulo", angle);
}

void DriveTrain::getLidarDistancia(int degrees, int& distance)
{
    scanData = lidar.GetData();
    distance = scanData.distance[degrees];

    if(degrees == 270)
    {
        frc::SmartDashboard::PutNumber("LidarDistancia 270", distance);
    }
    else
    {
         frc::SmartDashboard::PutNumber("LidarDistancia", distance);
    }
    
}

void DriveTrain::getDistanceCenter(double angle, int hipotenusa, double& distance)
{
    double anguloRad = (270 - angle) * (M_PI / 180.0); // ((angulo + (270 - angle))) * (M_PI / 180.0);

    distance = hipotenusa * (cos(anguloRad));

}


void DriveTrain::compararDistancia(double range, double distance, bool& resultado)
{
    double diferenca = distance - range;
    resultado = diferenca <= 10;
}

void DriveTrain::resetNavX()
{
    navX.ZeroYaw();
}

void DriveTrain::getNavX(double& angle)
{
    angle = fmod(( navX.GetAngle() * -1) + 360, 360.0);
    frc::SmartDashboard::PutNumber("navX Angulo", angle);
}

