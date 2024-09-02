#ifndef MOVEMENT_CONTROL_HPP
#define MOVEMENT_CONTROL_HPP

#include "DriveTrain.hpp"

class MovementControl
{
 public:
    void initMove();
    void guardarAngulo(bool& guardarAngulo, double& anguloArmazenado);
    void lerDistanciaFiltrada(int angle, int& distance);
    void evitarParedes();
    void detectarCanto();
    void moveByLidar();

 private:
    DriveTrain drivetrain;
    bool canto = false;
    bool guardar = false;
    double anguloArmazenado = 0;
    bool posicaoAtingida = false;
    double d45 = 0;
    double d45n = 0;
    bool initVarrer = true;
    bool varer45 = true;
    bool varer45_2 = false;
    bool varreu45 = false;
    bool varreu45_2 = false;
    bool varreu45n = false;
    bool varreu45n_2 = false;
};

#endif