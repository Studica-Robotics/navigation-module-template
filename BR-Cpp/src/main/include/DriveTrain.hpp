#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include <math.h>
#include "studica/TitanQuad.h"
#include "AHRS.h"
#include "studica/Lidar.h"
#include <frc/Joystick.h>

class DriveTrain
{
 public:

   DriveTrain();

   void setM0(double speed);
   void setM1(double speed);
   void setM2(double speed);
   void setM3(double speed);

   void stopMotores();
   
   void frente();
   void re();
   void girarHorario();
   void girarAntiHorario();
   void meiaVolta();
   void volta45(double anguloAntigo, bool& posicaoAtingida);
   void volta45n(double anguloAntigo, bool& posicaoAtingida);
   void volta90(double anguloAntigo, bool& posicaoAtingida);
   void volta90n(double anguloAntigo, bool& posicaoAtingida);
   void volta135(double anguloAntigo, bool& posicaoAtingida);
   void volta135n(double anguloAntigo, bool& posicaoAtingida);

   void rotateTo(double position, bool& satisfiedPosition);
   void goTo0();
   void goTo90();
   void goTo180();
   void goTo270();

   void startLidar();
   void stopLidar();
   void getLidarAngulo(int degrees, double& angle);
   void getLidarDistancia(int degrees, int& distance);

   void getDistanceCenter(double angle, int hipotenusa, double& distance);

   void compararDistancia(double range, double distance, bool& result);

   void resetNavX();
   void getNavX(double& angle);

 private:

   studica::TitanQuad M0{42, 0};
   studica::TitanQuad M1{42, 1};
   studica::TitanQuad M2{42, 2};
   studica::TitanQuad M3{42, 3};

   AHRS navX {frc::SPI::Port::kMXP};

   studica::Lidar lidar{studica::Lidar::Port::kUSB1};
   studica::Lidar::ScanData scanData;
};

#endif