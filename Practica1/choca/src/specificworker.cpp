/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <math.h>   


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    state=State::IDLE;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    
    innerModel = new InnerModel("/home/svenminds/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
    
    
  try
  {
      
      RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); 
      RoboCompDifferentialRobot::TBaseState bState;
      differentialrobot_proxy->getBaseState(bState);
      innerModel->updateTransformValues("base",bState.x,0,bState.z,0,bState.alpha,0);
      QVec ini;
        
      
      switch(state)
      {
          
           case State::IDLE:
               if (pick.isActive())
                   state = State::GOTO;
               break;
           case State::GOTO:
               gotoTarget(ldata);
               break;
           case State::BUG:
               bug();
               break;
          
    }
      
}
catch (const Ice::Exception &exc)
{
    std::cout << exc << std::endl;
}
}




void SpecificWorker::gotoTarget(const TLaserData &tLaser)
{
    if(obstacle(tLaser) == true)   // If ther is an obstacle ahead, then transit to BUG
    {
        state = State::BUG;
        return;
        
    }
    QVec rt = innerModel->transform("base", pick.getAux(), "world");
    float dist = rt.norm();
    float ang  = atan2(rt.x(), rt.z());
    if(dist < 100)          // If close to obstacle stop and transit to IDLE
    {
        state = State::IDLE;
        pick.setActive(true);
        return;
        
    }
    float adv = dist;
    if ( fabs(rot) > 0.05 )
        adv = 0;
    
}
void SpecificWorker::bug()

{

}


/**
 * Metodo para detectar obstaculos
 */

bool SpecificWorker::obstacle(TLaserData tLaser)
{
    const int minDist = 400;
    std::sort ( tLaser.begin() + 30, tLaser.end()- 30, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ){	return a.dist < b.dist;});
    return ( tLaser[30].dist < minDist );
}

bool SpecificWorker::targetAtSight(TLaserData lasercopy)
{
    QPolygonF polygon;
    for (auto l: lasercopy)
    {
        QVec lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
        polygon << QPointF(lr.x(), lr.z());
        
    }
    QVec t = pick.getAux();
    return  polygon.contains( QPointF(t.x(), t.z() ) );
    
}



/***/


float SpecificWorker::setGauss(float vADV, float vROT,float h)
{
    
    float lambda = -(pow(vADV, 2.0))/log(h);
    return exp(-pow(vROT, 2.0)/lambda);
    
}

float SpecificWorker::setSignmoide(float distancia)
{
    
    return 1/(1+exp(-distancia))-0.5;
    
}


void SpecificWorker::stopRobot()
{
    
    differentialrobot_proxy->stopBase();

}
void SpecificWorker::setPick(const Pick &mypick)
{
    
    qDebug() << "New target selected: " << mypick.x << mypick.z;
    pick.copy ( mypick.x,mypick.z );
    pick.setActive ( true );
}
  








