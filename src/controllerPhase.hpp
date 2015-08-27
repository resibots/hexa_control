#ifndef CONTROLLERPHASE_HPP
#define CONTROLLERPAHSE_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <math.h>





class ControllerPhase
{

protected :

  std::vector< std::vector<float> > _legsParams;
  std::vector<int> _brokenLegs;
private:
  float delayedPhase(float t, float phi);

public :




  bool isBroken(int leg)
  {
    for (int j=0;j<_brokenLegs.size();j++)
    {
      if (leg==_brokenLegs[j])
      {
        return true;
      }
    }
    return false;
  }


  ControllerPhase(const std::vector<float>& ctrl,std::vector<int> brokenLegs):_brokenLegs(brokenLegs)
  {
    assert(ctrl.size()==24);
    for (int leg=0;leg<6;leg++)
    {
      std::vector<float> param;

      param.push_back(0); //pinit mot 0
      param.push_back(ctrl[leg*4]); //amp mot 0
      param.push_back(ctrl[leg*4+1]); //phase mot 0
      param.push_back(0); //Pinit mot 1 & 2
      param.push_back(ctrl[leg*4+2]); //amp Mot 1 & 2
      param.push_back(ctrl[leg*4+3]); //phase mot 1
      param.push_back(ctrl[leg*4+3]); // phase mot 2

      _legsParams.push_back(param);
    }


  }


  std::vector<int> get_pos_dyna(float t);
  std::vector<int> get_speeds_dyna( );
  std::vector<bool> get_directions_dyna( );


};

#endif
