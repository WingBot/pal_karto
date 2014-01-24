/**************************************************************************
**
**  Class:  file: PalKartoClient.h
**
**  Author: Luca Marchionni (luca)
**  Email : luca.marchionni@pal-robotics.com
**  Created on: 23-5-2012
**
**  Copyright (c) 2010 PAL Robotics sl. All Rights Reserved
**************************************************************************/

#ifndef PALKARTOCLIENT_H
#define PALKARTOCLIENT_H

#include <auto_ptr.h>

namespace ros
{
  class ServiceClient;
}
namespace pal
{
  namespace slam
  {
    class PalKartoClient
    {
      public:
        PalKartoClient();
        virtual ~PalKartoClient();

        bool start();

        bool stop();

      protected:
        std::auto_ptr<ros::ServiceClient> _startClient;
        std::auto_ptr<ros::ServiceClient> _stopClient;
    };
  }
}

#endif // PALKARTOCLIENT_H
