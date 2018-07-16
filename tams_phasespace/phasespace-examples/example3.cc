// example3.cc -*- C++ -*-
// simple rigid tracking program

#include <iostream>
#include <cstring>
#include <vector>

#include "owl.hpp"

using namespace std;

////

inline void printInfo(const OWL::Markers &markers)
{
  for(OWL::Markers::const_iterator m = markers.begin(); m != markers.end(); m++)
    {
      if(m->cond <= 0) continue;
      cout << "   " << m->id << ") pos=" << m->x << " " << m->y << " " << m->z << endl;
    }
}

inline void printInfo(const OWL::Rigids &rigids)
{
  for(OWL::Rigids::const_iterator r = rigids.begin(); r != rigids.end(); r++)
    {
      if(r->cond <= 0) continue;
      cout << "   " << r->id << ") pos=" << r->pose[0] << " " << r->pose[1] << " " << r->pose[2];
      cout << " rot=" << r->pose[3] << " " << r->pose[4] << " " << r->pose[5] << " " << r->pose[6];
      cout << endl;
    }
}

////

int main()
{
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Rigids rigids;

  if(owl.open("localhost") <= 0 || owl.initialize() <= 0) return 0;
  
  // create the rigid tracker
  uint32_t trackerID = 0;
  vector<OWL::TrackerInfo> tinfo;
  tinfo.push_back(OWL::TrackerInfo(trackerID, "rigid", "myrigid", ""));
  owl.createTrackers(&*tinfo.begin(), &*tinfo.end());

  // assign markers to the rigid and specify local coordinates in millimeters (taken from wand.json)
  vector<OWL::MarkerInfo> minfo;
  minfo.push_back(OWL::MarkerInfo(4, trackerID, "4", "pos=0,920,-7.615"));
  minfo.push_back(OWL::MarkerInfo(5, trackerID, "5", "pos=-7.615,795,0"));
  minfo.push_back(OWL::MarkerInfo(6, trackerID, "6", "pos=0,670,-7.615"));
  minfo.push_back(OWL::MarkerInfo(7, trackerID, "7", "pos=-7.615,545,0"));
  owl.assignMarkers(&*minfo.begin(), &*minfo.end());
  
  // start streaming
  owl.frequency(480.0);
  owl.streaming(true);
  
  // main loop
  while(owl.isOpen() && owl.property<int>("initialized"))
    {
      const OWL::Event *event = owl.nextEvent(1000);
      if(!event) continue;

      switch(event->type_id())
        {
        case OWL::Type::ERROR:
          {
            string s;
            if(event->get(s)) cerr << event->name() << ": " << s << endl;
          }
          break;
        case OWL::Type::FRAME:
          {
            cout << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << endl;
            for(const OWL::Event *e = event->begin(); e != event->end(); e++)
              {
                switch(e->type_id())
                  {
                  case OWL::Type::MARKER:
                    {
                      if(e->get(markers) > 0)
                        {
                          cout << " " << e->type_name() << " " << e->name() << "=" << markers.size() << ":" << endl;
                          printInfo(markers);
                        }
                    }
                    break;
                  case OWL::Type::RIGID:
                    {
                      if(e->get(rigids) > 0)
                        {
                          cout << " " << e->type_name() << " " << e->name() << "=" << rigids.size() << ":" << endl;
                          printInfo(rigids);
                        }
                    }
                    break;
                  } // switch
              }
          }
          break;
        } // switch
    } // while

  owl.done();
  owl.close();

  return 0;
}
