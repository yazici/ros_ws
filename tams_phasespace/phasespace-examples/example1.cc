// example1.cc -*- C++ -*-
// simple point tracking program

#include <iostream>
#include <cstring>

#include "owl.hpp"

using namespace std;

int main1()
{
  OWL::Context owl;
  OWL::Markers markers;

  if(owl.open("localhost") <= 0 || owl.initialize() <= 0) return 0;

  owl.frequency(480.0);
  owl.streaming(true);

  // main loop
  while(owl.isOpen() && owl.property<int>("initialized"))
    {
      const OWL::Event *e = owl.nextEvent(1000);
      if(!e) continue;

      if(e->type_id() == OWL::Type::ERROR)
        {
          string s;
          if(e->get(s)) cerr << e->name() << ": " << s << endl;
          break;
        }
      else if(e->type_id() == OWL::Type::FRAME)
        {
          cout << "time=" << e->time() << " " << e->type_name() << " " << e->name() << "=" << e->size<OWL::Event>() << ":" << endl;
          if(e->get(markers) > 0)
            {
              cout << " " << e->type_name() << " " << e->name() << "=" << markers.size() << ":" << endl;
              for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                if(m->cond > 0)
                  cout << "  " << m->id << ") " << m->x << " " << m->y << " " << m->z << endl;
            }
        }
    } // while

  owl.done();
  owl.close();

  return 0;
}
