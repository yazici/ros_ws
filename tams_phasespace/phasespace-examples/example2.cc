// example2.cc -*- C++ -*-
// simple point tracking program

#include <iostream>
#include <cstring>

#include "owl.hpp"

using namespace std;

int main()
{
  OWL::Context owl;
  OWL::Markers markers;

  if(owl.open("localhost") <= 0 || owl.initialize() <= 0) return 0;

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
                          for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                            if(m->cond > 0)
                              cout << "  " << m->id << ") " << m->x << " " << m->y << " " << m->z << endl;
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
