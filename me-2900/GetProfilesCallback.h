#ifndef TRANSFERMULTIPLEPROFILESEXAMPLE_H
#define TRANSFERMULTIPLEPROFILESEXAMPLE_H

#include <libllt.h>
#include <iostream>
#include <vector>

#define MAX_INTERFACE_COUNT 	5
#define MAX_RESOLUTION 		6

void ControlLostCallback(ArvGvDevice *gv_device, gpointer ptr);

int GetProfilesCallback(void);
void DisplayProfiles(double * , double * , guint32);
void DisplayTimestamp (guchar * );

void NewProfile (const void * , size_t , gpointer);

LLT * m_pLLT;
guint32 m_uiResolution;
std::vector<unsigned char> m_uiProfileBuffer;
TScannerType m_tscanCONTROLType;
guint32 m_ProfileCount, m_NeededProfileCount;
guint32 m_uiProfileDataSize;


#endif // TRANSFERMULTIPLEPROFILESEXAMPLE_H
