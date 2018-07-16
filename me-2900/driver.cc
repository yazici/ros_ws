#include "GetProfilesCallback.h"
#include <sstream>
#include <string>
#include "entities.pb.h"
#include <nats/nats.h>

// Global variables for debugging only
gboolean firstTrans = true;
guint32 profileCounter = 0, oldProfileCounter = 0, lostProfiles = 0;
double sc = 0, so = 0;
// Connection to Nats
natsConnection *nc = NULL;
natsSubscription *sub = NULL;
natsMsg *msg = NULL;
natsStatus s = NATS_OK;
const char *nats_channel_env = NULL;

// Event handle
EHANDLE event;

using namespace std;

int main()
{

    // Get NATS_HOST from Environment Variable
    const char *nats_host_env = std::getenv("NATS_HOST");
    if (nats_host_env == NULL)
    {
        std::cout << "NATS_HOST not set" << std::endl;
        nats_host_env = NATS_DEFAULT_URL;
    }
    std::cout << "NATS_HOST=" << nats_host_env << std::endl;

    s = natsConnection_ConnectTo(&nc, nats_host_env);
    if (s != NATS_OK)
    {
        std::cout << "nats connection failed ";
        return 1;
    }
    // Get NATS_CHANNEL from Environment Variable
    nats_channel_env = std::getenv("NATS_CHANNEL");
    if (nats_channel_env == NULL)
    {
        std::cout << "NATS_CHANNEL not set" << std::endl;
        nats_channel_env = "foo";
    }
    std::cout << "NATS_CHANNEL=" << nats_channel_env << std::endl;

    // Get IDLE_TIME from Environment Variable
    const char *IdleTime_env = std::getenv("IDLE_TIME");
    if (IdleTime_env == NULL)
    {
        std::cout << "IDLE_TIME not set" << std::endl;
        IdleTime_env = "500";
    }
    std::cout << "IDLE_TIME=" << IdleTime_env << std::endl;

    // Get SHUTTER_TIME from Environment Variable
    const char *ShutterTime_env = std::getenv("SHUTTER_TIME");
    if (ShutterTime_env == NULL)
    {
        std::cout << "SHUTTER_TIME not set" << std::endl;
        ShutterTime_env = "200";
    }
    std::cout << "SHUTTER_TIME=" << ShutterTime_env << std::endl;

    const char *resolution_env = std::getenv("RESOLUTION");
    if (resolution_env == NULL)
    {
        std::cout << "RESOLUTION not set" << std::endl;
        resolution_env = "3";
    }
    std::cout << "RESOLUTION=" << resolution_env << std::endl;

    int iRetValue = 0;
    m_NeededProfileCount = 1;

    CreateEvent(&event);

    std::vector<char *> vcInterfaces(MAX_INTERFACE_COUNT);
    std::vector<guint32> vuiResolutions(MAX_RESOLUTION);
    guint32 uiInterfaceCount = 0;

    guint32 uiIdleTime = atoi(IdleTime_env); // Convert const char* to int
    guint32 uiShutterTime = atoi(ShutterTime_env);

    iRetValue = GetDeviceInterfaces(&vcInterfaces[0], vcInterfaces.size());
    if (iRetValue == ERROR_GETDEVINTERFACE_REQUEST_COUNT)
    {
        cout << "There are more than " << vcInterfaces.size() << " scanCONTROL connected \n";
        uiInterfaceCount = vcInterfaces.size();
    }
    else if (iRetValue < 1)
    {
        cout << "A error occured during searching for connected scanCONTROL \n";
        uiInterfaceCount = 0;
    }
    else
        uiInterfaceCount = iRetValue;

    if (uiInterfaceCount == 0)
        cout << "no interface found" << endl;

    for (guint32 i = 0; i < uiInterfaceCount; i++)
    {
        cout << "Interface: " << vcInterfaces[i] << "\n";
    }

    /* Make an instance for each LLT */
    m_pLLT = new LLT();

    if ((iRetValue = SetPathtoDeviceProperties("./device_properties.dat")) < GENERAL_FUNCTION_OK)
    {
        cout << "Error setting device ID path\nExit program...\n";
        exit(0);
    }

    if ((m_pLLT->SetDeviceInterface(vcInterfaces[0])) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while setting dev id " << iRetValue << "!\n";
        exit(0);
    }

    /* Connect to sensor */
    if ((iRetValue = m_pLLT->Connect()) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
        exit(0);
    }

    if ((iRetValue = m_pLLT->GetLLTType(&m_tscanCONTROLType)) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while GetLLTType!\n";
        exit(0);
    }

    if (iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
    {
        cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library.\n";
    }

    if (m_tscanCONTROLType == scanCONTROL27xx_xxx)
    {
        cout << "The scanCONTROL is a scanCONTROL27xx\n";
    }
    else if (m_tscanCONTROLType == scanCONTROL26xx_xxx)
    {
        cout << "The scanCONTROL is a scanCONTROL26xx\n";
    }
    else if (m_tscanCONTROLType == scanCONTROL29xx_xxx)
    {
        cout << "The scanCONTROL is a scanCONTROL29xx\n";
    }
    else
    {
        cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n";
    }

    // cout << "Get all possible resolutions\n";
    if ((iRetValue = m_pLLT->GetResolutions(&vuiResolutions[0], vuiResolutions.size())) < GENERAL_FUNCTION_OK)
    {
        cout << "Error GetResolutions!\n";
        exit(0);
    }
    std::cout << vuiResolutions[atoi(resolution_env)] << std::endl;
    m_uiResolution = vuiResolutions[atoi(resolution_env)];

    if (m_pLLT->SetResolution(m_uiResolution) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while setting transmission mode!\n";
        exit(0);
    }

    if (m_pLLT->SetProfileConfig(PROFILE) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while setting SetProfileConfig!\n";
        exit(0);
    }

    if (m_pLLT->SetFeature(FEATURE_FUNCTION_IDLETIME, uiIdleTime) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while setting FEATURE_FUNCTION_IDLETIME!\n";
        exit(0);
    }

    if (m_pLLT->SetFeature(FEATURE_FUNCTION_SHUTTERTIME, uiShutterTime) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while setting FEATURE_FUNCTION_SHUTTERTIME!\n";
        exit(0);
    }

    if (m_pLLT->SetFeature(FEATURE_FUNCTION_TRIGGER, 0x00000000) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while setting FEATURE_FUNCTION_TRIGGER!\n";
        exit(0);
    }

    cout << "Register callbacks\n";

    // Example user data
    int llt_id = 1;

    /* Register Callbacks for program handling */
    if ((m_pLLT->RegisterBufferCallback((gpointer)&NewProfile, &llt_id)) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while registering buffer callback!\n";
        exit(0);
    }

    if ((m_pLLT->RegisterControlLostCallback((gpointer)&ControlLostCallback, NULL)) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while registering control lost callback!\n";
        exit(0);
    }

    if (GetProfilesCallback() < GENERAL_FUNCTION_OK)
    {
        cout << "Error in GetProfiles_Callback!\n";
        exit(0);
    }

    /* Disconnect to sensor */
    cout << "Disconnect\n";
    if ((iRetValue = m_pLLT->Disconnect()) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
        exit(0);
    }

    FreeEvent(&event);

    delete m_pLLT;
}

int GetProfilesCallback()
{
    int iRetValue = 0;
    m_ProfileCount = 0;

    std::vector<double> ValueX, ValueZ;

    cout << "Show getting profiles via Callback\n";

    m_uiProfileBuffer.resize(m_uiResolution * 64);

    ValueX.resize(m_uiResolution);
    ValueZ.resize(m_uiResolution);

    // Setup transfer of multiple profiles
    if ((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, true)) < GENERAL_FUNCTION_OK)
    {
        // cout << "Error in profile transfer!\n";
        return iRetValue;
    }

    cout << "Start aquisition of profiles\n";

    while (true)
    {
        m_ProfileCount = 0;

        if (WaitForSingleObject(&event, 2) != 0)
        {
            printf("Timeout!\n");
            exit(0);
        }

        ResetEvent(&event);

        /* Display example points */
        iRetValue = ConvertProfile2Values(&m_uiProfileBuffer[0], m_uiProfileBuffer.size(), &m_pLLT->appData, m_uiResolution, 0, NULL, NULL, NULL, &ValueX[0], &ValueZ[0], NULL, NULL);
        if (iRetValue == (CONVERT_X | CONVERT_Z))
        {
            // cout << "X/Z extracted\n";
        }

        DisplayProfiles(&ValueX[0], &ValueZ[0], m_uiResolution);
    }

    /* Stop transfer */
    if ((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, false)) < GENERAL_FUNCTION_OK)
    {
        cout << "Error while stopping transmission!\n";
        return iRetValue;
    }

    return GENERAL_FUNCTION_OK;
}

void NewProfile(const void *pucData, size_t uiSize, gpointer user_data)
{

    if (pucData != NULL && uiSize == m_uiProfileBuffer.size() && *(int *)user_data == 1 && m_ProfileCount < m_NeededProfileCount)
    {
        m_uiProfileDataSize = uiSize;
        memcpy(&m_uiProfileBuffer[0], pucData, uiSize);
        m_ProfileCount++;
    }
    if (m_ProfileCount >= m_NeededProfileCount)
    {
        SetEvent(&event);
    }
}

void ControlLostCallback(ArvGvDevice *mydevice, gpointer user_data)
{
    /* Control of the device is lost. Display a message and force application exit */
    if (mydevice)
    {
        cout << "Control lost\n";
        exit(0);
    }
}

void DisplayProfiles(double *adValueX, double *adValueZ, guint32 uiResolution)
{
    // Neue Pointcloud
    entities::PointCloud pc;

    for (guint32 i = 1; i < uiResolution; i++)
    {
        entities::Point3d *point = pc.add_points();
        point->set_x(adValueX[i]);
        point->set_y(0);
        point->set_z(adValueZ[i]);
    }

    // Serializing zu byte-Array um diesen per Nats zu versenden:
    size_t size = pc.ByteSizeLong();
    void *data = malloc(size);
    pc.SerializeToArray(data, size);

    // Versenden
    natsConnection_Publish(nc, nats_channel_env, (const void *)data, size);

    // PointCloud leeren
    pc.Clear();
}

void DisplayTimestamp(guchar *uiTimestamp)
{
    double dShutterOpen, dShutterClose;
    guint32 uiProfileCount;

    Timestamp2TimeAndCount(&uiTimestamp[0], &dShutterOpen, &dShutterClose, &uiProfileCount);

    cout.precision(8);
    // cout << "Profile Count: " << uiProfileCount << " ShutterOpen: " << dShutterOpen << " ShutterClose: " << dShutterClose << "\n";
    cout.precision(6);
}
