/* Xash3D/CS 1.6 Gyroscope Feature BY PowerSiderS
   Modification Fixes BY NaruTo 1.6 */

#include "hud.h"
#include "usercmd.h"
#include "cvardef.h"

#ifdef __ANDROID__
#include <android/sensor.h>
#include <android/looper.h>
#endif

#include "gyroscope.h"

cvar_t *gyroscope;
cvar_t *gyroscope_sensitivity;
cvar_t *gyroscope_invert_x;
cvar_t *gyroscope_invert_y;

#ifdef __ANDROID__

static ASensorManager *g_pSensorManager = NULL;
static ASensorEventQueue *g_pSensorEventQueue = NULL;
static const ASensor *g_pGyroSensor = NULL;
static ALooper *g_pLooper = NULL;

static float g_flGyroYaw = 0.0f;
static float g_flGyroPitch = 0.0f;
static int64_t g_iLastTimestamp = 0;
static bool g_bGyroInitialized = false;
static bool g_bGyroSensorEnabled = false;
static bool g_bFirstEvent = true;

static float g_flSmoothYaw = 0.0f;
static float g_flSmoothPitch = 0.0f;

static const float NS2S = 1.0f / 1000000000.0f;

// =========================
// Helper Functions
// =========================

static float ApplyDeadzone(float v, float dz)
{
	if (fabsf(v) < dz)
		return 0.0f;

	float sign = (v > 0) ? 1.0f : -1.0f;
	float norm = (fabsf(v) - dz) / (1.0f - dz);
	return sign * norm;
}

static float ApplyCurve(float v)
{
	float sign = (v > 0) ? 1.0f : -1.0f;
	return sign * powf(fabsf(v), 1.6f); // PUBG-like curve
}


static void Gyro_EnableSensor(void)
{
	if (!g_bGyroInitialized || !g_pSensorEventQueue || !g_pGyroSensor)
		return;

	if (!g_bGyroSensorEnabled)
	{
		ASensorEventQueue_enableSensor(g_pSensorEventQueue, g_pGyroSensor);
		ASensorEventQueue_setEventRate(g_pSensorEventQueue, g_pGyroSensor, 16666);

		g_bGyroSensorEnabled = true;
		g_bFirstEvent = true;
		g_flGyroYaw = 0.0f;
		g_flGyroPitch = 0.0f;
	}
}

static void Gyro_DisableSensor(void)
{
	if (!g_bGyroInitialized || !g_pSensorEventQueue || !g_pGyroSensor)
		return;

	if (g_bGyroSensorEnabled)
	{
		ASensorEventQueue_disableSensor(g_pSensorEventQueue, g_pGyroSensor);
		g_bGyroSensorEnabled = false;
		g_flGyroYaw = 0.0f;
		g_flGyroPitch = 0.0f;
	}
}

static int Gyro_SensorCallback(int fd, int events, void *data)
{
	ASensorEvent event;

	while (ASensorEventQueue_getEvents(g_pSensorEventQueue, &event, 1) > 0)
	{
		if (event.type != ASENSOR_TYPE_GYROSCOPE)
			continue;

		if (g_bFirstEvent)
		{
			g_iLastTimestamp = event.timestamp;
			g_bFirstEvent = false;
			continue;
		}

		float dT = (float)(event.timestamp - g_iLastTimestamp) * NS2S;
		g_iLastTimestamp = event.timestamp;

		if (dT <= 0.0f || dT > 0.5f)
			continue;

		float gyroX = event.data[0];
		float gyroY = event.data[1];

		float rawYaw   = -gyroX;
		float rawPitch = -gyroY;

		// Invert controls
		if (gyroscope_invert_x && gyroscope_invert_x->value != 0.0f)
			rawYaw = -rawYaw;

		if (gyroscope_invert_y && gyroscope_invert_y->value != 0.0f)
			rawPitch = -rawPitch;

		rawYaw   = ApplyDeadzone(rawYaw, 0.015f);
		rawPitch = ApplyDeadzone(rawPitch, 0.015f);

		rawYaw   = ApplyCurve(rawYaw);
		rawPitch = ApplyCurve(rawPitch);

		float sens = (gyroscope_sensitivity && gyroscope_sensitivity->value > 0.0f)
		             ? gyroscope_sensitivity->value : 1.0f;

		float base = 120.0f;

		float scaledYaw   = rawYaw   * dT * base * sens;
		float scaledPitch = rawPitch * dT * base * sens;

		float alpha = 0.18f;

		g_flSmoothYaw   = g_flSmoothYaw   * (1.0f - alpha) + scaledYaw   * alpha;
		g_flSmoothPitch = g_flSmoothPitch * (1.0f - alpha) + scaledPitch * alpha;


		g_flGyroYaw   += g_flSmoothYaw;
		g_flGyroPitch += g_flSmoothPitch;
	}

	return 1;
}

void Gyro_Init(void)
{
	gyroscope = gEngfuncs.pfnRegisterVariable("gyroscope", "0", FCVAR_ARCHIVE);
	gyroscope_sensitivity = gEngfuncs.pfnRegisterVariable("gyroscope_sensitivity", "1.8", FCVAR_ARCHIVE);

	gyroscope_invert_x = gEngfuncs.pfnRegisterVariable("gyroscope_invert_x", "0", FCVAR_ARCHIVE);
	gyroscope_invert_y = gEngfuncs.pfnRegisterVariable("gyroscope_invert_y", "0", FCVAR_ARCHIVE);

	g_pSensorManager = ASensorManager_getInstance();
	if (!g_pSensorManager)
	{
		gEngfuncs.Con_Printf("Gyroscope: Failed to get sensor manager\n");
		return;
	}

	g_pGyroSensor = ASensorManager_getDefaultSensor(g_pSensorManager, ASENSOR_TYPE_GYROSCOPE);
	if (!g_pGyroSensor)
	{
		gEngfuncs.Con_Printf("Gyroscope: Gyroscope not available\n");
		return;
	}

	g_pLooper = ALooper_forThread();
	if (!g_pLooper)
		g_pLooper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

	g_pSensorEventQueue = ASensorManager_createEventQueue(
		g_pSensorManager,
		g_pLooper,
		ALOOPER_POLL_CALLBACK,
		Gyro_SensorCallback,
		NULL
	);

	g_bGyroInitialized = true;

	gEngfuncs.Con_Printf("Gyroscope: PUBG-style initialized\n");
}

void Gyro_Shutdown(void)
{
	Gyro_DisableSensor();

	if (g_pSensorManager && g_pSensorEventQueue)
		ASensorManager_destroyEventQueue(g_pSensorManager, g_pSensorEventQueue);

	g_bGyroInitialized = false;
}


void Gyro_Update(float *yaw, float *pitch)
{
	if (!g_bGyroInitialized)
		return;

	bool enable = (gyroscope && gyroscope->value != 0.0f);

	if (enable && !g_bGyroSensorEnabled)
		Gyro_EnableSensor();
	else if (!enable && g_bGyroSensorEnabled)
		Gyro_DisableSensor();

	if (!g_bGyroSensorEnabled)
	{
		if (yaw) *yaw = 0.0f;
		if (pitch) *pitch = 0.0f;
		return;
	}

	if (g_pLooper)
		ALooper_pollOnce(0, NULL, NULL, NULL);

	if (yaw)   *yaw = g_flGyroYaw;
	if (pitch) *pitch = g_flGyroPitch;

	g_flGyroYaw = 0.0f;
	g_flGyroPitch = 0.0f;
}

void Gyro_Reset(void)
{
	g_flGyroYaw = 0.0f;
	g_flGyroPitch = 0.0f;
	g_flSmoothYaw = 0.0f;
	g_flSmoothPitch = 0.0f;
	g_bFirstEvent = true;
	g_iLastTimestamp = 0;
}

int Gyro_IsEnabled(void)
{
	if (!g_bGyroInitialized || !gyroscope)
		return 0;

	return (gyroscope->value != 0.0f);
}

#else

void Gyro_Init(void)
{
	gyroscope = gEngfuncs.pfnRegisterVariable("gyroscope", "0", FCVAR_ARCHIVE);
	gyroscope_sensitivity = gEngfuncs.pfnRegisterVariable("gyroscope_sensitivity", "1.0", FCVAR_ARCHIVE);
}

void Gyro_Shutdown(void) {}
void Gyro_Update(float *yaw, float *pitch)
{
	if (yaw) *yaw = 0.0f;
	if (pitch) *pitch = 0.0f;
}
void Gyro_Reset(void) {}
int Gyro_IsEnabled(void) { return 0; }

#endif
