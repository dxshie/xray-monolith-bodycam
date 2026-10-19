#include "pch_script.h"
#include "bodycam_camera.h"
#include "bodycam_settings.h"
#include "Actor.h"
#include "ai_space.h"
#include "script_engine.h"

using namespace luabind;

static bool bodycam_request_motion_mode(int mode)
{
	return Bodycam::RequestMotionMode(mode);
}

static int bodycam_get_motion_mode()
{
	return static_cast<int>(Bodycam::GetMotionSelection().Requested());
}

static int bodycam_get_active_motion_mode()
{
	return static_cast<int>(Bodycam::GetMotionSelection().Active());
}

static Fvector bodycam_get_reference_weapon_position(LPCSTR section, bool ads)
{
	const auto& value = Bodycam::GetReferenceWeaponPosition(section);
	return ads ? value.ads : value.hip;
}
static Fvector bodycam_get_reference_weapon_position_baseline(LPCSTR section, bool ads)
{
	const auto& value = Bodycam::GetReferenceWeaponPositionBaseline(section);
	return ads ? value.ads : value.hip;
}
static bool bodycam_set_reference_weapon_position(LPCSTR section, bool ads, float x, float y, float z)
{
	Fvector value;
	value.set(x, y, z);
	return Bodycam::SetReferenceWeaponPosition(section, ads, value);
}

// Logs each unknown setting name once.
static void bodycam_report_unknown_setting(LPCSTR name)
{
	static xr_set<shared_str> reported;
	if (reported.insert(name).second)
		Msg("! bodycam: unknown setting '%s', using the script's fallback", name);
}

static float bodycam_get_float(LPCSTR name, float fallback)
{
	float value = fallback;
	if (!Bodycam::GetFloat(name, value))
		bodycam_report_unknown_setting(name);
	return value;
}

static bool bodycam_get_bool(LPCSTR name, bool fallback)
{
	bool value = fallback;
	if (!Bodycam::GetBool(name, value))
		bodycam_report_unknown_setting(name);
	return value;
}

static void bodycam_set_viewmodel_profile(float pos_x, float pos_y, float pos_z, float pitch, float yaw, float roll, float blend_speed)
{
	CActor* actor = Bodycam::CurrentActor();
	if (!actor)
		return;

	Fvector pos;
	pos.set(pos_x, pos_y, pos_z);
	Fvector rot;
	rot.set(yaw, pitch, roll);
	actor->cam_BodycamSetViewmodelProfile(pos, rot, blend_speed);
}

static void bodycam_clear_viewmodel_profile(float blend_speed)
{
	if (CActor* actor = Bodycam::CurrentActor())
		actor->cam_BodycamClearViewmodelProfile(blend_speed);
}

static object bodycam_get_bindings()
{
	lua_State* L = ai().script_engine().lua();
	object bindings = newtable(L);
	int index = 1;

	u32 count = 0;
	const Bodycam::FloatBinding* floats = Bodycam::GetFloatBindings(count);
	for (u32 i = 0; i < count; ++i)
	{
		const Bodycam::FloatBinding& binding = floats[i];
		object entry = newtable(L);
		entry["name"] = binding.name;
		entry["console_name"] = binding.console_name;
		entry["type"] = "float";
		entry["default"] = binding.default_value;
		entry["min"] = binding.min_value;
		entry["max"] = binding.max_value;
		bindings[index++] = entry;
	}

	const Bodycam::BoolBinding* bools = Bodycam::GetBoolBindings(count);
	for (u32 i = 0; i < count; ++i)
	{
		const Bodycam::BoolBinding& binding = bools[i];
		object entry = newtable(L);
		entry["name"] = binding.name;
		entry["console_name"] = binding.console_name;
		entry["type"] = "bool";
		entry["default"] = binding.default_value;
		bindings[index++] = entry;
	}

	return bindings;
}

// Bodycam: drive the dead-body redaction post-process. Builds a
// LIST of head rects (one per visible corpse) into the shared xrEngine globals
// that CRenderTarget::phase_redaction() (xrRenderPC_R4) reads each frame. Rects are
// normalised [0,1] screen space (top-left origin). Usage from Lua each frame:
//   bodycam.redaction_begin()
//   bodycam.redaction_add(x0,y0,x1,y1)   -- once per visible corpse head box
//   bodycam.redaction_commit(intensity)  -- publishes the list (0 intensity clears)
// The begin/commit split makes the render thread see a complete list, never a
// half-filled one. Inert without this exe: the render pass ignores an empty list.
#define BODYCAM_REDACTION_MAX 16   // keep in sync with xr_ioc_cmd.cpp and bodycam_redaction.ps

extern ENGINE_API float g_bodycam_redaction_rects[BODYCAM_REDACTION_MAX * 4];
extern ENGINE_API int   g_bodycam_redaction_count;
extern ENGINE_API float g_bodycam_redaction_intensity;
extern ENGINE_API int   g_bodycam_redaction_mode;
extern ENGINE_API int   g_bodycam_redaction_active;
extern ENGINE_API int   g_bodycam_redaction_debug;

// Effect variant applied inside every head box: 0 redaction, 1 pixelate, 2 black.
// Separate from commit so older mod builds that don't call it just leave mode 0
// (redaction) -- graceful degradation.
static void bodycam_redaction_set_mode(int mode)
{
	g_bodycam_redaction_mode = mode;
}

extern ENGINE_API float g_bodycam_redaction_depths[BODYCAM_REDACTION_MAX];
extern ENGINE_API float g_bodycam_redaction_world[BODYCAM_REDACTION_MAX * 6];

static int   s_bodycam_redaction_pending = 0;
static float s_bodycam_redaction_pending_rects[BODYCAM_REDACTION_MAX * 4];
static float s_bodycam_redaction_pending_depths[BODYCAM_REDACTION_MAX];
static float s_bodycam_redaction_pending_world[BODYCAM_REDACTION_MAX * 6];

static void bodycam_redaction_begin()
{
	s_bodycam_redaction_pending = 0;
}

// (x0,y0,x1,y1) = the box in MAIN-camera normalised screen space (for the main
// pass); depth = its view-space depth in metres (the shader skips pixels nearer
// than it, masking the viewmodel/foreground). (wcx,wcy,wcz) + (whw,whh) = the box
// WORLD centre + world half-width/height in metres, which the SVP pass reprojects
// through the scope camera (the main-camera rect can't be). world args default to
// 0 (no in-scope draw) for the back-compat wrapper.
static void bodycam_redaction_add(float x0, float y0, float x1, float y1, float depth,
	float wcx, float wcy, float wcz, float whw, float whh)
{
	if (s_bodycam_redaction_pending >= BODYCAM_REDACTION_MAX) return;
	if (!(x1 > x0 && y1 > y0)) return;   // skip degenerate rects
	float* r = &s_bodycam_redaction_pending_rects[s_bodycam_redaction_pending * 4];
	r[0] = x0; r[1] = y0; r[2] = x1; r[3] = y1;
	s_bodycam_redaction_pending_depths[s_bodycam_redaction_pending] = depth;
	float* wpad = &s_bodycam_redaction_pending_world[s_bodycam_redaction_pending * 6];
	wpad[0] = wcx; wpad[1] = wcy; wpad[2] = wcz; wpad[3] = whw; wpad[4] = whh; wpad[5] = 0.f;
	++s_bodycam_redaction_pending;
}

static void bodycam_redaction_commit(float intensity)
{
	const int was_active = g_bodycam_redaction_active;
	for (int i = 0; i < s_bodycam_redaction_pending * 4; ++i)
		g_bodycam_redaction_rects[i] = s_bodycam_redaction_pending_rects[i];
	for (int i = 0; i < s_bodycam_redaction_pending; ++i)
		g_bodycam_redaction_depths[i] = s_bodycam_redaction_pending_depths[i];
	for (int i = 0; i < s_bodycam_redaction_pending * 6; ++i)
		g_bodycam_redaction_world[i] = s_bodycam_redaction_pending_world[i];
	g_bodycam_redaction_count = s_bodycam_redaction_pending;
	g_bodycam_redaction_intensity = intensity;
	g_bodycam_redaction_active = (s_bodycam_redaction_pending > 0 && intensity > 0.f) ? 1 : 0;
	if (g_bodycam_redaction_debug && g_bodycam_redaction_active && !was_active)
		Msg("[bodycam] bodycam.redaction ACTIVE count=%d intensity=%.2f", g_bodycam_redaction_count, intensity);
}

// Back-compat single-rect wrappers (older mod builds call these). depth 0 = no
// test, world 0 = no in-scope draw.
static void bodycam_set_redaction_rect(float x0, float y0, float x1, float y1, float intensity)
{
	bodycam_redaction_begin();
	bodycam_redaction_add(x0, y0, x1, y1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	bodycam_redaction_commit(intensity);
}

static void bodycam_clear_redaction()
{
	bodycam_redaction_begin();
	bodycam_redaction_commit(0.f);
}

// Corpse silhouette redaction (stencil mask). mode: -1 off, 0 pixelate, 1 black.
// The engine draws the effect where the redaction stencil bit is set (tagged in
// the g-buffer), so it hugs the corpse silhouette. intensity 0 also disables.
extern ENGINE_API int   g_bodycam_redact_corpse_mode;
extern ENGINE_API float g_bodycam_redact_corpse_intensity;
static void bodycam_redact_corpse(int mode, float intensity)
{
	g_bodycam_redact_corpse_mode = (intensity > 0.f) ? mode : -1;
	g_bodycam_redact_corpse_intensity = intensity;
}

#pragma optimize("s", on)
void Bodycam::script_register(lua_State* L)
{
	module(L, "bodycam")[def("request_motion_mode", &bodycam_request_motion_mode), def("get_motion_mode", &bodycam_get_motion_mode),
		def("get_active_motion_mode", &bodycam_get_active_motion_mode),
		def("get_reference_weapon_position", &bodycam_get_reference_weapon_position),
		def("get_reference_weapon_position_baseline", &bodycam_get_reference_weapon_position_baseline),
		def("has_reference_weapon_position_baseline", &Bodycam::HasReferenceWeaponPositionBaseline),
		def("restore_reference_weapon_position", &Bodycam::RestoreReferenceWeaponPosition),
		def("set_reference_weapon_position", &bodycam_set_reference_weapon_position),
		def("save_reference_weapon_position", &Bodycam::SaveReferenceWeaponPosition), def("get_float", &bodycam_get_float),
		def("set_float", &Bodycam::SetFloat), def("get_bool", &bodycam_get_bool), def("set_bool", &Bodycam::SetBool),
		def("apply_preset", &Bodycam::ApplyPreset), def("set_viewmodel_profile", &bodycam_set_viewmodel_profile),
		def("clear_viewmodel_profile", &bodycam_clear_viewmodel_profile), def("dump", &Bodycam::DumpState),
		def("get_bindings", &bodycam_get_bindings),
		def("set_redaction_rect", &bodycam_set_redaction_rect), def("clear_redaction", &bodycam_clear_redaction),
		def("redaction_begin", &bodycam_redaction_begin), def("redaction_add", &bodycam_redaction_add),
		def("redaction_commit", &bodycam_redaction_commit), def("redaction_set_mode", &bodycam_redaction_set_mode),
		def("redact_corpse", &bodycam_redact_corpse)];
}
