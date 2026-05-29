// =============================================================================
// KinematicRunPlugin2.cpp
// BipedalSim: Kinematic Runner — VERSION 2
//
// Implements the 10-DOF Normalized Gait Phase State Machine described in
// Instructions.txt.  Instead of raw sine waves dragging joints around blindly,
// every joint follows an explicitly modelled biomechanical phase schedule
// mapped on a per-side normalised phase phi in [0.0, 1.0).
//
// Architecture:
//   phi in [0.0, 0.5)  → Stance Phase  (foot on ground, pushing forward)
//   phi in [0.5, 1.0)  → Swing  Phase  (foot in air, recovering)
//
// Coordinate convention (from live N8RO rig testing in Char Animation1):
//   Z-axis  = forward / backward pitch  (all hip/knee/ankle swing)
//   X-axis  = outward roll / abduction  (shoulder abduction, elbow hinge)
//   Degrees = N8RO expects degrees, NOT radians
// =============================================================================

#include "KinematicRunPlugin2.h"
#include "MathUtils.h"
#include <model/IModel.h>
#include <model/AnimationModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/PluginContext.h>
#include <plugin/IPluginServices.h>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>

namespace arkheon::kinematic2 {
namespace {

// =============================================================================
// Helper: clamp a value to [lo, hi]
// =============================================================================
static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// =============================================================================
// Helper: smooth-step easing for natural acceleration / deceleration
// =============================================================================
static inline double smoothstep(double edge0, double edge1, double x) {
    double t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// =============================================================================
// Helper: linear interpolation
// =============================================================================
static inline double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

// =============================================================================
// Keyframe Interpolator
//
// Reads from a table of (phi, value) keyframe pairs sorted by phi in [0, 1).
// The last keyframe wraps around to the first (cyclic).
// Uses smoothstep easing between each pair of keyframes.
// =============================================================================
using KF = std::pair<double, double>;

static double interpKF(double phi,
                       std::initializer_list<KF> kfs)
{
    const int n = static_cast<int>(kfs.size());
    if (n == 0) return 0.0;

    const KF* arr = kfs.begin();

    // Find the last keyframe whose phi <= current phi
    int prev = 0;
    for (int i = 1; i < n; ++i) {
        if (arr[i].first <= phi) prev = i;
    }
    int next = (prev + 1) % n;

    const double phi0 = arr[prev].first;
    const double val0 = arr[prev].second;
    // When wrapping, the next keyframe is treated as being at phi = 1.0
    const double phi1 = (next == 0) ? 1.0 : arr[next].first;
    const double val1 = arr[next].second;

    double span = phi1 - phi0;
    double t    = (span > 1e-9) ? ((phi - phi0) / span) : 0.0;
    t = clamp(t, 0.0, 1.0);

    return lerp(val0, val1, smoothstep(0.0, 1.0, t));
}

// =============================================================================
// LimbAngles — output struct for one side of the body
// =============================================================================
struct LimbAngles {
    double hipZ;       // Hip flexion/extension on Z (degrees, + = forward, - = back)
    double kneeZ;      // Knee flexion on Z (degrees, 0 = straight, - = bent back)
    double ankleZ;     // Ankle flex on Z (degrees, + = heel strike, - = toe push)
    double shoulderZ;  // Shoulder swing on Z (degrees, opposite to hip)
    double elbowX;     // Elbow bend on X-axis (degrees, stays ~85-95)
};

// =============================================================================
// computeLimbAngles
//
// Driven purely by the 8-stage keyframe table planned from real human running.
// phi is the normalised gait phase for this side in [0, 1):
//
//  Stage | phi  | Name
//    1   | 0.00 | Initial Contact  — heel touches ground
//    2   | 0.12 | Loading          — knee absorbs impact
//    3   | 0.25 | Mid-Stance       — body over foot, leg straight
//    4   | 0.40 | Terminal Stance  — heel rising, push begins
//    5   | 0.50 | Toe-Off          — foot leaves ground
//    6   | 0.62 | Initial Swing    — knee folds rapidly upward
//    7   | 0.75 | Mid-Swing        — peak knee bend, max foot clearance
//    8   | 0.88 | Terminal Swing   — leg extends forward for next contact
// =============================================================================
static LimbAngles computeLimbAngles(double phi) {
    LimbAngles a{};

    // -------------------------------------------------------------------------
    // STANCE PHASE: phi in [0.0, 0.5)
    // Foot is on the ground, pushing the body forward.
    // -------------------------------------------------------------------------
    if (phi < 0.5) {
        // Normalise within stance to [0, 1]
        const double s = phi / 0.5;   // 0 = heel-strike, 1 = toe-off

        // Hip: full symmetric sine wave across the complete phi cycle.
        // Using 2π×phi gives: 0° at start, +30° at phi=0.25, 0° at phi=0.5,
        // −30° at phi=0.75, 0° at phi=1.0 — all 6 steps of the desired gait.
        // NOTE: phi (not s) is used so both phases share the same continuous wave.
        a.hipZ = 30.0 * std::sin(2.0 * 3.14159265358979323846 * phi);

        // Knee: slight ~8° bend at mid-stance for shock absorption only
        {
            double bendPeak = -8.0;
            double kneeCurve = std::sin(s * 3.14159265);
            a.kneeZ = bendPeak * kneeCurve;
        }

        // Ankle: toe-off push, reduced range ±10° to match smaller stride
        a.ankleZ = lerp(8.0, -10.0, smoothstep(0.1, 0.9, s));

        // Shoulder (opposite): mirrors hip
        a.shoulderZ = -a.hipZ * (25.0 / 30.0);

        // Elbow: dynamic pump
        a.elbowX = lerp(100.0, 78.0, smoothstep(0.0, 1.0, s));
    }
    // -------------------------------------------------------------------------
    // SWING PHASE: phi in [0.5, 1.0)
    // Leg is in the air, recovering for the next step.
    // -------------------------------------------------------------------------
    else {
        // Normalise within swing to [0, 1]
        const double s = (phi - 0.5) / 0.5;  // 0 = toe-off, 1 = heel-strike

        // Hip: same continuous full sine wave (identical formula to stance phase).
        // The phase structure (knee/ankle) still switches at phi=0.5 as before.
        a.hipZ = 30.0 * std::sin(2.0 * 3.14159265358979323846 * phi);

        // Knee: peak reduced from -110° to -65°.
        // At -110° with hip at +45°, the foot was geometrically above pelvis.
        // At -65° with hip at +30°, the foot stays ~30cm below the knee — natural jog.
        {
            double kneePeak = -65.0;
            double curve;
            if (s < 0.5) {
                curve = smoothstep(0.0, 0.5, s);
            } else {
                curve = 1.0 - smoothstep(0.5, 1.0, s);
            }
            a.kneeZ = kneePeak * curve;
        }

        // Ankle: dorsiflex to clear ground during swing, reduced to ±10°
        a.ankleZ = lerp(10.0, 3.0, smoothstep(0.1, 0.9, s));

        // Shoulder mirrors hip
        a.shoulderZ = -a.hipZ * (25.0 / 30.0);

        // Elbow: dynamic pump
        a.elbowX = lerp(78.0, 100.0, smoothstep(0.0, 1.0, s));
    }

    // -------------------------------------------------------------------------
    // Hard biomechanical clamp — prevents extravagant contortions
    // (Instructions.txt Section 2 — The Boundaries)
    // -------------------------------------------------------------------------
    a.hipZ      = clamp(a.hipZ,      -30.0,  30.0);  // widened to match symmetric ±30° sine
    a.kneeZ     = clamp(a.kneeZ,     -65.0,   0.0);
    a.ankleZ    = clamp(a.ankleZ,    -10.0,  10.0);
    a.shoulderZ = clamp(a.shoulderZ, -25.0,  25.0);
    a.elbowX    = clamp(a.elbowX,     70.0, 110.0);

    return a;
}

// =============================================================================
// KinematicRunModel2
//
// Implements IModel + IAnimationModel.  The evaluate() method:
//   1. Derives the global normalised gait phase Φ from simulation time.
//   2. Computes per-side phases phi_L and phi_R (180° / 0.5 offset).
//   3. Calls computeLimbAngles() for each side.
//   4. Emits joint overrides using the confirmed N8RO axis mapping.
// =============================================================================
class KinematicRunModel2 final
    : public arkheon::astsim::IModel,
      public arkheon::astsim::IAnimationModel
{
public:
    KinematicRunModel2() = default;

    [[nodiscard]] std::string getTypeName() const override {
        return "KinematicRunModel2";
    }

    [[nodiscard]] std::unique_ptr<IModel> clone() const override {
        return std::make_unique<KinematicRunModel2>(*this);
    }

    // No-op: satisfies the post-N8RO-update virtual configure() requirement
    void configure(const arkheon::astlib::JsonValue& /*config*/) override {}

    [[nodiscard]] bool evaluate(
        const arkheon::astsim::AnimationModelInput& input,
        arkheon::astsim::AnimationModelOutput& output) override
    {
        const double t = input.simulationTimeSeconds;

        // -----------------------------------------------------------------------
        // Step 1 – Normalised Gait Phase (Instructions.txt Section 3)
        //   Phi = fmod(t * f, 1.0)   where f = step frequency in steps/second
        //   2.5 steps/second is the default from the blueprint
        // -----------------------------------------------------------------------
        const double STEP_FREQ = 2.5;  // steps per second
        const double Phi = std::fmod(t * STEP_FREQ, 1.0);

        // Per-side phase: right starts at Phi, left is offset by 0.5
        const double phi_R = Phi;
        const double phi_L = std::fmod(Phi + 0.5, 1.0);

        // -----------------------------------------------------------------------
        // Step 2 – Compute biomechanical angles for each side
        // -----------------------------------------------------------------------
        const LimbAngles R = computeLimbAngles(phi_R);
        const LimbAngles L = computeLimbAngles(phi_L);

        // -----------------------------------------------------------------------
        // Step 3 – Emit joint overrides
        //
        // Confirmed N8RO axis mapping (from live rig testing in Char Animation1):
        //   Z = forward/backward pitch  (hip, knee, ankle, shoulder swing)
        //   X = outward roll/abduction  (shoulder abduction, elbow hinge)
        //
        // CRITICAL FIX: Convert all outputs from degrees to radians, as
        // the ASTSIM engine's AnimationJointOverride struct expects radians
        // in its xRad, yRad, zRad fields (contrary to comments in original code).
        // -----------------------------------------------------------------------
        constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
        output.clearExistingJointOverrides = true;
        output.jointOverrides.clear();

        // Shoulders: small X abduction (±10°) to prevent body clipping,
        //            Z = forward/backward swing from state machine
        output.jointOverrides.push_back({"leftShoulder",  -10.0 * DEG_TO_RAD, 0.0, L.shoulderZ * DEG_TO_RAD});
        output.jointOverrides.push_back({"rightShoulder",  10.0 * DEG_TO_RAD, 0.0, R.shoulderZ * DEG_TO_RAD});

        // Elbows: X = hinge (dynamic 70°–110°), Z = 0
        output.jointOverrides.push_back({"leftElbow",  L.elbowX * DEG_TO_RAD, 0.0, 0.0});
        output.jointOverrides.push_back({"rightElbow", R.elbowX * DEG_TO_RAD, 0.0, 0.0});

        // Hips: Z = swing (dynamic)
        output.jointOverrides.push_back({"leftHip",  0.0, 0.0, L.hipZ * DEG_TO_RAD});
        output.jointOverrides.push_back({"rightHip", 0.0, 0.0, R.hipZ * DEG_TO_RAD});

        // Knees: Z = flex (always ≤ 0°, never forward)
        output.jointOverrides.push_back({"leftKnee",  0.0, 0.0, L.kneeZ * DEG_TO_RAD});
        output.jointOverrides.push_back({"rightKnee", 0.0, 0.0, R.kneeZ * DEG_TO_RAD});

        // Ankles: Z = flex (dorsi/plantarflexion)
        output.jointOverrides.push_back({"leftAnkle",  0.0, 0.0, L.ankleZ * DEG_TO_RAD});
        output.jointOverrides.push_back({"rightAnkle", 0.0, 0.0, R.ankleZ * DEG_TO_RAD});

        return !output.jointOverrides.empty();
    }
};

} // anonymous namespace

// =============================================================================
// Plugin lifecycle
// =============================================================================

int KinematicRunPlugin2::getInterfaceVersion() const { return 1; }

arkheon::astlib::PluginMetadata KinematicRunPlugin2::getMetadata() const {
    arkheon::astlib::PluginMetadata m;
    m.setPluginId("kinematic-run-plugin2");
    m.setVersion("2.0.0");
    m.setAuthor("Student");
    return m;
}

void KinematicRunPlugin2::initialize(arkheon::astlib::PluginContext& context) {
    initialized_ = true;

    if (context.services) {
        auto* svc = context.services->getService(
            arkheon::astsim::IModelPluginService::kPluginServiceId);
        modelPluginService_ =
            static_cast<arkheon::astsim::IModelPluginService*>(svc);
    }

    if (modelPluginService_) {
        auto& registry = modelPluginService_->modelFactoryRegistry();
        auto* proto    = registry.getRegisteredPrototype(modelType_);
        auto* anim     = dynamic_cast<arkheon::astsim::IAnimationModel*>(proto);
        if (anim) {
            animationRegistered_ = anim->registerAnimation(
                animationCode_,
                [](const arkheon::astsim::AnimationModelInput& input,
                   arkheon::astsim::AnimationModelOutput& output) -> bool {
                    static KinematicRunModel2 instance;
                    return instance.evaluate(input, output);
                });
        }
    }
}

void KinematicRunPlugin2::tick(double /*dt*/) {}

void KinematicRunPlugin2::shutdown() {
    if (modelPluginService_ && animationRegistered_) {
        auto& registry = modelPluginService_->modelFactoryRegistry();
        auto* proto    = registry.getRegisteredPrototype(modelType_);
        auto* anim     = dynamic_cast<arkheon::astsim::IAnimationModel*>(proto);
        if (anim) {
            anim->registerAnimation(animationCode_, nullptr);
        }
    }
    animationRegistered_ = false;
    modelPluginService_  = nullptr;
}

} // namespace arkheon::kinematic2

// =============================================================================
// C exports (required by the N8RO plugin loader)
// =============================================================================
extern "C" {

ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
    return new arkheon::kinematic2::KinematicRunPlugin2();
}

ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin) {
    delete plugin;
}

ARKHEON_ASTLIB_API const char* get_plugin_signature() {
    return "ARKHEON_PLUGIN_V1";
}

} // extern "C"
