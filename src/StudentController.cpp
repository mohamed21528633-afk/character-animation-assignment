
#include "arkheon/character/ICharacterController.h"

#include <cstring>
#include <cmath>
#include <string>
#include <unordered_set>
#include <core/logging/GlobalLogger.h>

#include <model/AnimationModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>
#include <plugin/IPlugin.h>
#include <plugin/IPluginServices.h>
#include <plugin/PluginContext.h>

namespace {

    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kWalkSpeed = 0.02f;
    constexpr float kRunSpeed = 0.07f;
    constexpr float kManualSpeed = 0.03f;
    constexpr float kTurnStepRad = 0.10f;
    constexpr float kWalkDuration = 15.0f;
    constexpr float kRunDuration = 15.0f;
    constexpr float kJumpDuration = 10.0f;
    constexpr float kPullDuration = 20.0f;
    constexpr float kJumpCycleDuration = 1.2f;
    constexpr double kSequenceTotalDuration = 60.0;

    enum MotionPhase {
        MOTION_WALK = 0,
        MOTION_RUN = 1,
        MOTION_JUMP = 2,
        MOTION_PULL = 3,
        MOTION_STOP = 4
    };

    struct Controller {
        float seg_len[10] = { 0 };
        int32_t last_seq_id = -1;
        int32_t active_motion = 0;

        float arm_swing = 0.0f;
        float leg_swing = 0.0f;
        float torso_sway = 0.0f;
        float jump_height = 0.0f;

        int motion_phase = MOTION_WALK;
        float phase_time = 0.0f;
        bool sequence_enabled = true;

        int debug_counter = 0;
    };

    static arkheon_quat quat_axis_angle(float x, float y, float z, float angle_rad)
    {
        const float len = std::sqrt(x * x + y * y + z * z);
        if (len <= 0.000001f)
        {
            return { 0, 0, 0, 1 };
        }

        const float half = angle_rad * 0.5f;
        const float s = std::sin(half) / len;
        return { x * s, y * s, z * s, std::cos(half) };
    }

    static arkheon_quat quat_mul(const arkheon_quat& a, const arkheon_quat& b)
    {
        return {
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        };
    }

    static bool has_manual_locomotion(const arkheon_input_state* input)
    {
        return input &&
            (input->keys[26] || input->keys[22] ||
             input->keys[4]  || input->keys[7]);
    }

    static float clamp01(float v)
    {
        if (v < 0.0f)
        {
            return 0.0f;
        }
        if (v > 1.0f)
        {
            return 1.0f;
        }
        return v;
    }

    static bool has_joint(
        const std::unordered_set<std::string>& available_joints,
        const char* joint_id)
    {
        if (!joint_id || *joint_id == '\0')
        {
            return false;
        }
        return available_joints.empty() ||
            available_joints.find(joint_id) != available_joints.end();
    }

    static void add_joint_override(
        arkheon::astsim::AnimationModelOutput& output,
        const std::unordered_set<std::string>& available_joints,
        const char* joint_id,
        double x_rad,
        double y_rad,
        double z_rad)
    {
        if (has_joint(available_joints, joint_id))
        {
            output.jointOverrides.push_back({ joint_id, x_rad, y_rad, z_rad });
        }
    }

    static MotionPhase phase_from_time(double t)
    {
        if (t < kWalkDuration)
        {
            return MOTION_WALK;
        }
        if (t < kWalkDuration + kRunDuration)
        {
            return MOTION_RUN;
        }
        if (t < kWalkDuration + kRunDuration + kJumpDuration)
        {
            return MOTION_JUMP;
        }
        if (t < kSequenceTotalDuration)
        {
            return MOTION_PULL;
        }
        return MOTION_STOP;
    }

    static double phase_local_time(double t, MotionPhase phase)
    {
        switch (phase)
        {
        case MOTION_WALK:
            return t;
        case MOTION_RUN:
            return t - kWalkDuration;
        case MOTION_JUMP:
            return t - kWalkDuration - kRunDuration;
        case MOTION_PULL:
            return t - kWalkDuration - kRunDuration - kJumpDuration;
        default:
            return 0.0;
        }
    }

    static bool evaluate_n8ro_sequence(
        const arkheon::astsim::AnimationModelInput& input,
        arkheon::astsim::AnimationModelOutput& output)
    {
        std::unordered_set<std::string> available_joints;
        available_joints.reserve(input.entity.joints.size() * 2);
        for (const auto& joint : input.entity.joints)
        {
            available_joints.insert(joint.jointId);
            available_joints.insert(joint.externalJointName);
        }

        const double t = std::min(input.simulationTimeSeconds, kSequenceTotalDuration);
        const MotionPhase phase = phase_from_time(t);
        const double local_t = phase_local_time(t, phase);

        output.clearExistingJointOverrides = true;
        output.jointOverrides.clear();

        if (phase == MOTION_STOP)
        {
            add_joint_override(output, available_joints, "leftShoulder", 0.20, 0.0, -0.10);
            add_joint_override(output, available_joints, "rightShoulder", 0.20, 0.0, 0.10);
            add_joint_override(output, available_joints, "leftElbow", 0.35, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightElbow", 0.35, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftHip", -0.30, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightHip", -0.30, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftKnee", 0.10, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightKnee", 0.10, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftAnkle", -0.05, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightAnkle", -0.05, 0.0, 0.0);
            return !output.jointOverrides.empty();
        }

        if (phase == MOTION_WALK || phase == MOTION_RUN)
        {
            const double speed = phase == MOTION_WALK ? 4.0 : 10.0;
            const double amp = phase == MOTION_WALK ? 1.0 : 1.6;
            const double leg_l = std::sin(local_t * speed) * amp;
            const double leg_r = std::sin(local_t * speed + kPi) * amp;
            const double arm_l = -leg_l * 0.85;
            const double arm_r = -leg_r * 0.85;
            const double knee_l = std::max(0.0, leg_l) * 0.75 + 0.15;
            const double knee_r = std::max(0.0, leg_r) * 0.75 + 0.15;
            const double ankle_l = -knee_l * 0.45;
            const double ankle_r = -knee_r * 0.45;
            const double sway = std::sin(local_t * speed * 0.5) * 0.25;

            add_joint_override(output, available_joints, "leftHip", -0.45 + leg_l * 0.55, sway, leg_l * 0.20);
            add_joint_override(output, available_joints, "rightHip", -0.45 + leg_r * 0.55, -sway, leg_r * 0.20);
            add_joint_override(output, available_joints, "leftKnee", knee_l, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightKnee", knee_r, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftAnkle", ankle_l, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightAnkle", ankle_r, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftShoulder", 0.35 + arm_l * 0.45, arm_l * 0.35, -0.35 + arm_l * 0.25);
            add_joint_override(output, available_joints, "rightShoulder", 0.35 + arm_r * 0.45, arm_r * 0.35, 0.35 + arm_r * 0.25);
            add_joint_override(output, available_joints, "leftElbow", 0.45 - arm_l * 0.20, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightElbow", 0.45 - arm_r * 0.20, 0.0, 0.0);
            add_joint_override(output, available_joints, "spineLower", sway * 0.35, sway * 0.20, 0.0);
            add_joint_override(output, available_joints, "spineUpper", sway * 0.20, -sway * 0.15, 0.0);
            return !output.jointOverrides.empty();
        }

        if (phase == MOTION_JUMP)
        {
            const double cycle = std::fmod(local_t, kJumpCycleDuration) / kJumpCycleDuration;
            const double jump_wave = std::sin(cycle * kPi);
            double knee = 0.20;
            double hip = -0.25;
            double shoulder = 0.40;
            double elbow = 0.35;
            double ankle = -0.10;

            if (cycle < 0.25)
            {
                const double crouch = cycle / 0.25;
                knee = 0.20 + crouch * 1.35;
                hip = -0.25 - crouch * 0.55;
                shoulder = 0.40 + crouch * 0.30;
                elbow = 0.35 + crouch * 0.40;
                ankle = -0.10 - crouch * 0.35;
            }
            else if (cycle < 0.55)
            {
                const double extend = (cycle - 0.25) / 0.30;
                knee = 1.55 - extend * 1.25;
                hip = -0.80 + extend * 0.65;
                shoulder = 0.70 + extend * 0.90;
                elbow = 0.75 - extend * 0.35;
                ankle = -0.45 + extend * 0.30;
            }
            else
            {
                const double land = (cycle - 0.55) / 0.45;
                knee = 0.30 + land * 0.85;
                hip = -0.15 - land * 0.20;
                shoulder = 1.60 - land * 1.10;
                elbow = 0.40 + land * 0.15;
                ankle = -0.15 - land * 0.20;
            }

            add_joint_override(output, available_joints, "leftHip", hip, jump_wave * 0.15, 0.0);
            add_joint_override(output, available_joints, "rightHip", hip, -jump_wave * 0.15, 0.0);
            add_joint_override(output, available_joints, "leftKnee", knee, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightKnee", knee, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftAnkle", ankle, 0.0, 0.0);
            add_joint_override(output, available_joints, "rightAnkle", ankle, 0.0, 0.0);
            add_joint_override(output, available_joints, "leftShoulder", shoulder, jump_wave * 0.25, -0.55 - jump_wave * 0.35);
            add_joint_override(output, available_joints, "rightShoulder", shoulder, -jump_wave * 0.25, 0.55 + jump_wave * 0.35);
            add_joint_override(output, available_joints, "leftElbow", elbow, 0.0, -0.20);
            add_joint_override(output, available_joints, "rightElbow", elbow, 0.0, 0.20);
            add_joint_override(output, available_joints, "spineLower", -0.20 + jump_wave * 0.35, 0.0, 0.0);
            return !output.jointOverrides.empty();
        }

        // MOTION_PULL
        const double pull_wave = std::sin(local_t * 2.0);
        const double pull_effort = 0.65 + std::fabs(pull_wave) * 0.25;

        add_joint_override(output, available_joints, "leftShoulder", 0.55, pull_wave * 0.20, -1.35 - pull_effort * 0.15);
        add_joint_override(output, available_joints, "rightShoulder", 0.55, -pull_wave * 0.20, 1.35 + pull_effort * 0.15);
        add_joint_override(output, available_joints, "leftElbow", 1.75 + pull_effort * 0.25, pull_wave * 0.15, -0.35);
        add_joint_override(output, available_joints, "rightElbow", 1.75 + pull_effort * 0.25, -pull_wave * 0.15, 0.35);
        add_joint_override(output, available_joints, "leftHip", -0.55, pull_wave * 0.10, 0.0);
        add_joint_override(output, available_joints, "rightHip", -0.55, -pull_wave * 0.10, 0.0);
        add_joint_override(output, available_joints, "leftKnee", 0.55 + pull_effort * 0.15, 0.0, 0.0);
        add_joint_override(output, available_joints, "rightKnee", 0.55 + pull_effort * 0.15, 0.0, 0.0);
        add_joint_override(output, available_joints, "leftAnkle", -0.35, 0.0, 0.0);
        add_joint_override(output, available_joints, "rightAnkle", -0.35, 0.0, 0.0);
        add_joint_override(output, available_joints, "spineLower", -0.75 + pull_wave * 0.10, pull_wave * 0.15, 0.0);
        add_joint_override(output, available_joints, "spineUpper", -0.55 + pull_wave * 0.08, -pull_wave * 0.12, 0.0);

        return !output.jointOverrides.empty();
    }

    class N8roSequencePlugin final : public arkheon::astlib::IPlugin {
    public:
        int getInterfaceVersion() const override
        {
            return 1;
        }

        arkheon::astlib::PluginMetadata getMetadata() const override
        {
            arkheon::astlib::PluginMetadata metadata;
            metadata.setPluginId("character-plugin-230201904");
            metadata.setVersion("1.0.0");
            metadata.setAuthor("Student 230201904");
            return metadata;
        }

        void initialize(arkheon::astlib::PluginContext& context) override
        {
            shutdown_ = false;
            animation_registered_ = false;
            model_factory_registry_ = nullptr;

            if (!context.services)
            {
                AST_LOG_WARNING_SIMPLE("[character-plugin-230201904] Plugin services unavailable.");
                return;
            }

            auto* raw_service = context.services->getService(
                arkheon::astsim::IModelPluginService::kPluginServiceId);
            auto* service =
                static_cast<arkheon::astsim::IModelPluginService*>(raw_service);
            model_factory_registry_ = service ? &service->modelFactoryRegistry() : nullptr;
            if (!model_factory_registry_)
            {
                AST_LOG_WARNING_SIMPLE("[character-plugin-230201904] Model factory registry unavailable.");
                return;
            }

            auto* prototype_base =
                model_factory_registry_->getRegisteredPrototype(model_type_);
            if (!prototype_base)
            {
                prototype_base =
                    model_factory_registry_->getRegisteredPrototype("AnimationModelNathanHuman");
            }
            if (!prototype_base)
            {
                AST_LOG_WARNING_SIMPLE("[character-plugin-230201904] animationModelNathanHuman prototype not found.");
                return;
            }

            auto* animation_model =
                dynamic_cast<arkheon::astsim::IAnimationModel*>(prototype_base);
            if (!animation_model)
            {
                AST_LOG_WARNING_SIMPLE("[character-plugin-230201904] Prototype is not an animation model.");
                return;
            }

            animation_registered_ =
                animation_model->registerAnimation("Idle Alert", evaluate_n8ro_sequence);
            animation_registered_ =
                animation_model->registerAnimation("Idle Neutral", evaluate_n8ro_sequence) ||
                animation_registered_;
            animation_registered_ =
                animation_model->registerAnimation("Idle Breathing", evaluate_n8ro_sequence) ||
                animation_registered_;
            animation_registered_ =
                animation_model->registerAnimation("Idle Shake", evaluate_n8ro_sequence) ||
                animation_registered_;

            AST_LOG_INFO_SIMPLE(
                std::string("[character-plugin-230201904] Motion sequence registered: ") +
                (animation_registered_ ? "yes" : "no"));
        }

        void tick(double dt) override
        {
            static_cast<void>(dt);
        }

        void shutdown() override
        {
            if (model_factory_registry_ && animation_registered_)
            {
                auto* prototype_base =
                    model_factory_registry_->getRegisteredPrototype(model_type_);
                if (!prototype_base)
                {
                    prototype_base =
                        model_factory_registry_->getRegisteredPrototype("AnimationModelNathanHuman");
                }

                auto* animation_model =
                    dynamic_cast<arkheon::astsim::IAnimationModel*>(prototype_base);
                if (animation_model)
                {
                    static_cast<void>(animation_model->registerAnimation(
                        "Idle Alert",
                        arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
                    static_cast<void>(animation_model->registerAnimation(
                        "Idle Neutral",
                        arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
                    static_cast<void>(animation_model->registerAnimation(
                        "Idle Breathing",
                        arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
                    static_cast<void>(animation_model->registerAnimation(
                        "Idle Shake",
                        arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
                }
            }

            animation_registered_ = false;
            shutdown_ = true;
            model_factory_registry_ = nullptr;
        }

    private:
        bool shutdown_ = false;
        bool animation_registered_ = false;
        std::string model_type_ = "animationModelNathanHuman";
        arkheon::astsim::ModelFactoryRegistry* model_factory_registry_ = nullptr;
    };

} // anonymous namespace

static arkheon_vec3 compute_com(
    const arkheon_bone_state bones[66])
{
    arkheon_vec3 com = { 0, 0, 0 };
    float total_mass = 0.0f;

    struct Segment {
        int bone_index;
        float mass;
    };

    Segment segments[] =
    {
        {9,  3.0f},
        {33, 3.0f},
        {10, 2.0f},
        {34, 2.0f},
        {56, 8.0f},
        {61, 8.0f},
        {57, 5.0f},
        {62, 5.0f},
        {58, 2.0f},
        {63, 2.0f}
    };

    for (const auto& s : segments)
    {
        arkheon_vec3 p = bones[s.bone_index].world_position;
        com.x += p.x * s.mass;
        com.y += p.y * s.mass;
        com.z += p.z * s.mass;
        total_mass += s.mass;
    }

    if (total_mass > 0.0f)
    {
        com.x /= total_mass;
        com.y /= total_mass;
        com.z /= total_mass;
    }

    return com;
}

extern "C" {

    ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin()
    {
        return new N8roSequencePlugin();
    }

    ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin)
    {
        delete plugin;
    }

    ARKHEON_ASTLIB_API const char* get_plugin_signature()
    {
        return "ARKHEON_PLUGIN_V1";
    }

    ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void)
    {
        return ARKHEON_CHARACTER_SDK_VERSION;
    }

    ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void)
    {
        return "Student Plugin v0.1";
    }

    ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(
        void* /*h*/,
        int32_t out[3])
    {
        out[0] = 12;
        out[1] = 47;
        out[2] = 83;
    }

    ARKHEON_CHAR_EXPORT void* arkheon_character_create(
        const float segs[10])
    {
        auto* c = new Controller();
        std::memcpy(c->seg_len, segs, sizeof(c->seg_len));
        c->motion_phase = MOTION_WALK;
        c->phase_time = 0.0f;
        c->sequence_enabled = true;
        return c;
    }

    ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* h)
    {
        delete static_cast<Controller*>(h);
    }

    ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
        void* h,
        const arkheon_frame* frame,
        const arkheon_bone_state in_bones[66],
        arkheon_bone_override out_overrides[10],
        arkheon_vec3* out_root_translation_delta,
        arkheon_quat* out_root_rotation_delta,
        const arkheon_input_state* input,
        const arkheon_mission_goal* goal,
        const arkheon_env_api* /*env*/)
    {
        auto* c = static_cast<Controller*>(h);
        if (!c || !in_bones || !out_overrides ||
            !out_root_translation_delta || !out_root_rotation_delta)
        {
            return 1;
        }

        for (int i = 0; i < 10; ++i)
        {
            out_overrides[i].apply = 0;
            out_overrides[i].local_rotation = { 0, 0, 0, 1 };
        }

        *out_root_translation_delta = { 0, 0, 0 };
        *out_root_rotation_delta = { 0, 0, 0, 1 };

        bool is_moving = false;
        const bool manual_input = has_manual_locomotion(input);
        const bool paused = frame && frame->is_paused;

        if (input && !paused)
        {
            if (input->keys[26])
            {
                out_root_translation_delta->z += kManualSpeed;
                is_moving = true;
            }
            if (input->keys[22])
            {
                out_root_translation_delta->z -= kManualSpeed;
                is_moving = true;
            }
            if (input->keys[4])
            {
                out_root_translation_delta->x -= kManualSpeed;
                is_moving = true;
            }
            if (input->keys[7])
            {
                out_root_translation_delta->x += kManualSpeed;
                is_moving = true;
            }
            if (input->keys[4])
            {
                *out_root_rotation_delta = quat_axis_angle(0.0f, 1.0f, 0.0f, -kTurnStepRad);
            }
            if (input->keys[7])
            {
                *out_root_rotation_delta = quat_axis_angle(0.0f, 1.0f, 0.0f, kTurnStepRad);
            }

            if (input->hotkey_motion_a)
            {
                c->motion_phase = MOTION_WALK;
                c->phase_time = 0.0f;
                c->jump_height = 0.0f;
                c->sequence_enabled = true;
            }
            if (input->hotkey_motion_b)
            {
                c->motion_phase = MOTION_PULL;
                c->phase_time = 0.0f;
                c->jump_height = 0.0f;
                c->sequence_enabled = true;
            }
            if (input->hotkey_motion_c)
            {
                c->motion_phase = MOTION_JUMP;
                c->phase_time = 0.0f;
                c->jump_height = 0.0f;
                c->sequence_enabled = true;
            }
        }

        if (frame && !paused)
        {
            c->phase_time += static_cast<float>(frame->delta_time_s);
        }

        if (c->sequence_enabled && !paused)
        {
            if (c->motion_phase == MOTION_WALK && c->phase_time > kWalkDuration)
            {
                c->motion_phase = MOTION_RUN;
                c->phase_time = 0.0f;
            }
            else if (c->motion_phase == MOTION_RUN && c->phase_time > kRunDuration)
            {
                c->motion_phase = MOTION_JUMP;
                c->phase_time = 0.0f;
            }
            else if (c->motion_phase == MOTION_JUMP && c->phase_time > kJumpDuration)
            {
                c->motion_phase = MOTION_PULL;
                c->phase_time = 0.0f;
            }
            else if (c->motion_phase == MOTION_PULL && c->phase_time > kPullDuration)
            {
                c->motion_phase = MOTION_STOP;
                c->phase_time = 0.0f;
                c->sequence_enabled = false;
            }
        }

        if (c->sequence_enabled && !manual_input && !paused && c->motion_phase == MOTION_WALK)
        {
            out_root_translation_delta->z += kWalkSpeed;
            is_moving = true;
        }
        else if (c->sequence_enabled && !manual_input && !paused && c->motion_phase == MOTION_RUN)
        {
            out_root_translation_delta->z += kRunSpeed;
            is_moving = true;
        }
        else if (c->sequence_enabled && !manual_input && !paused && c->motion_phase == MOTION_JUMP)
        {
            const float cycle_time = std::fmod(c->phase_time, kJumpCycleDuration);
            const float jump_t = clamp01(cycle_time / kJumpCycleDuration);
            const float next_height = std::sin(jump_t * kPi) * 0.35f;
            out_root_translation_delta->y += next_height - c->jump_height;
            out_root_translation_delta->z += kWalkSpeed * 0.75f;
            c->jump_height = next_height;
            is_moving = true;
        }
        else if (c->sequence_enabled && !manual_input && !paused && c->motion_phase == MOTION_PULL)
        {
            c->jump_height = 0.0f;
            out_root_translation_delta->z -= 0.01f;
            is_moving = true;
        }
        else if (c->motion_phase != MOTION_JUMP)
        {
            c->jump_height = 0.0f;
        }

        arkheon_vec3 com = compute_com(in_bones);

        const float support_mid_x =
            (in_bones[58].world_position.x + in_bones[63].world_position.x) * 0.5f;
        const float lateral_com = com.x - support_mid_x;
        float left_balance_angle = 0.0f;
        float right_balance_angle = 0.0f;

        if (lateral_com > 0.05f)
        {
            left_balance_angle = -0.06f;
        }
        else if (lateral_com < -0.05f)
        {
            right_balance_angle = 0.06f;
        }

        float t = frame ? static_cast<float>(frame->simulation_time_s) : 0.0f;
        float local_t = c->phase_time;
        float target_wave = 0.0f;
        float target_leg_l = 0.0f;
        float target_leg_r = 0.0f;
        float target_elbow = 0.0f;
        float pull_forward = 0.0f;

        if (c->motion_phase == MOTION_WALK)
        {
            const float phase = t * 4.0f;
            target_leg_l = std::sin(phase);
            target_leg_r = std::sin(phase + kPi);
            target_wave = -target_leg_l;
            target_elbow = 0.25f;
        }
        else if (c->motion_phase == MOTION_RUN)
        {
            const float phase = t * 10.0f;
            target_leg_l = std::sin(phase) * 1.6f;
            target_leg_r = std::sin(phase + kPi) * 1.6f;
            target_wave = -target_leg_l;
            target_elbow = 0.35f;
        }
        else if (c->motion_phase == MOTION_JUMP)
        {
            const float cycle_time = std::fmod(local_t, kJumpCycleDuration);
            const float jump_t = clamp01(cycle_time / kJumpCycleDuration);
            target_leg_l = 0.35f + std::sin(jump_t * kPi) * 0.65f;
            target_leg_r = target_leg_l;
            target_wave = 0.55f + std::sin(jump_t * kPi) * 0.45f;
            target_elbow = 0.20f;
        }
        else if (c->motion_phase == MOTION_PULL)
        {
            target_wave = std::sin(t * 2.0f) * 0.15f;
            target_leg_l = 0.35f;
            target_leg_r = 0.35f;
            target_elbow = 0.85f;
            pull_forward = -0.95f + std::sin(t * 2.0f) * 0.08f;
        }

        c->arm_swing += (target_wave - c->arm_swing) * 0.12f;
        c->leg_swing += (target_leg_l - c->leg_swing) * 0.12f;

        const float wave = c->arm_swing;
        const float leg_l = target_leg_l * 0.12f + c->leg_swing * 0.88f;
        const float leg_r = target_leg_r * 0.12f + c->leg_swing * 0.88f;

        out_overrides[0].apply = 1;
        out_overrides[1].apply = 1;
        if (c->motion_phase == MOTION_PULL)
        {
            out_overrides[0].local_rotation =
                quat_mul(quat_axis_angle(0.0f, 0.0f, 1.0f, pull_forward),
                         quat_axis_angle(1.0f, 0.0f, 0.0f, 0.35f));
            out_overrides[1].local_rotation =
                quat_mul(quat_axis_angle(0.0f, 0.0f, 1.0f, -pull_forward),
                         quat_axis_angle(1.0f, 0.0f, 0.0f, 0.35f));
        }
        else
        {
            out_overrides[0].local_rotation = quat_axis_angle(1.0f, 0.0f, 0.0f, wave * 0.55f);
            out_overrides[1].local_rotation = quat_axis_angle(1.0f, 0.0f, 0.0f, -wave * 0.55f);
        }

        out_overrides[4].apply = 1;
        out_overrides[4].local_rotation =
            quat_axis_angle(1.0f, 0.0f, 0.0f, leg_l * 0.45f + left_balance_angle);
        out_overrides[5].apply = 1;
        out_overrides[5].local_rotation =
            quat_axis_angle(1.0f, 0.0f, 0.0f, leg_r * 0.45f + right_balance_angle);

        out_overrides[6].apply = 1;
        out_overrides[6].local_rotation =
            quat_axis_angle(1.0f, 0.0f, 0.0f, std::max(0.0f, leg_l) * 0.35f + 0.08f);
        out_overrides[7].apply = 1;
        out_overrides[7].local_rotation =
            quat_axis_angle(1.0f, 0.0f, 0.0f, std::max(0.0f, leg_r) * 0.35f + 0.08f);

        out_overrides[8].apply = 1;
        out_overrides[8].local_rotation =
            quat_axis_angle(1.0f, 0.0f, 0.0f, -std::max(0.0f, leg_l) * 0.15f);
        out_overrides[9].apply = 1;
        out_overrides[9].local_rotation =
            quat_axis_angle(1.0f, 0.0f, 0.0f, -std::max(0.0f, leg_r) * 0.15f);

        out_overrides[2].apply = 1;
        out_overrides[3].apply = 1;
        out_overrides[2].local_rotation = quat_axis_angle(1.0f, 0.0f, 0.0f, target_elbow);
        out_overrides[3].local_rotation = quat_axis_angle(1.0f, 0.0f, 0.0f, target_elbow);

        if (goal && goal->sequence_id != c->last_seq_id)
        {
            c->last_seq_id = goal->sequence_id;
        }

        static_cast<void>(is_moving);
        static_cast<void>(com);
        static_cast<void>(c->debug_counter);

        return 0;
    }

} // extern "C"
