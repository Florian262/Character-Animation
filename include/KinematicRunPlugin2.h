#pragma once

#include <plugin/IPlugin.h>
#include <plugin/IModelPluginService.h>
#include <string>

namespace arkheon::kinematic2 {

class KinematicRunPlugin2 final : public arkheon::astlib::IPlugin {
public:
    KinematicRunPlugin2() = default;
    ~KinematicRunPlugin2() override = default;

    [[nodiscard]] int getInterfaceVersion() const override;
    [[nodiscard]] arkheon::astlib::PluginMetadata getMetadata() const override;

    void initialize(arkheon::astlib::PluginContext& context) override;
    void tick(double dt) override;
    void shutdown() override;

private:
    bool initialized_          = false;
    bool animationRegistered_  = false;
    std::string pluginId_      = "kinematic-run-plugin2";
    std::string modelType_     = "animationModelNathanHuman";
    std::string animationCode_ = "Run Kinematic";
    arkheon::astsim::IModelPluginService* modelPluginService_ = nullptr;
};

} // namespace arkheon::kinematic2

extern "C" {
ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin();
ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin);
ARKHEON_ASTLIB_API const char* get_plugin_signature();
}
