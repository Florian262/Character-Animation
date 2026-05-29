// mock_host2.cpp — Math verification harness for kinematic_run_plugin.dll (v2)
// Mirrors the mock host from Char Animation1, updated for the new architecture.

#include <iostream>
#include <windows.h>
#include <memory>
#include <string_view>
#include <functional>
#include <cstdio>

#include <plugin/IPlugin.h>
#include <plugin/IPluginServices.h>
#include <plugin/PluginContext.h>
#include <plugin/PluginMetadata.h>
#include <plugin/PluginStats.h>
#include <plugin/IPluginLoader.h>
#include <model/IModel.h>
#include <model/AnimationModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>

typedef arkheon::astlib::IPlugin* (*CreatePluginFn)();
typedef void (*DestroyPluginFn)(arkheon::astlib::IPlugin*);
typedef const char* (*GetPluginSignatureFn)();

// ---------------------------------------------------------------------------
// Mock Model
// ---------------------------------------------------------------------------
class MockAnimationModel2
    : public arkheon::astsim::IModel,
      public arkheon::astsim::IAnimationModel
{
public:
    [[nodiscard]] std::string getTypeName() const override { return "MockAnimationModel2"; }
    [[nodiscard]] std::unique_ptr<IModel> clone() const override { return nullptr; }

    void configure(const arkheon::astlib::JsonValue& /*config*/) override {}

    [[nodiscard]] bool evaluate(
        const arkheon::astsim::AnimationModelInput& input,
        arkheon::astsim::AnimationModelOutput& output) override
    {
        if (evalFunc_) return evalFunc_(input, output);
        return false;
    }

    [[nodiscard]] bool registerAnimation(
        std::string_view animationCode,
        AnimationEvaluationFunction fn) override
    {
        std::cout << "[Mock Host v2] Plugin registered animation: " << animationCode << "\n";
        evalFunc_ = fn;
        return true;
    }

    arkheon::astsim::IAnimationModel::AnimationEvaluationFunction evalFunc_;
};

// ---------------------------------------------------------------------------
// Mock Services
// ---------------------------------------------------------------------------
class MockModelPluginService2 : public arkheon::astsim::IModelPluginService {
public:
    arkheon::astsim::ModelFactoryRegistry& modelFactoryRegistry() override { return reg_; }
    const arkheon::astsim::ModelFactoryRegistry& modelFactoryRegistry() const override { return reg_; }
    arkheon::astsim::ModelFactoryRegistry reg_;
};

class MockPluginServices2 : public arkheon::astlib::IPluginServices {
public:
    arkheon::astlib::DbSchemaManager* getSchemaManager() override { return nullptr; }
    arkheon::astlib::IMessageBus* getMessageBus() override { return nullptr; }
    void* getService(std::string_view id) override {
        if (id == arkheon::astsim::IModelPluginService::kPluginServiceId) return &modelSvc_;
        return nullptr;
    }
    bool hasService(std::string_view id) const override {
        return id == arkheon::astsim::IModelPluginService::kPluginServiceId;
    }
    MockModelPluginService2 modelSvc_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::cout << "=============================================\n";
    std::cout << "   N8RO Mock Host v2 — Normalized Gait Test \n";
    std::cout << "=============================================\n";

    const char* dllPath = "kinematic_run_plugin.dll";
    if (argc > 1) dllPath = argv[1];

    std::cout << "[Mock Host v2] Loading: " << dllPath << "...\n";
    HMODULE hLib = LoadLibraryA(dllPath);
    if (!hLib && argc <= 1) {
        const char* fb1 = "Release\\kinematic_run_plugin.dll";
        std::cout << "[Mock Host v2] Trying fallback: " << fb1 << "...\n";
        hLib = LoadLibraryA(fb1);
        if (!hLib) {
            const char* fb2 = "C:\\N8RO\\userPlugins\\sim\\kinematic_run_plugin.dll";
            std::cout << "[Mock Host v2] Trying deployed path: " << fb2 << "...\n";
            hLib = LoadLibraryA(fb2);
            if (hLib) dllPath = fb2;
        } else {
            dllPath = fb1;
        }
    }

    if (!hLib) {
        std::cerr << "[Mock Host v2] ERROR: Could not load DLL. Code: " << GetLastError() << "\n";
        return 1;
    }
    std::cout << "[Mock Host v2] Loaded: " << dllPath << "\n";

    auto get_sig     = (GetPluginSignatureFn)GetProcAddress(hLib, "get_plugin_signature");
    auto create_plg  = (CreatePluginFn)GetProcAddress(hLib, "create_plugin");
    auto destroy_plg = (DestroyPluginFn)GetProcAddress(hLib, "destroy_plugin");

    if (!get_sig || !create_plg || !destroy_plg) {
        std::cerr << "[Mock Host v2] ERROR: Missing exported functions.\n";
        FreeLibrary(hLib); return 1;
    }

    std::cout << "[Mock Host v2] Signature: " << get_sig() << "\n";

    // Build mock environment
    MockPluginServices2 services;
    auto mockModel = std::make_unique<MockAnimationModel2>();
    auto* mockPtr  = mockModel.get();
    services.modelSvc_.reg_.registerFactory("animationModelNathanHuman", std::move(mockModel));

    // Create & initialize plugin
    arkheon::astlib::IPlugin* plugin = create_plg();
    if (!plugin) { std::cerr << "[Mock Host v2] ERROR: create_plugin returned null.\n"; FreeLibrary(hLib); return 1; }

    arkheon::astlib::PluginContext ctx;
    ctx.services = &services;
    plugin->initialize(ctx);

    // Run evaluation loop
    if (mockPtr->evalFunc_) {
        std::cout << "[Mock Host v2] Evaluation callback registered. Running test loop...\n";
        std::cout << "[Mock Host v2] Step Freq = 2.5 Hz | Phi range [0,1) per stride\n\n";

        // Test across one full gait cycle (0.4 seconds = 1 stride at 2.5 Hz)
        const double stride_time = 1.0 / 2.5;   // 0.4 s
        const int    steps       = 10;

        for (int i = 0; i <= steps; ++i) {
            double t = stride_time * i / steps;

            arkheon::astsim::AnimationModelInput input;
            input.simulationTimeSeconds = t;
            input.deltaTimeSeconds      = stride_time / steps;

            arkheon::astsim::AnimationModelOutput output;
            bool ok = mockPtr->evalFunc_(input, output);

            double phi = std::fmod(t * 2.5, 1.0);
            const char* phase_R = (phi < 0.5) ? "Stance" : "Swing ";
            double phi_L = std::fmod(phi + 0.5, 1.0);
            const char* phase_L = (phi_L < 0.5) ? "Stance" : "Swing ";

            std::printf("\n--- t=%5.3fs | Phi=%.2f | Right=%-6s | Left=%-6s ---\n",
                        t, phi, phase_R, phase_L);

            if (ok) {
                for (const auto& ov : output.jointOverrides) {
                    std::printf("  %-16s -> X: %8.3f | Y: %8.3f | Z: %8.3f\n",
                                ov.jointId.c_str(), ov.xRad, ov.yRad, ov.zRad);
                }
            } else {
                std::printf("  (no overrides)\n");
            }
        }
    } else {
        std::cerr << "[Mock Host v2] ERROR: Plugin did not register its evaluation callback!\n";
    }

    std::cout << "\n[Mock Host v2] Shutting down...\n";
    plugin->shutdown();
    destroy_plg(plugin);
    FreeLibrary(hLib);
    std::cout << "[Mock Host v2] Done.\n";
    return 0;
}
