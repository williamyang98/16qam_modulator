#include <stdio.h>

// graphics code
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

// implot library
#include "implot.h"

// our imgui extras
#include "imgui_config.h"
#include "font_awesome_definitions.h"

#include <thread>
#include <stdlib.h>
#include <stdint.h>

#include "qam_demodulator.h"
#include "audio_processor.h"
#include "filter_designer.h"

#include <io.h>
#include <fcntl.h>

#define PRINT_LOG 1
#if PRINT_LOG 
  #define LOG_MESSAGE(...) fprintf(stderr, ##__VA_ARGS__)
#else
  #define LOG_MESSAGE(...) (void)0
#endif


// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

int is_main_window_focused = true;

static void glfw_error_callback(int error, const char* description)
{
    LOG_MESSAGE("Glfw Error %d: %s\n", error, description);
}

// this occurs when we minimise or change focus to another window
static void glfw_window_focus_callback(GLFWwindow* window, int focused)
{
    is_main_window_focused = focused;
}

class AudioFrameHandler: public QAM_Demodulator_Callback
{
public:
    struct {
        int total = 0;
        int incorrect = 0;
        int correct = 0;
        int corrupted = 0;
        int repaired = 0;
        float GetPacketErrorRate() {
            return  (float)incorrect / (float)total;
        }
        float GetRepairFailureRate() {
            return (float)repaired / (float)correct;
        }
        void reset() {
            total = 0;
            incorrect = 0;
            correct = 0;
            corrupted = 0;
            repaired = 0;
        }
    } stats;
private:
    const int frame_length;
    AudioProcessor* audio;
public: 
    AudioFrameHandler(AudioProcessor* _audio, const int _frame_length)
    : frame_length(_frame_length), audio(_audio) {}
public:
    virtual void OnFrameResult(
        FrameSynchroniser::Result res, 
        FrameDecoder::Payload payload)
    {
        using Res = FrameSynchroniser::Result;
        switch (res) {
        case Res::PREAMBLE_FOUND:
            break;
        case Res::BLOCK_SIZE_OK:
            break;
        case Res::BLOCK_SIZE_ERR:
            LOG_MESSAGE("Got invalid block size: %d\n", payload.length);
            stats.corrupted++;
            break;
        case Res::PAYLOAD_ERR:
            stats.total++;
            stats.incorrect++;
            break;
        case Res::PAYLOAD_OK:
            stats.total++;
            stats.correct++;
            if (payload.decoded_error > 0) {
                stats.repaired++;
            }
            if (payload.length == frame_length) {
                audio->ProcessFrame(payload.buf, payload.length);
            }
            break;
        case Res::NONE:
        default:
            break;
        }
    }
};

class App {
public:
    CarrierDemodulatorSpecification carrier_demod_spec;
    QAM_Demodulator_Specification qam_spec;
    ConstellationSpecification* constellation = NULL;
    FILE* rx_fp = NULL;

    AudioProcessor* audio_processor = NULL;
    AudioFrameHandler* audio_frame_handler = NULL;
    CarrierToSymbolDemodulatorBuffers* carrier_demod_buffer = NULL;
    CarrierToSymbolDemodulatorBuffers* snapshot_buffer = NULL;
    std::complex<uint8_t>* rx_buffer = NULL;
private:
    QAM_Demodulator* qam_demodulator = NULL;
public:
    struct {
        bool rebuild = false;
        bool snapshot = false;
    } controls;
    bool is_read_loop = false;
public:
    App() {}
    void Run() {
        int rd_total_blocks = 0;
        while (true) {
            const int ds_block_size = qam_spec.block_size * qam_spec.downsample_filter.factor;
            size_t rd_block_size = fread(rx_buffer, sizeof(std::complex<uint8_t>), ds_block_size, rx_fp);
            if (rd_block_size != ds_block_size) {
                LOG_MESSAGE("Got mismatched block size after %d blocks\n", rd_total_blocks);
                if (is_read_loop) {
                    fseek(rx_fp, 0, 0);
                    continue;
                }
                break;
            }
            rd_total_blocks++; 

            if (qam_demodulator != NULL) {
                qam_demodulator->Process(rx_buffer);
            }

            if (ReadFlag(controls.snapshot)) {
                snapshot_buffer->CopyFrom(carrier_demod_buffer);
            }

            if (ReadFlag(controls.rebuild)) {
                BuildDemodulator();
            }
        }
    }
    void BuildDemodulator() {
        if (qam_demodulator != NULL) {
            delete qam_demodulator;
        }
        qam_demodulator = new QAM_Demodulator(carrier_demod_spec, qam_spec, constellation, carrier_demod_buffer);
        qam_demodulator->SetCallback(audio_frame_handler);
    }
private:
    bool ReadFlag(bool& flag) {
        const bool rv = flag;
        flag = false;
        return rv;
    }
};

void demodulator_thread(App* app) 
{
    app->Run();
}

int main(int argc, char** argv)
{
    // app startup
    FILE* fp_in = stdin;

    if (argc > 1) {
        FILE* tmp = NULL;
        fopen_s(&tmp, argv[1], "r");
        if (tmp == NULL) {
            LOG_MESSAGE("Failed to open file for reading\n");
            return 1;
        } 
        fp_in = tmp;
    }

    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    _setmode(_fileno(fp_in), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);

    // const int block_size = 4096;
    const int ds_factor = 1;
    const int block_size = 8192 / ds_factor;
    const float Fsample = 1e6 / (float)(ds_factor);
    const float Fsymbol = 87e3;
    const float Faudio = Fsymbol/5.0f;
    const int audio_buffer_size = (int)(std::ceilf(Faudio));

    auto app = new App();
    app->constellation = new SquareConstellation(4);
    app->rx_fp = fp_in;

    const int audio_frame_length = 100;
    app->audio_processor = new AudioProcessor(audio_buffer_size, audio_frame_length, Faudio);
    app->audio_frame_handler = new AudioFrameHandler(app->audio_processor, audio_frame_length);
    app->carrier_demod_buffer = new CarrierToSymbolDemodulatorBuffers(block_size);
    app->snapshot_buffer = new CarrierToSymbolDemodulatorBuffers(block_size);
    app->rx_buffer = new std::complex<uint8_t>[block_size*ds_factor];

    {
        const float PI = 3.1415f;
        auto& spec = app->carrier_demod_spec;
        spec.f_sample = Fsample; 
        spec.f_symbol = Fsymbol;
        spec.baseband_filter.cutoff = Fsymbol;
        spec.baseband_filter.M = 10;
        spec.ac_filter.k = 0.99999f;
        spec.agc.beta = 0.1f;
        spec.agc.initial_gain = 0.1f;
        spec.carrier_pll.f_center = 0e3;
        spec.carrier_pll.f_gain = 2.5e3;
        spec.carrier_pll.phase_error_gain = 8.0f/PI;
        spec.carrier_pll_filter.butterworth_cutoff = 5e3;
        spec.carrier_pll_filter.integrator_gain = 1000.0f;
        spec.ted_pll.f_gain = 5e3;
        spec.ted_pll.f_offset = 0e3;
        spec.ted_pll.phase_error_gain = 1.0f;
        spec.ted_pll_filter.butterworth_cutoff = 10e3;
        spec.ted_pll_filter.integrator_gain = 250.0f;
    }
    {
        auto& spec = app->qam_spec;
        spec.block_size = block_size;
        spec.scrambler_syncword = 0b1000010101011001;
        spec.preamble_code = 0b11111001101011111100110101101101;
        spec.crc8_polynomial = 0xD5;
    }

    const auto original_carrier_demod_spec = app->carrier_demod_spec;
    const auto original_qam_spec = app->qam_spec;

    app->BuildDemodulator();

    // swap between live and snapshot buffer
    auto carrier_demod_buffer = app->carrier_demod_buffer;
    auto snapshot_buffer = app->snapshot_buffer;
    auto render_buffer = carrier_demod_buffer;

    // save demodulated audio
    const int16_t audio_gain_min = 0;
    const int16_t audio_gain_max = 32;

    auto demod_thread = std::thread(demodulator_thread, app);

    // Generate x-axis timescale for implot
    auto time_scale = new float[block_size];
    const float Fs = 1e6;
    const float Ts = 1.0f/Fs;
    for (int i = 0; i < block_size; i++) {
        time_scale[i] = (float)i * Ts;
    }

    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        return 1;
    }

        // Decide GL+GLSL versions
    #if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100
        const char* glsl_version = "#version 100";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    #elif defined(__APPLE__)
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
    #else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
    #endif

    // Create window with graphics context
    glfwWindowHint(GLFW_MAXIMIZED, 1);
    GLFWwindow* window = glfwCreateWindow(
        1280, 720, 
        "QPSK Demodulator Telemetry", 
        NULL, NULL);

    if (window == NULL) {
        return 1;
    }

    glfwSetWindowFocusCallback(window, glfw_window_focus_callback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
    //io.ConfigViewportsNoAutoMerge = true;
    //io.ConfigViewportsNoTaskBarIcon = true;

    // Setup Dear ImGui style
    // ImGui::StyleColorsDark();
    ImGui::StyleColorsLight();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
    ImGuiSetupCustomConfig();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);
    io.Fonts->AddFontFromFileTTF("res/Roboto-Regular.ttf", 15.0f);
    {
        static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        ImFontConfig icons_config; 
        icons_config.MergeMode = true; 
        icons_config.PixelSnapH = true;
        io.Fonts->AddFontFromFileTTF("res/font_awesome.ttf", 16.0f, &icons_config, icons_ranges);
    }

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    double x_min = 0.0f;
    double x_max = (double)block_size;
    double iq_stream_y_min = -1.25f;
    double iq_stream_y_max =  1.25f;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        if (!is_main_window_focused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();

        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

        ImGui::Begin("Telemetry");
        {
        auto dockspace_id = ImGui::GetID("Telemetry dockspace");
        ImGui::DockSpace(dockspace_id);
        }
        ImGui::End();

        ImGui::Begin("Audio Buffer");
        if (ImPlot::BeginPlot("##Audio buffer")) {
            auto buf = app->audio_processor->GetInputBuffer();
            const auto length = app->audio_processor->GetBufferLength();
            ImPlot::SetupAxisLimits(ImAxis_Y1, 9, 256, ImPlotCond_Once);
            ImPlot::PlotLine("Audio", buf, length);
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("PCM 16Bit Buffer");
        if (ImPlot::BeginPlot("##Audio buffer")) {
            static double y0 = static_cast<double>(INT16_MAX);
            static double y1 = static_cast<double>(INT16_MIN);
            auto buf = app->audio_processor->GetOutputBuffer();
            const auto length = app->audio_processor->GetBufferLength();
            ImPlot::SetupAxisLimits(ImAxis_Y1, y1, y0, ImPlotCond_Once);
            ImPlot::PlotLine("Audio", buf, length);
            ImPlot::DragLineY(0, &y0, ImVec4(1,0,0,1), 1);
            ImPlot::DragLineY(1, &y1, ImVec4(1,0,0,1), 1);
            ImPlot::EndPlot();
        }
        ImGui::End();


        if (ImGui::Begin("Controls")) {
            const bool is_rendering_snapshot = (render_buffer == snapshot_buffer);
            if (!is_rendering_snapshot) {
                if (ImGui::Button("Snapshot")) {
                    app->controls.snapshot = true;
                    render_buffer = snapshot_buffer;
                }
            } else {
                if (ImGui::Button("Resume")) {
                    render_buffer = carrier_demod_buffer;
                }
            }

            // ImGui::SliderFloat("Symbol frequency scaler", &demod.ted_clock.fcenter_factor, 0.0f, 2.0f);
            // ImGui::SliderFloat("Carrier frequency offset", &demod.pll_mixer.fcenter, -10000.0f, 10000.0f);
            auto& audio_gain = app->audio_processor->output_gain;
            ImGui::SliderScalar("Audio gain", ImGuiDataType_S16, &audio_gain, &audio_gain_min, &audio_gain_max);
            ImGui::End();
        }

        if (ImGui::Begin("Statistics")) {
            auto& stats = app->audio_frame_handler->stats;
            ImGui::Text("Received=%d\n", stats.total);
            ImGui::Text("Correct=%d\n", stats.correct);
            ImGui::Text("Incorrect=%d\n", stats.incorrect);
            ImGui::Text("Corrupted=%d\n", stats.corrupted);
            ImGui::Text("Repaired=%d\n", stats.repaired);
            ImGui::Text("Packet error rate=%.2f%%\n", stats.GetPacketErrorRate()*100.0f);
            ImGui::Text("Packet repair rate=%.2f%%\n", stats.GetRepairFailureRate()*100.0f);

            if (ImGui::Button("Reset")) {
                stats.reset();
            }
            ImGui::End();
        }


        ImGui::Begin("Constellation");
        if (ImPlot::BeginPlot("##Constellation", ImVec2(-1,0), ImPlotFlags_Equal)) {
            ImPlot::SetupAxisLimits(ImAxis_X1, -2, 2, ImPlotCond_Once);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -2, 2, ImPlotCond_Once);
            const float marker_size = 3.0f;
            {
                auto buffer = reinterpret_cast<float*>(render_buffer->y_sym_out);
                ImPlot::SetNextMarkerStyle(0, marker_size);
                ImPlot::PlotScatter("IQ demod", &buffer[0], &buffer[1], block_size, 0, 0, 2*sizeof(float));
            }
            {
                auto buffer = reinterpret_cast<float*>(render_buffer->x_pll_out);
                ImPlot::HideNextItem(true, ImPlotCond_Once);
                ImPlot::SetNextMarkerStyle(0, marker_size);
                ImPlot::PlotScatter("IQ raw", &buffer[0], &buffer[1], block_size, 0, 0, 2*sizeof(float));
            }
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("IQ signals");
        if (ImPlot::BeginPlot("Symbol out")) {
            auto buffer = reinterpret_cast<float*>(render_buffer->y_sym_out);
            ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
            ImPlot::SetupAxisLinks(ImAxis_Y1, &iq_stream_y_min, &iq_stream_y_max);
            ImPlot::PlotLine("I", &buffer[0], block_size, 1.0, 0.0, 0, 0, 2*sizeof(float));
            ImPlot::PlotLine("Q", &buffer[1], block_size, 1.0, 0.0, 0, 0, 2*sizeof(float));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("PLL out")) {
            auto buffer = reinterpret_cast<float*>(render_buffer->x_pll_out);
            ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
            ImPlot::SetupAxisLinks(ImAxis_Y1, &iq_stream_y_min, &iq_stream_y_max);
            ImPlot::PlotLine("I", &buffer[0], block_size, 1.0, 0.0, 0, 0, 2*sizeof(float));
            ImPlot::PlotLine("Q", &buffer[1], block_size, 1.0, 0.0, 0, 0, 2*sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Errors");
        if (ImPlot::BeginPlot("##Errors")) {
            ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
            ImPlot::PlotLine("PLL error", render_buffer->error_pll, block_size);
            ImPlot::PlotLine("TED error", render_buffer->error_ted, block_size);
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Triggers");
        if (ImPlot::BeginPlot("##Triggers")) {
            ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -0.2, 1.5, ImPlotCond_Once);
            ImPlot::PlotStems("Zero crossing", (uint8_t*)render_buffer->trig_zero_crossing, block_size);
            ImPlot::PlotStems("Ramp oscillator", (uint8_t*)render_buffer->trig_ted_clock, block_size);
            ImPlot::PlotStems("Integrate+dump", (uint8_t*)render_buffer->trig_integrator_dump, block_size);
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Spec Builder");
        {
            auto& spec = app->carrier_demod_spec;
            const float A = 100e3;
            const float B = 10e3;
            const float C = 10e3;
            ImGui::SliderInt("Baseband FIR LPF order", &spec.baseband_filter.M, 2, 200);
            ImGui::SliderFloat("Baseband FIR LPF cutoff", &spec.baseband_filter.cutoff, 0.0f, Fsample/(float)ds_factor * 0.5f);
            ImGui::SliderFloat("AC Filter", &spec.ac_filter.k, 0.9999f, 1.0f);
            ImGui::SliderFloat("AGC beta", &spec.agc.beta, 0.0f, 1.0f);
            ImGui::SliderFloat("Carrier PLL Fcenter", &spec.carrier_pll.f_center, -B, B);
            ImGui::SliderFloat("Carrier PLL Fgain", &spec.carrier_pll.f_gain, 0e3, A);
            ImGui::SliderFloat("Carrier PLL Filter Cutoff", &spec.carrier_pll_filter.butterworth_cutoff, 0e3, A);
            ImGui::SliderFloat("Carrier PLL Filter Integrator", &spec.carrier_pll_filter.integrator_gain, 0e3, C);
            ImGui::SliderFloat("TED PLL Fgain", &spec.ted_pll.f_gain, 0e3, A);
            ImGui::SliderFloat("TED PLL Foffset", &spec.ted_pll.f_offset, -A, A);
            ImGui::SliderFloat("TED PLL Filter Cutoff", &spec.ted_pll_filter.butterworth_cutoff, 0e3, A);
            ImGui::SliderFloat("TED PLL Filter Integrator", &spec.ted_pll_filter.integrator_gain, 0e3, C);
            
            if (ImGui::Button("Build")) {
                app->controls.rebuild = true;
            }

            ImGui::SameLine();
            if (ImGui::Button("Revert to default")) {
                app->carrier_demod_spec = original_carrier_demod_spec;
            }
        }
        ImGui::End();

        // Rendering
        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    	
        // Update and Render additional Platform Windows
        // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }


    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    // closing down
    demod_thread.join();
    fclose(fp_in);

    return 0;
}
