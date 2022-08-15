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

#include "carrier_dsp.h"
#include "frame_synchroniser.h"

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

static void glfw_error_callback(int error, const char* description)
{
    LOG_MESSAGE("Glfw Error %d: %s\n", error, description);
}

void demodulator_thread(
    FILE* fp, CarrierToSymbolDemodulator* demod, const int block_size,
    CarrierToSymbolDemodulatorBuffers* snapshot_buffer, bool* snapshot_trigger,
    uint8_t* audio_buffer, const int audio_buffer_size,
    int* audio_gain) 
{
    auto x_buffer = new std::complex<uint8_t>[block_size];
    auto y_buffer = new std::complex<float>[block_size];

    constexpr uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
    constexpr uint16_t SCRAMBLER_CODE = 0b1000010101011001;
    // constexpr uint32_t CRC32_POLY = 0x04C11DB7;
    constexpr uint8_t CRC8_POLY = 0xD5;
    // auto frame_sync = FrameSynchroniser<uint32_t>(PREAMBLE_CODE, SCRAMBLER_CODE, CRC32_POLY);
    auto frame_sync = FrameSynchroniser<uint32_t>(PREAMBLE_CODE, SCRAMBLER_CODE, CRC8_POLY, audio_buffer_size);

    int curr_audio_buffer_index = 0;

    float* audio_filter_buffer = new float[audio_buffer_size];
    uint16_t* pcm_buffer = new uint16_t[audio_buffer_size];

    auto payload_handler = [&pcm_buffer, &audio_buffer, &curr_audio_buffer_index, audio_buffer_size, &audio_gain](uint8_t* x, const uint16_t N) {
        // if not an audio block
        if (N != 100) {
            LOG_MESSAGE("message=%.*s\n", N, x);
            return;
        }

        for (int i = 0; i < N; i++) {
            const uint8_t v = x[i];
            audio_buffer[curr_audio_buffer_index] = v;
            curr_audio_buffer_index = (curr_audio_buffer_index+1) % audio_buffer_size;


            // amplify the signal
            int16_t v0 = static_cast<int16_t>(v);
            v0 = v0-127;
            v0 = v0 * (*audio_gain);
            v0 = v0 + (1u << 8);
            uint16_t v1 = (uint16_t)(v0);
            v1 = v1 * 2;
            pcm_buffer[i] = (uint16_t)(v1);
        }
        fwrite(pcm_buffer, sizeof(uint16_t), N, stdout);
    };

    size_t rd_block_size = 0;
    int rd_total_blocks = 0;
    while ((rd_block_size = fread(x_buffer, 2*sizeof(uint8_t), block_size, fp)) > 0) {
        if (rd_block_size != block_size) {
            LOG_MESSAGE("Got mismatched block size after %d blocks\n", rd_total_blocks);
            continue;
        }
        rd_total_blocks++; 
        const auto total_symbols = demod->ProcessBlock(x_buffer, y_buffer);
        for (int i = 0; i < total_symbols; i++) {
            const auto IQ = y_buffer[i];
            const auto res = frame_sync.process(IQ);
            using Res = FrameSynchroniser<uint32_t>::ProcessResult;
            if (res == Res::PAYLOAD_OK) {
                auto& p = frame_sync.payload;
                payload_handler(p.buf, p.length);
            }
        }

        if (*snapshot_trigger) {
            snapshot_buffer->CopyFrom(demod->buffers);
            *snapshot_trigger = false;
        }
    }
    
    delete [] x_buffer;
    delete [] y_buffer;
    delete [] pcm_buffer;
    delete [] audio_filter_buffer;
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

    // const int block_size = 4096;
    const int block_size = 8192;
    CarrierToSymbolDemodulator demod(block_size);
    demod.pll_mixer.fcenter = -1000; // NOTE: we are doing this because our loop filter doesnt have an integral term

    // swap between live and snapshot buffer
    auto demod_buffer = demod.buffers;
    auto snapshot_buffer = new CarrierToSymbolDemodulatorBuffers(block_size);
    bool snapshot_trigger = false;

    auto render_buffer = demod_buffer;

    const int audio_buffer_size = 10000;
    uint8_t audio_buffer[audio_buffer_size] = {0};
    int audio_gain = 16;
    
    auto demod_thread = std::thread(
        demodulator_thread, 
        fp_in, &demod, block_size, 
        snapshot_buffer, &snapshot_trigger,
        audio_buffer, audio_buffer_size, &audio_gain);

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
            ImPlot::SetupAxisLimits(ImAxis_Y1, 9, 256, ImPlotCond_Once);
            ImPlot::PlotLine("Audio", audio_buffer, audio_buffer_size);
            ImPlot::EndPlot();
        }
        ImGui::End();


        if (ImGui::Begin("Controls")) {
            const bool is_rendering_snapshot = (render_buffer == snapshot_buffer);
            if (!is_rendering_snapshot) {
                if (ImGui::Button("Snapshot")) {
                    snapshot_trigger = true;
                    render_buffer = snapshot_buffer;
                }
            } else {
                if (ImGui::Button("Resume")) {
                    render_buffer = demod_buffer;
                }
            }

            ImGui::SliderFloat("Symbol frequency scaler", &demod.ted_clock.fcenter_factor, 0.0f, 2.0f);
            ImGui::SliderFloat("Carrier frequency offset", &demod.pll_mixer.fcenter, -10000.0f, 10000.0f);
            ImGui::SliderInt("Audio gain", &audio_gain, 0, 128);
            ImGui::End();
        }


        ImGui::Begin("Constellation");
        if (ImPlot::BeginPlot("##Constellation", ImVec2(-1,0), ImPlotFlags_Equal)) {
            ImPlot::SetupAxisLimits(ImAxis_X1, -15, 15, ImPlotCond_Once);
            {
                auto buffer = reinterpret_cast<float*>(render_buffer->y_sym_out);
                ImPlot::PlotScatter("IQ demod", &buffer[0], &buffer[1], block_size, 0, 0, 2*sizeof(float));
            }
            {
                auto buffer = reinterpret_cast<float*>(render_buffer->x_pll_out);
                ImPlot::HideNextItem(true, ImPlotCond_Once);
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

    delete snapshot_buffer;

    return 0;
}
