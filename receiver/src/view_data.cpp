#include <stdio.h>
#include <thread>
#include <stdlib.h>
#include <stdint.h>

#if defined(_WIN32)
#include <io.h>
#include <fcntl.h>
#endif

#include "app.h"
#include "audio/portaudio_output.h"
#include "audio/resampled_pcm_player.h"
#include "audio/portaudio_utility.h"
#include "utility/getopt/getopt.h"

// GUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <implot.h>
#include "imgui_config.h"
#include "font_awesome_definitions.h"

// Hold all of the data we need to render
struct Renderer {
    // models
    App& app;
    PaDeviceList& pa_device_list;
    PortAudio_Output& pa_audio_output;
    // control state 
    struct AppRenderState {
        QAM_Synchroniser_Specification original_qam_sync_spec;
        int shared_block_size;
        QAM_Synchroniser_Buffer* render_buffer;
        ImPlotRange xrange_audio_buffer;
        ImPlotRange xrange_dsp_buffers;
        ImPlotRange yrange_input_buffer;        // baseband input
        ImPlotRange yrange_ds_input_buffer;     // AGC normalises to -1...+1
    } app_render_state;

    Renderer(App& _app, PaDeviceList& _pa_device_list, PortAudio_Output& _pa_audio_output)
    : app(_app), pa_device_list(_pa_device_list), pa_audio_output(_pa_audio_output)
    {
        {
            auto& s = app_render_state;
            const int shared_block_size = app.GetActiveBuffer().GetInputSize();
            const double audio_block_size = (double)app.GetAudioFilter().GetOutputBufferSize();
            s.original_qam_sync_spec = app.qam_sync_spec;
            s.shared_block_size = shared_block_size;
            s.render_buffer = &(app.GetActiveBuffer());
            s.xrange_audio_buffer = {0, audio_block_size};
            s.xrange_dsp_buffers = {0, (double)shared_block_size};
            s.yrange_input_buffer = {-128, 128};
            s.yrange_ds_input_buffer = {-1.25, 1.25};
        }
    }
};

int RenderAll(Renderer& renderer);
void RenderApp(App& app, Renderer::AppRenderState& state);
void RenderPortAudioControls(PaDeviceList& device_list, PortAudio_Output& audio_output);

void usage() {
    fprintf(stderr, 
        "view_data, runs 16QAM demodulation on raw IQ values with GUI\n\n"
        "\t[-f sample rate (default: 1MHz)]\n"
        "\t[-s symbol rate (default: 200kHz)]\n"
        "\t[-b intermediate block size (default: 1024)]\n"
        "\t[-D downsample factor (default: 2)]\n"
        "\t[-S upsample factor (default: 4)]\n"
        "\t    rd_block_size = D*block_size\n"
        "\t    us_block_size = S*block_size\n"
        "\t    rd_block_size -> block_size -> us_block_size\n"
        "\t[-i input filename (default: None)]\n"
        "\t    If no file is provided then stdin is used\n"
        "\t[-A disable audio output (default: true)]\n"
        "\t[-h (show usage)]\n"
    );
}

int main(int argc, char** argv)
{
    int ds_factor = 2;
    int us_factor = 4;
    int demod_block_size = 1024;
    float Fsample = 1e6; 
    float Fsymbol = 200e3;

    char* rd_filename = NULL;

    // audio stream is symbol_rate / N
    const char audio_packet_sampling_ratio = 5;
    bool is_output_audio = true;

    int opt; 
    while ((opt = getopt_custom(argc, argv, "f:s:b:D:S:i:Ah")) != -1) {
        switch (opt) {
        case 'f':
            Fsample = (float)(atof(optarg));
            if (Fsample <= 0) {
                fprintf(stderr, "Sampling rate must be positive (%.2f)\n", Fsample); 
                return 1;
            }
            break;
        case 's':
            Fsymbol = (float)(atof(optarg));
            if (Fsymbol <= 0) {
                fprintf(stderr, "Symbol rate must be positive (%.2f)\n", Fsymbol); 
                return 1;
            }
            break;
        case 'b':
            demod_block_size = (int)(atof(optarg));
            if (demod_block_size <= 0) {
                fprintf(stderr, "Block size must be positive (%d)\n", demod_block_size); 
                return 1;
            }
            break;
        case 'D':
            ds_factor = (int)(atof(optarg));
            if (ds_factor <= 0) {
                fprintf(stderr, "Downsampling factor must be positive (%d)\n", ds_factor); 
                return 1;
            }
            break;
        case 'S':
            us_factor = (int)(atof(optarg));
            if (us_factor <= 0) {
                fprintf(stderr, "Upsampling factor must be positive (%d)\n", us_factor); 
                return 1;
            }
            break;
        case 'i':
            rd_filename = optarg;
            break;
        case 'A':
            is_output_audio = false;
            break;
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    // app startup
    FILE* fp_in = stdin;
    if (rd_filename != NULL) {
        errno_t err = fopen_s(&fp_in, rd_filename, "r");
        if (err != 0) {
            LOG_MESSAGE("Failed to open file for reading\n");
            return 1;
        }
    }

#if defined(_WIN32)
    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    _setmode(_fileno(fp_in), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    const float Faudio = Fsymbol/(float)audio_packet_sampling_ratio;
    const int audio_buffer_size = (int)Faudio;
    const int decoder_buffer_size = 1024;

    auto app = App(
        fp_in, demod_block_size, 
        decoder_buffer_size, ds_factor, us_factor, 
        audio_buffer_size, Faudio);

    {
        const float PI = 3.1415f;
        auto& spec = app.qam_sync_spec;
        spec.f_sample = Fsample; 
        spec.f_symbol = Fsymbol;
        
        spec.downsampling_filter.M = ds_factor;
        spec.downsampling_filter.K = 6;

        spec.upsampling_filter.L = us_factor;
        spec.upsampling_filter.K = 6;

        spec.ac_filter.k = 0.99999f;
        spec.agc.beta = 0.2f;
        spec.agc.initial_gain = 0.1f;
        spec.carrier_pll.f_center = 0e3;
        spec.carrier_pll.f_gain = 2.5e3;
        spec.carrier_pll.phase_error_gain = 8.0f/PI;
        spec.carrier_pll_filter.butterworth_cutoff = 5e3;
        spec.carrier_pll_filter.integrator_gain = 1000.0f;
        spec.ted_pll.f_gain = 30e3;
        spec.ted_pll.f_offset = 0e3;
        spec.ted_pll.phase_error_gain = 1.0f;
        spec.ted_pll_filter.butterworth_cutoff = 60e3;
        spec.ted_pll_filter.integrator_gain = 250.0f;
    }

    app.BuildDemodulator();
    app.GetFrameHandler().is_output_audio = is_output_audio;

    // Setup audio
    auto pa_handler = ScopedPaHandler();
    PaDeviceList pa_devices;
    PortAudio_Output pa_output;
    std::unique_ptr<Resampled_PCM_Player> pcm_player;
    {
        auto& mixer = pa_output.GetMixer();
        auto buf = mixer.CreateManagedBuffer(4);
        auto Fs = pa_output.GetSampleRate();
        pcm_player = std::make_unique<Resampled_PCM_Player>(buf, Fs);

        #ifdef _WIN32
        const auto target_host_api_index = Pa_HostApiTypeIdToHostApiIndex(PORTAUDIO_TARGET_HOST_API_ID);
        const auto target_device_index = Pa_GetHostApiInfo(target_host_api_index)->defaultOutputDevice;
        pa_output.Open(target_device_index);
        #else
        pa_output.Open(Pa_GetDefaultOutputDevice());
        #endif
    }

    app.GetAudioFilter().OnOutputBlock().Attach([&pcm_player, Faudio](tcb::span<const Frame<float>> data) {
        pcm_player->SetInputSampleRate((int)Faudio);
        pcm_player->ConsumeBuffer(data);
    });

    auto demod_thread = std::thread([&app]() {
        app.Run();
    });

    auto renderer = Renderer(app, pa_devices, pa_output);
    const auto rv = RenderAll(renderer);
    app.Stop();
    demod_thread.join();
    fclose(fp_in);

    return rv;
}

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


int RenderAll(Renderer& renderer) {
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

        if (ImGui::Begin("Telemetry")) {
            auto dockspace_id = ImGui::GetID("Telemetry dockspace");
            ImGui::DockSpace(dockspace_id);
        }
        ImGui::End();

        RenderApp(renderer.app, renderer.app_render_state);

        if (ImGui::Begin("Audio Controls")) {
            RenderPortAudioControls(renderer.pa_device_list, renderer.pa_audio_output);
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

    return 0;
}

void RenderApp(App& app, Renderer::AppRenderState& state) {
    // swap between live and snapshot buffer
    auto* active_buffer = &(app.GetActiveBuffer());
    auto* snapshot_buffer = &(app.GetSnapshotBuffer());

    auto get_xscale = [&state](const int N) -> double {
        const double x = (double)state.shared_block_size / (double)N;
        return x;
    };

    ImGui::Begin("PCM 16Bit Buffer");
    if (ImPlot::BeginPlot("##Audio buffer")) {
        auto& audio_filter = app.GetAudioFilter();
        auto buf = audio_filter.GetOutputBuffer();
        const int N = (int)buf.size();
        auto* data = reinterpret_cast<const float*>(buf.data());

        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_audio_buffer.Min, &state.xrange_audio_buffer.Max);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0f, 1.0f, ImPlotCond_Once);
        ImPlot::PlotLine("Left", &data[0], N, 1, 0, 0, 0, 2*sizeof(float));
        ImPlot::PlotLine("Right", &data[1], N, 1, 0, 0, 0, 2*sizeof(float));
        ImPlot::EndPlot();
    }
    ImGui::End();

    if (ImGui::Begin("Controls")) {
        const bool is_rendering_snapshot = (state.render_buffer == snapshot_buffer);
        if (!is_rendering_snapshot) {
            if (ImGui::Button("Snapshot")) {
                app.controls.snapshot = true;
                state.render_buffer = snapshot_buffer;
            }
        } else {
            if (ImGui::Button("Resume")) {
                state.render_buffer = active_buffer;
            }
        }

        auto& audio_filter = app.GetAudioFilter();
        auto& frame_handler = app.GetFrameHandler();
        ImGui::Checkbox("Output Audio", &(frame_handler.is_output_audio));
        ImGui::Checkbox("Output Data", &(frame_handler.is_output_data));
        ImGui::End();
    }

    if (ImGui::Begin("Statistics")) {
        auto& frame_handler = app.GetFrameHandler();
        auto& stats = frame_handler.stats;
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
            const auto buf = state.render_buffer->y_sym_out;
            const int N = (int)buf.size();
            auto* data = reinterpret_cast<float*>(buf.data());
            ImPlot::SetNextMarkerStyle(0, marker_size);
            ImPlot::PlotScatter("IQ demod", &data[0], &data[1], N, 0, 0, 2*sizeof(float));
        }
        {
            const auto buf = state.render_buffer->x_pll_out;
            const int N = (int)buf.size();
            auto* data = reinterpret_cast<float*>(buf.data());
            ImPlot::HideNextItem(true, ImPlotCond_Once);
            ImPlot::SetNextMarkerStyle(0, marker_size);
            ImPlot::PlotScatter("IQ raw", &data[0], &data[1], N, 0, 0, 2*sizeof(float));
        }
        ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Symbol out");
    if (ImPlot::BeginPlot("Symbol out")) {
        auto buf = state.render_buffer->y_sym_out;
        const int N = (int)buf.size();
        auto* data = reinterpret_cast<float*>(buf.data());
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::SetupAxisLinks(ImAxis_Y1, &state.yrange_ds_input_buffer.Min, &state.yrange_ds_input_buffer.Max);
        ImPlot::PlotLine("I", &data[0], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::PlotLine("Q", &data[1], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("PLL out")) {
        auto buf = state.render_buffer->x_pll_out;
        const int N = (int)buf.size();
        auto* data = reinterpret_cast<float*>(buf.data());
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::SetupAxisLinks(ImAxis_Y1, &state.yrange_ds_input_buffer.Min, &state.yrange_ds_input_buffer.Max);
        ImPlot::PlotLine("I", &data[0], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::PlotLine("Q", &data[1], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Upsampled")) {
        auto buf = state.render_buffer->x_upsampled;
        const int N = (int)buf.size();
        auto* data = reinterpret_cast<float*>(buf.data());
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::SetupAxisLinks(ImAxis_Y1, &state.yrange_ds_input_buffer.Min, &state.yrange_ds_input_buffer.Max);
        ImPlot::PlotLine("I", &data[0], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::PlotLine("Q", &data[1], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Raw signals");
    if (ImPlot::BeginPlot("Raw Signal")) {
        auto buf = state.render_buffer->x_in;
        const int N = (int)buf.size();
        auto* data = reinterpret_cast<float*>(buf.data());
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::SetupAxisLinks(ImAxis_Y1, &state.yrange_input_buffer.Min, &state.yrange_input_buffer.Max);
        ImPlot::PlotLine("I", &data[0], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::PlotLine("Q", &data[1], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Downsampled signal")) {
        auto buf = state.render_buffer->x_downsampled;
        const int N = (int)buf.size();
        auto* data = reinterpret_cast<float*>(buf.data());
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::SetupAxisLinks(ImAxis_Y1, &state.yrange_input_buffer.Min, &state.yrange_input_buffer.Max);
        ImPlot::PlotLine("I", &data[0], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::PlotLine("Q", &data[1], N, get_xscale(N), 0.0, 0, 0, 2*sizeof(float));
        ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Errors");
    if (ImPlot::BeginPlot("##Errors")) {
        auto buf0 = state.render_buffer->error_pll;
        auto buf1 = state.render_buffer->error_ted;
        const int N0 = (int)buf0.size();
        const int N1 = (int)buf1.size();
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::PlotLine("PLL error", buf0.data(), N0, get_xscale(N0));
        ImPlot::PlotLine("TED error", buf1.data(), N1, get_xscale(N1));
        ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Triggers");
    if (ImPlot::BeginPlot("##Triggers")) {
        auto buf0 = state.render_buffer->trig_zero_crossing;
        auto buf1 = state.render_buffer->trig_ted_clock;
        auto buf2 = state.render_buffer->trig_integrator_dump;
        const int N0 = (int)buf0.size();
        const int N1 = (int)buf1.size();
        const int N2 = (int)buf2.size();
        ImPlot::SetupAxisLinks(ImAxis_X1, &state.xrange_dsp_buffers.Min, &state.xrange_dsp_buffers.Max);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -0.2, 1.5, ImPlotCond_Once);
        ImPlot::PlotStems("Zero crossing", (uint8_t*)buf0.data(), N0, 0.0f, get_xscale(N0));
        ImPlot::PlotStems("Ramp oscillator", (uint8_t*)buf1.data(), N1, 0.0f, get_xscale(N1));
        ImPlot::PlotStems("Integrate+dump", (uint8_t*)buf2.data(), N2, 0.0f, get_xscale(N2));
        ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Spec Builder");
    {
        auto& spec = app.qam_sync_spec;
        const float A = 100e3;
        const float B = 10e3;
        const float C = 10e3;
        ImGui::SliderInt("Downsampling filter size", &spec.downsampling_filter.K, 2, 20);
        ImGui::SliderInt("Upsampling filter size", &spec.upsampling_filter.K, 2, 20);
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
            app.controls.rebuild = true;
        }

        ImGui::SameLine();
        if (ImGui::Button("Revert to default")) {
            app.qam_sync_spec = state.original_qam_sync_spec;
        }
    }
    ImGui::End();
}

void RenderPortAudioControls(PaDeviceList& device_list, PortAudio_Output& audio_output) {
    auto& devices = device_list.GetDevices();

    const auto selected_index = audio_output.GetSelectedIndex();
    const char* selected_name = "Unselected";
    for (auto& device: devices) {
        if (device.index == selected_index) {
            selected_name = device.label.c_str();
            break;
        }
    }

    ImGui::Text("Output Devices (%d)", (int)devices.size());
    ImGui::PushItemWidth(-1.0f);
    if (ImGui::BeginCombo("###Output Devices", selected_name, ImGuiComboFlags_None)) {
        for (auto& device: devices) {
            const bool is_selected = (device.index == selected_index);
            ImGui::PushID(device.index);
            if (ImGui::Selectable(device.label.c_str(), is_selected)) {
                if (!is_selected) {
                    audio_output.Open(device.index);
                }
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::PopItemWidth();

    auto& mixer = audio_output.GetMixer();
    auto& volume_gain = mixer.GetOutputGain();

    static bool is_overgain = false;
    static float last_unmuted_volume = 0.0f;

    bool is_muted = (volume_gain == 0.0f);
    const float max_gain = is_overgain ? 6.0f : 2.0f;
    if (!is_overgain) {
        volume_gain = (volume_gain > max_gain) ? max_gain : volume_gain;
    }

    ImGui::PushItemWidth(-1.0f);
    ImGui::Text("Volume");

    const float volume_scale = 100.0f;
    float curr_volume = volume_gain * volume_scale;
    if (ImGui::SliderFloat("###Volume", &curr_volume, 0.0f, max_gain*volume_scale, "%.0f", ImGuiSliderFlags_AlwaysClamp)) {
        volume_gain = (curr_volume / volume_scale);
        if (volume_gain > 0.0f) {
            last_unmuted_volume = volume_gain;
        } else {
            last_unmuted_volume = 1.0f;
        }
    }
    ImGui::PopItemWidth();

    if (is_muted) {
        if (ImGui::Button("Unmute")) {
            volume_gain = last_unmuted_volume;
        }
    } else {
        if (ImGui::Button("Mute")) {
            last_unmuted_volume = volume_gain;
            volume_gain = 0.0f;
        }
    }

    ImGui::SameLine();

    if (ImGui::Button(is_overgain ? "Normal gain" : "Boost gain")) {
        is_overgain = !is_overgain;
    }
}