// Dear ImGui: standalone example application for DirectX 9

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp

#include "../include/gui.h"
#include "../include/TCPSocket.h"

#include "imgui.h"
#include "imgui_impl_dx9.h"
#include "imgui_impl_win32.h"
#include <d3d9.h>
#include <tchar.h>

#include <iostream>
#include <vector>
#include <string>
#include <Windows.h>
#include <Commdlg.h> // Common dialog box library
#include <fstream>
#include <thread>
#include "Message.hpp"

UpdateVariables updVars;

TCPSocket* _socket = new TCPSocket(&updVars);

// Data
static LPDIRECT3D9              g_pD3D = nullptr;
static LPDIRECT3DDEVICE9        g_pd3dDevice = nullptr;
static UINT                     g_ResizeWidth = 0, g_ResizeHeight = 0;
static D3DPRESENT_PARAMETERS    g_d3dpp = {};

// Forward declarations of helper functions
bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void ResetDevice();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Networking
bool _connecting = false;
bool _connected = false;

// Debug
std::vector<std::string> logs;
bool _manualInput = false;
bool _droneArmed = false;

// Textfields
// CONNECTION
char addrBuffer[255]{};
char portBuffer[255]{};

// SEND RPY (MANUAL INPUT)
char rollBuffer[255]{};
char pitchBuffer[255]{};
char yawBuffer[255]{};

// SEND PID (MANUAL INPUT)
char propBuffer[255]{};
char integBuffer[255]{};
char diffBuffer[255]{};
char typeBuffer[255]{};

// SEND RPYT (MANUAL INPUT)
char rpytRollBuffer[255]{};
char rpytPitchBuffer[255]{};
char rpytYawBuffer[255]{};
char rpytThrustBuffer[255]{};
char rpytFlagBuffer[255]{};
char rpytFilenameBuffer[255]{};

// USERDEFINED HOVER HEIGHT (MANUAL INPUT)
char hoverheightBuffer[255]{};
char hoverFileNameBuffer[255]{};

int state_;

// Function to save data to a file
bool SaveDataToFile(const std::wstring& filePath) {
    std::ofstream outputFile(filePath, std::ios::out | std::ios::binary);
    if (!outputFile) {
        // Failed to open the file for writing
        return false;
    }

    // Write each string from the vector to the file
    for (const auto& log : logs) {
        outputFile << log;
    }

    // Close the file
    outputFile.close();

    return true;
}

// Function to open a save file dialog
bool SaveFileDialog(HWND hwnd, LPWSTR filePath, LPCWSTR defaultExtension, LPCWSTR fileFilter) {
    OPENFILENAME ofn;
    ZeroMemory(&ofn, sizeof(OPENFILENAME));
    ofn.lStructSize = sizeof(OPENFILENAME);
    ofn.hwndOwner = hwnd;
    ofn.lpstrFile = filePath;
    ofn.lpstrFile[0] = '\0';
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter = fileFilter;
    ofn.nFilterIndex = 1;
    ofn.lpstrDefExt = defaultExtension;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

    // Display the save file dialog
    return GetSaveFileName(&ofn);
}

void log(std::string text, std::string prefix) {
    std::cout << "[" << prefix << "] " << text << std::endl;
    logs.push_back("[" + prefix + "] " + text);
}

void updateProps(UpdateVariables* updVars, UpdateMessage* msg)
{
    updVars->roll = msg->roll;
    updVars->pitch = msg->pitch;
    updVars->yaw = msg->yaw;
    updVars->thrust = msg->thrust;
    updVars->lat = msg->lat;
    updVars->lon = msg->lon;
    updVars->alt = msg->alt;
    updVars->state = msg->state;
}
std::string OpenFileDialog(HWND hWnd, LPWSTR filePath, LPCWSTR defaultExtension, LPCWSTR fileFilter) {
    OPENFILENAME ofn;       // common dialog box structure

    // Initialize OPENFILENAME
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hWnd;
    ofn.lpstrFile = filePath;
    ofn.lpstrFile[0] = '\0';
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter = fileFilter; // "All Files\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = NULL;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = NULL;
    ofn.lpstrDefExt = defaultExtension;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

    // Display the Open dialog box
    if (GetOpenFileName(&ofn) == TRUE) {
        // Convert wide-character string to narrow-character string
        int bufferSize = WideCharToMultiByte(CP_UTF8, 0, ofn.lpstrFile, -1, NULL, 0, NULL, NULL);
        std::string filePath(bufferSize, '\0');
        WideCharToMultiByte(CP_UTF8, 0, ofn.lpstrFile, -1, &filePath[0], bufferSize, NULL, NULL);
        return filePath;
    }
    else {
        return ""; // User canceled the dialog
    }
}

void makeManualInputWindow()
{
    ImGui::Begin("Send Manual Inputs");

    ImGui::Text("Send PID Message:");
    ImGui::PushItemWidth(50);
    ImGui::InputText("kp", propBuffer, sizeof(propBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("ki", integBuffer, sizeof(integBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("kd", diffBuffer, sizeof(diffBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("type", typeBuffer, sizeof(typeBuffer));
    ImGui::SameLine();
    if (ImGui::Button("Send PID")) {
        SetPIDMessage msg;
        // TODO: This should be floats and not strings
        msg.kp = propBuffer;//std::stof(propBuffer);
        msg.ki = integBuffer;// std::stof(integBuffer);
        msg.kd = diffBuffer;// std::stof(diffBuffer);
        msg.flag = std::stoi(typeBuffer);
        _socket->Send(msg);
    }

    ImGui::Text("Send RPYFT Message:");
    ImGui::PushItemWidth(50);
    ImGui::InputText("roll", rpytRollBuffer, sizeof(rpytRollBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("pitch", rpytPitchBuffer, sizeof(rpytPitchBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("yaw", rpytYawBuffer, sizeof(rpytYawBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("thrust", rpytThrustBuffer, sizeof(rpytThrustBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(50);
    ImGui::InputText("flag", rpytFlagBuffer, sizeof(rpytFlagBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::InputText("csv filename", rpytFilenameBuffer, sizeof(rpytFilenameBuffer));
    ImGui::SameLine();
    if (ImGui::Button("Send RPYTFF")) {
        try {
            std::string fileName{ "ignore_null" };
            fileName = rpytFilenameBuffer;

            SetRPYTFFMessage msg;
            msg.roll = std::stof(rpytRollBuffer);
            msg.pitch = std::stof(rpytPitchBuffer);
            msg.yaw = std::stof(rpytYawBuffer);
            msg.thrust = std::stof(rpytThrustBuffer);
            msg.flag = std::stoi(rpytFlagBuffer);
            msg.fileName = fileName;
            _socket->Send(msg);
            log("StartTestMessage sent");
        }
        catch (const std::invalid_argument& e) {
            log("Send RPYFT: Invalid arguments!", "ERROR");
        }
    }

    ImGui::Text("Userdefined Hover Height");
    ImGui::PushItemWidth(50);
    ImGui::InputText("height", hoverheightBuffer, sizeof(hoverheightBuffer));
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::InputText("filename", hoverFileNameBuffer, sizeof(hoverFileNameBuffer));
    ImGui::SameLine();
    if (ImGui::Button("Send Height")) {
        std::string fileName{ "hover_test" };
        fileName = hoverFileNameBuffer;

        SetHoverHeightMessage msg;
        msg.height = std::stof(hoverheightBuffer);
        msg.fileName = fileName;
        _socket->Send(msg);
        log("Hover height has been succesfully set!");
    }

    if (ImGui::Button("Close")) {
        _manualInput = false;
    }

    ImGui::End();
}

void makeConnectWindow() {
    // ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Connect to Drone");

    std::string addrText{ "127.0.0.1" };
    std::string portText{ "8888" };
    ImGui::InputText("Address", addrBuffer, sizeof(addrBuffer));
    ImGui::InputText("Port", portBuffer, sizeof(portBuffer));

    ImGui::Text("Default connection: 127.0.0.1:8888");

    if (ImGui::Button("Connect")) {
        addrText = addrBuffer;
        portText = portBuffer;
        std::cout << addrText << ":" << portText << std::endl;

        bool connected = _socket->Connect(addrText, std::stoi(portText));
        if (connected) {
            _connecting = false;
            _connected = true;
        }
    }
    if (ImGui::Button("Cancel")) {
        _connecting = false;
    }

    ImGui::End();
}

void makeLogPanel() {
    // Begin Console Panel (Bottom Left)
    ImGui::SetNextWindowPos(ImVec2(0, 619), ImGuiCond_Always);
    ImGui::BeginChild("Console", ImVec2(750, 200), true, ImGuiWindowFlags_None);
    ImGui::BeginChild("LogOutput", ImVec2(-1, -ImGui::GetFrameHeightWithSpacing()), false, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::TextUnformatted("Log Output");
    ImGui::Separator();

    // Display logs
    for (const auto& log : logs) {
        ImGui::TextWrapped("%s", log.c_str());
    }

    // Ensure scroll is at bottom to show latest logs
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::EndChild();
}

void makePropsPanel() {
    // Begin Properties Panel (Bottom Right)
    ImGui::SetNextWindowPos(ImVec2(1000, 319), ImGuiCond_Always);
    ImGui::BeginChild("Properties", ImVec2(300, 500), true, ImGuiWindowFlags_None);
    ImGui::TextUnformatted("Properties");
    ImGui::Separator();

    ImVec4 connStrColor = _connected ? ImVec4(0, 1.0f, 0, 1.0f) : ImVec4(1.0f, 0, 0, 1.0f);
    const char* connStr = _connected ? "Connected" : "Not Connected";
    ImGui::Text("Connection Status: ");
    ImGui::SameLine();
    ImGui::TextColored(connStrColor, connStr);

    ImGui::Text("Camera Feed FPS: %d", 0);
    ImGui::Text("Drone Current Time: %s", "0");
    ImGui::Text("Drone Roll: %f", updVars.roll);
    ImGui::Text("Drone Pitch: %f", updVars.pitch);
    ImGui::Text("Drone Yaw: %f", updVars.yaw);
    ImGui::Text("Drone Thrust: %f", updVars.thrust);
    ImGui::Text("Drone Latitude: %f", updVars.lat);
    ImGui::Text("Drone Longitude: %f", updVars.lon);
    ImGui::Text("Drone Altitude: %f", updVars.alt);

    ImGui::EndChild();
}

void makeCmdPanel(HWND hwnd) {
    static WCHAR filePath[MAX_PATH] = L"";

    // Begin Hierachy Panel (Top Right)
    ImGui::SetNextWindowPos(ImVec2(1000, 19), ImGuiCond_Always);
    ImGui::BeginChild("Commands", ImVec2(300, 300), true, ImGuiWindowFlags_None);
    ImGui::TextUnformatted("Commands");
    ImGui::Separator();

    std::string prefix = "CMD";

    // will notice if were connected to server or not and adjust UI accordingly
    _connected = _socket->connected();

    if (!_connected) {
        if (ImGui::Button("Connect to Drone")) {
            _connecting = true;
        }
    }
    else {
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f); // Adjust alpha to make button appear disabled
        ImGui::Button("Connect to Manifold");
        ImGui::PopStyleVar();
    }

    if (ImGui::Button("Set Control Authority")) {
        SetAuthorityMessage msg;
        _socket->Send(msg);
        log("Authority Set\n", prefix);
    }

    if (ImGui::Button("Arm Drone")) {
        ArmMessage msg;
        _droneArmed = !_droneArmed;
        msg.status = _droneArmed;
        _socket->Send(msg);
        log("Drone armed");
    }

    if (ImGui::Button("Take off")) {
        TakeoffMessage msg;
        _socket->Send(msg);
        log("Takeoff Initialized\n", prefix);
    }

    if (ImGui::Button("Land")) {
        LandMessage msg;
        _socket->Send(msg);
        log("Landing Initialized\n", prefix);
    }

    if (ImGui::Button("Manual Input")) {
        _manualInput = !_manualInput;
    }

    if (ImGui::Button("Stop Hover Test")) {
        StopHoverTestMessage msg;
        _socket->Send(msg);
        log("Hover Test Stopped");
    }
    if (ImGui::Button("Stop Test")) {
        StopTestMessage msg;
        _socket->Send(msg);
        log("Test Stopped");
    }

    if (ImGui::Button("Start Capturing imgs")) {
        StartImageCaptureMessage msg;
        _socket->Send(msg);
        log("Image capture initiated!");
    }

    if (ImGui::Button("Stop Capturing imgs")) {
        StopImageCaptureMessage msg;
        _socket->Send(msg);
        log("Image capture initiated!");
    }

	if (ImGui::Button("Upload Flight Path")) {
		std::string gpxPath = OpenFileDialog(hwnd, filePath, L"data.gpx", L"XML Files (*.xml)\0*.xml\0GPX Files (*.gpx)\0*.gpx\0All Files (*.*)\0*.*\0");

		std::ifstream file(gpxPath);
		if (file.is_open()) {
            std::string fileText;
			std::string line;
			while (std::getline(file, line)) {
                    fileText += line;
			}
            UploadFlightPathMessage msg;
            msg.data = fileText;
            _socket->Send(msg);

			file.close();
		}
		else {
				log("failed to open GPX data file.", "ERROR");
		}
	}

    ImGui::EndChild();
}

void makeCamPanel() {
    // Begin Scene Panel (Top Left)
    ImGui::SetNextWindowPos(ImVec2(0, 19), ImGuiCond_Always);
    ImGui::BeginChild("CamFeed", ImVec2(1000, 600), true, ImGuiWindowFlags_None);
    ImGui::TextUnformatted("Camera Feed");
    ImGui::Separator();

    ImVec2 imageSize(1000, 525); // Adjust the size as needed
    ImVec2 imagePos = ImVec2(0, 19);
    ImVec2 uv_min = ImVec2(0.0f, 0.0f); // UV coordinates for the rectangle corners
    ImVec2 uv_max = ImVec2(1.0f, 1.0f);
    ImVec4 tint_col = ImVec4(0.25f, 0.25f, 0.25f, 0.25f); // No tint
    ImVec4 border_col = ImVec4(0.0f, 0.0f, 0.0f, 1.0f); // Black border
    ImGui::Image(nullptr, imageSize, uv_min, uv_max, tint_col, border_col);

    ImGui::EndChild();
}

void makeDebugPanel(HWND hwnd) {
    static WCHAR filePath[MAX_PATH] = L"";
    std::string prefix = "DEBUG";

    // Begin Debug Panel (Bottom Center-ish)
    ImGui::SetNextWindowPos(ImVec2(750, 619), ImGuiCond_Always);
    ImGui::BeginChild("Debug", ImVec2(250, 200), true, ImGuiWindowFlags_None);
    ImGui::TextUnformatted("Debug");
    ImGui::Separator();

    if (ImGui::Button("Export Logs")) {
        log("Exporting logs to: {export_path}...", prefix);
        if (SaveFileDialog(hwnd, filePath, L"data.gpx", L"XML Files (*.xml)\0*.xml\0GPX Files (*.gpx)\0*.gpx\0All Files (*.*)\0*.*\0")) {
            std::wstring selectedFilePath(filePath);
            if (SaveDataToFile(selectedFilePath)) {
                log("Logs succesfully exported", prefix);
            }
            else {
                log("Could not export logs to selected path", "ERROR");
            }
        }
        else {
            log("Log export aborted", prefix);
        }
    }
    ImGui::EndChild();
}

void renderUI(HWND hwnd) 
{
    // Resize the window to fit the directx window.
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos);
    ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size);

    // Begin main window
    ImGui::Begin("Main Window", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);

    // Make the info panels
    makeCamPanel();
    makeCmdPanel(hwnd);
    makePropsPanel();

    makeLogPanel();
    makeDebugPanel(hwnd);

    if (_connecting) {
        makeConnectWindow();
    }
    if (_manualInput) {
        makeManualInputWindow();
    }

    // End main window
    ImGui::End();
}

int main(int, char**)
{
    // Create application window
    //ImGui_ImplWin32_EnableDpiAwareness();
    WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"ImGui Example", nullptr };
    ::RegisterClassExW(&wc);

    // Set window style to non-resizable
    DWORD dwStyle = WS_OVERLAPPEDWINDOW & ~(WS_THICKFRAME | WS_MAXIMIZEBOX);
    HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"NoStoneQuadDrone (NSQD) Client", dwStyle, 100, 100, 1330, 870, nullptr, nullptr, wc.hInstance, nullptr);

    // Initialize Direct3D
    if (!CreateDeviceD3D(hwnd))
    {
        CleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    // Show the window
    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX9_Init(g_pd3dDevice);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    while (!done)
    {
        // Poll and handle messages (inputs, window resize, etc.)
        // See the WndProc() function below for our to dispatch events to the Win32 backend.
        MSG msg;
        while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done)
            break;

        // Handle window resize (we don't resize directly in the WM_SIZE handler)
        if (g_ResizeWidth != 0 && g_ResizeHeight != 0)
        {
            g_d3dpp.BackBufferWidth = g_ResizeWidth;
            g_d3dpp.BackBufferHeight = g_ResizeHeight;
            g_ResizeWidth = g_ResizeHeight = 0;
            ResetDevice();
        }

        // Start the Dear ImGui frame
        ImGui_ImplDX9_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // UI
        renderUI(hwnd);
        _socket->HandleReceive();

        // Rendering
        ImGui::EndFrame();
        g_pd3dDevice->SetRenderState(D3DRS_ZENABLE, FALSE);
        g_pd3dDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
        g_pd3dDevice->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
        D3DCOLOR clear_col_dx = D3DCOLOR_RGBA((int)(clear_color.x * clear_color.w * 255.0f), (int)(clear_color.y * clear_color.w * 255.0f), (int)(clear_color.z * clear_color.w * 255.0f), (int)(clear_color.w * 255.0f));
        g_pd3dDevice->Clear(0, nullptr, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, clear_col_dx, 1.0f, 0);
        if (g_pd3dDevice->BeginScene() >= 0)
        {
            ImGui::Render();
            ImGui_ImplDX9_RenderDrawData(ImGui::GetDrawData());
            g_pd3dDevice->EndScene();
        }
        HRESULT result = g_pd3dDevice->Present(nullptr, nullptr, nullptr, nullptr);

        // Handle loss of D3D9 device
        if (result == D3DERR_DEVICELOST && g_pd3dDevice->TestCooperativeLevel() == D3DERR_DEVICENOTRESET)
            ResetDevice();
    }

    ImGui_ImplDX9_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
    
    delete _socket;

    return 0;
}

// Helper functions

bool CreateDeviceD3D(HWND hWnd)
{
    if ((g_pD3D = Direct3DCreate9(D3D_SDK_VERSION)) == nullptr)
        return false;

    // Create the D3DDevice
    ZeroMemory(&g_d3dpp, sizeof(g_d3dpp));
    g_d3dpp.Windowed = TRUE;
    g_d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
    g_d3dpp.BackBufferFormat = D3DFMT_UNKNOWN; // Need to use an explicit format with alpha if needing per-pixel alpha composition.
    g_d3dpp.EnableAutoDepthStencil = TRUE;
    g_d3dpp.AutoDepthStencilFormat = D3DFMT_D16;
    g_d3dpp.PresentationInterval = D3DPRESENT_INTERVAL_ONE;           // Present with vsync
    //g_d3dpp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;   // Present without vsync, maximum unthrottled framerate
    if (g_pD3D->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &g_d3dpp, &g_pd3dDevice) < 0)
        return false;

    return true;
}

void CleanupDeviceD3D()
{
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
    if (g_pD3D) { g_pD3D->Release(); g_pD3D = nullptr; }
}

void ResetDevice()
{
    ImGui_ImplDX9_InvalidateDeviceObjects();
    HRESULT hr = g_pd3dDevice->Reset(&g_d3dpp);
    if (hr == D3DERR_INVALIDCALL)
        IM_ASSERT(0);
    ImGui_ImplDX9_CreateDeviceObjects();
}

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg)
    {
    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED)
            return 0;
        g_ResizeWidth = (UINT)LOWORD(lParam); // Queue resize
        g_ResizeHeight = (UINT)HIWORD(lParam);
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
            return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}
