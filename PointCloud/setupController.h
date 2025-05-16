#include <SDL3/SDL.h>
#include <QMessageBox>

int setupController(SDL_Gamepad** gamepad) {
    // SDLの初期化
    if (SDL_Init(SDL_INIT_GAMEPAD) < 0) {
        SDL_Log("Failed to initialize SDL.");
        return -1;
    }
    int num_gamepads;
    SDL_JoystickID* gamepads = SDL_GetGamepads(&num_gamepads);
    if (!gamepads) {
        SDL_Log("No gamepads found.");
        QMessageBox FailedMSG;
        FailedMSG.setWindowTitle("PointCloud");
        FailedMSG.setIcon(QMessageBox::Information);
        FailedMSG.setText("Failed to connect controller.");
        FailedMSG.exec();
        return -1;
    }
    // 最初のゲームパッドを開く
    *gamepad = SDL_OpenGamepad(gamepads[0]);
    if (!*gamepad) {
        SDL_Log("Failed to open gamepad: %s", SDL_GetError());
        SDL_free(gamepads);
        SDL_Quit();
        return -1;
    }
    // ゲームパッドの名前を表示
    const char* name = SDL_GetGamepadName(*gamepad);
    SDL_Log("Opened gamepad: %s", name ? name : "Unknown");

    //通知を送信
    QMessageBox SuccessMSG;
    SuccessMSG.setWindowTitle("PointCloud");
    SuccessMSG.setIcon(QMessageBox::Information);
    SuccessMSG.setText("Controller is connected:" + QString(name ? name : "Unknown"));
    SuccessMSG.exec();

    return 0;
}
