#include "LQRControl.hpp"

LQRControl::LQRControl(bool vtol) : MulticopterRateControl(vtol) {
    // Initialize your LQR controller here
    _lqr_controller = new LQRController();
}

LQRControl::~LQRControl() {
    // Clean up any resources if needed
    delete _lqr_controller;
}

void LQRControl::Run() {
    // Main control loop implementation
    while (!should_exit()) {
        // Implement your LQR control logic here for rate control
        _lqr_controller->runControlLoop();

        // Call base class Run() method to handle other functionalities
        MulticopterRateControl::Run();
    }
}
